use core::fmt;
use std::vec::Vec;

use crate::{
    attestation::{AttestationError, AttestationProvider, EvidencePacket, SignedEvidencePacket},
    ekf_core::{
        predict::{PredictError, predict_in_place},
        state::{EskfState, PredictConfig},
    },
    statistical_monitor::{
        monitor::{MonitorError, StatisticalMonitor},
        observation::{MonitorVerdict, TrustLevel},
    },
    telemetry_adapter::{
        MavlinkSubscriber, SynchronizedGpsSample, TelemetryError, TelemetryUpdate,
        purge_frame_buffer,
    },
};

const MAX_SIGNED_EVIDENCE_PACKET_BYTES: usize = 384;

pub trait TelemetrySource {
    fn recv_next(&mut self) -> Result<TelemetryUpdate, TelemetryError>;
    fn record_predicted_state(&mut self, state: &EskfState);
    fn try_dequeue_synchronized_gps(
        &mut self,
    ) -> Result<Option<SynchronizedGpsSample>, TelemetryError>;
}

impl TelemetrySource for MavlinkSubscriber {
    fn recv_next(&mut self) -> Result<TelemetryUpdate, TelemetryError> {
        MavlinkSubscriber::recv_next(self)
    }

    fn record_predicted_state(&mut self, state: &EskfState) {
        MavlinkSubscriber::record_predicted_state(self, state);
    }

    fn try_dequeue_synchronized_gps(
        &mut self,
    ) -> Result<Option<SynchronizedGpsSample>, TelemetryError> {
        MavlinkSubscriber::try_dequeue_synchronized_gps(self)
    }
}

impl<T> TelemetrySource for Box<T>
where
    T: TelemetrySource + ?Sized,
{
    fn recv_next(&mut self) -> Result<TelemetryUpdate, TelemetryError> {
        (**self).recv_next()
    }

    fn record_predicted_state(&mut self, state: &EskfState) {
        (**self).record_predicted_state(state);
    }

    fn try_dequeue_synchronized_gps(
        &mut self,
    ) -> Result<Option<SynchronizedGpsSample>, TelemetryError> {
        (**self).try_dequeue_synchronized_gps()
    }
}

pub trait EvidenceSink {
    fn persist(&mut self, signed_evidence: &SignedEvidencePacket) -> Result<(), EvidenceSinkError>;
}

pub struct FileSink {
    file: std::fs::File,
}

impl FileSink {
    pub fn create<P: AsRef<std::path::Path>>(path: P) -> Result<Self, EvidenceSinkError> {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .map_err(EvidenceSinkError::Io)?;
        Ok(Self { file })
    }
}

impl EvidenceSink for FileSink {
    fn persist(&mut self, signed_evidence: &SignedEvidencePacket) -> Result<(), EvidenceSinkError> {
        use std::io::Write;

        let bytes = serialize_signed_evidence(signed_evidence)?;
        let length_prefix = u32::try_from(bytes.len())
            .map_err(|_| EvidenceSinkError::SerializedPacketTooLarge {
                packet_length: bytes.len(),
            })?
            .to_le_bytes();

        self.file
            .write_all(&length_prefix)
            .map_err(EvidenceSinkError::Io)?;
        self.file.write_all(&bytes).map_err(EvidenceSinkError::Io)?;
        self.file.flush().map_err(EvidenceSinkError::Io)
    }
}

pub struct LogSink<W> {
    writer: W,
}

impl<W> LogSink<W> {
    pub const fn new(writer: W) -> Self {
        Self { writer }
    }
}

impl LogSink<std::io::Stdout> {
    pub fn stdout() -> Self {
        Self::new(std::io::stdout())
    }
}

impl<W> EvidenceSink for LogSink<W>
where
    W: std::io::Write,
{
    fn persist(&mut self, signed_evidence: &SignedEvidencePacket) -> Result<(), EvidenceSinkError> {
        let bytes = serialize_signed_evidence(signed_evidence)?;
        let hex_line = hex_encode(&bytes);
        self.writer
            .write_all(hex_line.as_bytes())
            .map_err(EvidenceSinkError::Io)?;
        self.writer
            .write_all(b"\n")
            .map_err(EvidenceSinkError::Io)?;
        self.writer.flush().map_err(EvidenceSinkError::Io)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MissionReport {
    pub total_packets_processed: u64,
    pub imu_packets_processed: u64,
    pub gps_packets_processed: u64,
    pub verdicts_emitted: u64,
    pub trusted_verdicts: u64,
    pub flagged_verdicts: u64,
    pub rejected_verdicts: u64,
}

impl MissionReport {
    pub fn rejected_fraction(&self) -> f64 {
        if self.verdicts_emitted == 0 {
            0.0
        } else {
            self.rejected_verdicts as f64 / self.verdicts_emitted as f64
        }
    }

    pub fn rejected_percentage(&self) -> f64 {
        self.rejected_fraction() * 100.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StepOutcome {
    pub evidence_emitted: bool,
    pub trust_level: Option<TrustLevel>,
}

impl StepOutcome {
    const fn no_evidence() -> Self {
        Self {
            evidence_emitted: false,
            trust_level: None,
        }
    }

    const fn emitted(trust_level: TrustLevel) -> Self {
        Self {
            evidence_emitted: true,
            trust_level: Some(trust_level),
        }
    }
}

pub struct Orchestrator<T, A, S> {
    telemetry_source: T,
    eskf_state: EskfState,
    predict_config: PredictConfig,
    statistical_monitor: StatisticalMonitor,
    live_warmup_calibration: Option<LiveWarmupCalibrationState>,
    last_monitor_verdict: Option<MonitorVerdict>,
    attestation_provider: A,
    evidence_sink: S,
    mission_report: MissionReport,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveWarmupCalibrationConfig {
    pub warmup_verdicts: usize,
    pub minimum_horizontal_innovation_std_m: f32,
    pub minimum_horizontal_cusum_slack_sigma: f32,
    pub minimum_horizontal_cusum_threshold: f32,
}

impl Default for LiveWarmupCalibrationConfig {
    fn default() -> Self {
        Self {
            warmup_verdicts: 12,
            minimum_horizontal_innovation_std_m: 1.0,
            minimum_horizontal_cusum_slack_sigma: 0.2,
            minimum_horizontal_cusum_threshold: 3.0,
        }
    }
}

impl LiveWarmupCalibrationConfig {
    pub const fn new(warmup_verdicts: usize) -> Self {
        Self {
            warmup_verdicts: if warmup_verdicts == 0 {
                1
            } else {
                warmup_verdicts
            },
            minimum_horizontal_innovation_std_m: 1.0,
            minimum_horizontal_cusum_slack_sigma: 0.2,
            minimum_horizontal_cusum_threshold: 3.0,
        }
    }

    pub const fn with_minimum_horizontal_innovation_std_m(
        mut self,
        minimum_horizontal_innovation_std_m: f32,
    ) -> Self {
        self.minimum_horizontal_innovation_std_m = minimum_horizontal_innovation_std_m;
        self
    }

    pub const fn with_minimum_horizontal_cusum_slack_sigma(
        mut self,
        minimum_horizontal_cusum_slack_sigma: f32,
    ) -> Self {
        self.minimum_horizontal_cusum_slack_sigma = minimum_horizontal_cusum_slack_sigma;
        self
    }

    pub const fn with_minimum_horizontal_cusum_threshold(
        mut self,
        minimum_horizontal_cusum_threshold: f32,
    ) -> Self {
        self.minimum_horizontal_cusum_threshold = minimum_horizontal_cusum_threshold;
        self
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveWarmupCalibrationReport {
    pub warmup_verdicts: usize,
    pub horizontal_innovation_std_m: f32,
    pub horizontal_cusum_slack_sigma: f32,
    pub horizontal_cusum_threshold: f32,
}

#[derive(Clone, Debug, PartialEq)]
struct LiveWarmupCalibrationState {
    config: LiveWarmupCalibrationConfig,
    horizontal_residual_samples_m: Vec<f32>,
    report: Option<LiveWarmupCalibrationReport>,
}

impl LiveWarmupCalibrationState {
    fn new(config: LiveWarmupCalibrationConfig) -> Self {
        Self {
            config,
            horizontal_residual_samples_m: Vec::with_capacity(config.warmup_verdicts),
            report: None,
        }
    }

    fn is_complete(&self) -> bool {
        self.report.is_some()
    }

    fn report(&self) -> Option<LiveWarmupCalibrationReport> {
        self.report
    }

    fn warmup_samples_collected(&self) -> usize {
        self.horizontal_residual_samples_m.len()
    }

    fn record_horizontal_residual_m(
        &mut self,
        horizontal_residual_m: f32,
    ) -> Option<LiveWarmupCalibrationReport> {
        if self.report.is_some() {
            return self.report;
        }

        self.horizontal_residual_samples_m
            .push(horizontal_residual_m);
        if self.horizontal_residual_samples_m.len() < self.config.warmup_verdicts {
            return None;
        }

        let horizontal_innovation_std_m = sample_std_scalar(&self.horizontal_residual_samples_m)
            .max(self.config.minimum_horizontal_innovation_std_m)
            .max(1.0e-6);
        let mut normalized_horizontal_residuals = self
            .horizontal_residual_samples_m
            .iter()
            .map(|horizontal_residual_m| horizontal_residual_m / horizontal_innovation_std_m)
            .collect::<Vec<_>>();
        normalized_horizontal_residuals.sort_by(|left, right| left.total_cmp(right));

        let horizontal_cusum_slack_sigma =
            percentile_sorted(&normalized_horizontal_residuals, 0.95)
                .unwrap_or(self.config.minimum_horizontal_cusum_slack_sigma)
                .max(self.config.minimum_horizontal_cusum_slack_sigma)
                .max(0.0);
        let mut score = 0.0_f32;
        let mut clean_score_ceiling = 0.0_f32;
        for normalized_horizontal_residual in normalized_horizontal_residuals {
            score =
                (score + normalized_horizontal_residual - horizontal_cusum_slack_sigma).max(0.0);
            clean_score_ceiling = clean_score_ceiling.max(score);
        }
        let horizontal_cusum_threshold = (clean_score_ceiling + horizontal_cusum_slack_sigma)
            .max(self.config.minimum_horizontal_cusum_threshold);
        let report = LiveWarmupCalibrationReport {
            warmup_verdicts: self.config.warmup_verdicts,
            horizontal_innovation_std_m,
            horizontal_cusum_slack_sigma,
            horizontal_cusum_threshold,
        };
        self.report = Some(report);
        self.report
    }
}

impl<T, A, S> Orchestrator<T, A, S>
where
    T: TelemetrySource,
    A: AttestationProvider,
    S: EvidenceSink,
{
    pub fn new(
        telemetry_source: T,
        eskf_state: EskfState,
        predict_config: PredictConfig,
        statistical_monitor: StatisticalMonitor,
        attestation_provider: A,
        evidence_sink: S,
    ) -> Self {
        Self {
            telemetry_source,
            eskf_state,
            predict_config,
            statistical_monitor,
            live_warmup_calibration: None,
            last_monitor_verdict: None,
            attestation_provider,
            evidence_sink,
            mission_report: MissionReport::default(),
        }
    }

    pub fn with_live_warmup_calibration(mut self, config: LiveWarmupCalibrationConfig) -> Self {
        self.live_warmup_calibration = Some(LiveWarmupCalibrationState::new(config));
        self
    }

    pub fn mission_report(&self) -> MissionReport {
        self.mission_report
    }

    pub fn eskf_state(&self) -> &EskfState {
        &self.eskf_state
    }

    pub fn live_warmup_calibration_report(&self) -> Option<LiveWarmupCalibrationReport> {
        self.live_warmup_calibration
            .as_ref()
            .and_then(LiveWarmupCalibrationState::report)
    }

    pub fn live_warmup_samples_collected(&self) -> Option<usize> {
        self.live_warmup_calibration
            .as_ref()
            .map(LiveWarmupCalibrationState::warmup_samples_collected)
    }

    pub fn last_monitor_verdict(&self) -> Option<&MonitorVerdict> {
        self.last_monitor_verdict.as_ref()
    }

    pub fn step(&mut self) -> Result<StepOutcome, OrchestratorError> {
        let telemetry_update = self
            .telemetry_source
            .recv_next()
            .map_err(OrchestratorError::Telemetry)?;
        self.mission_report.total_packets_processed += 1;

        match telemetry_update {
            TelemetryUpdate::Imu {
                sample,
                mut raw_frame,
            } => {
                self.mission_report.imu_packets_processed += 1;
                let result = (|| {
                    predict_in_place(&mut self.eskf_state, &self.predict_config, &sample)
                        .map_err(OrchestratorError::Predict)?;
                    self.telemetry_source
                        .record_predicted_state(&self.eskf_state);
                    self.try_finalize_pending_verdict()
                })();
                purge_frame_buffer(&mut raw_frame);
                result
            }
            TelemetryUpdate::GpsObservationQueued { .. } => {
                self.mission_report.gps_packets_processed += 1;
                self.try_finalize_pending_verdict()
            }
        }
    }

    fn try_finalize_pending_verdict(&mut self) -> Result<StepOutcome, OrchestratorError> {
        let Some(mut synchronized_gps_sample) = self
            .telemetry_source
            .try_dequeue_synchronized_gps()
            .map_err(OrchestratorError::Telemetry)?
        else {
            return Ok(StepOutcome::no_evidence());
        };

        let result = (|| {
            if self
                .live_warmup_calibration
                .as_ref()
                .is_some_and(|state| !state.is_complete())
            {
                let mut preview_monitor = self.statistical_monitor;
                let mut monitor_verdict = preview_monitor
                    .evaluate_observations(
                        &synchronized_gps_sample.aligned_predicted_state,
                        &synchronized_gps_sample.gps_observation,
                        synchronized_gps_sample.barometer_observation.as_ref(),
                        synchronized_gps_sample.heading_observation.as_ref(),
                    )
                    .map_err(OrchestratorError::Monitor)?;
                let horizontal_residual_m = monitor_verdict.innovation.fixed_rows::<2>(0).norm();
                let calibration_report = self
                    .live_warmup_calibration
                    .as_mut()
                    .and_then(|state| state.record_horizontal_residual_m(horizontal_residual_m));
                if let Some(calibration_report) = calibration_report {
                    self.statistical_monitor.set_horizontal_residual_persistence(Some(
                        crate::statistical_monitor::monitor::HorizontalResidualPersistenceConfig::new(
                            calibration_report.horizontal_cusum_slack_sigma,
                            calibration_report.horizontal_cusum_threshold,
                        ),
                    ));
                    self.statistical_monitor
                        .set_horizontal_residual_normalization_std_override_m(Some(
                            calibration_report.horizontal_innovation_std_m,
                        ));
                    self.statistical_monitor.reset_runtime_state();
                }

                monitor_verdict.trust_level = TrustLevel::Trusted;
                self.last_monitor_verdict = Some(monitor_verdict.clone());
                let evidence_packet = EvidencePacket::from_synchronized_sample(
                    synchronized_gps_sample.timestamp_ns,
                    synchronized_gps_sample.raw_frame.as_slice(),
                    &synchronized_gps_sample,
                    &monitor_verdict,
                );
                let signed_evidence = self
                    .attestation_provider
                    .sign_evidence(&evidence_packet)
                    .map_err(OrchestratorError::Attestation)?;
                self.evidence_sink
                    .persist(&signed_evidence)
                    .map_err(OrchestratorError::EvidenceSink)?;
                self.record_verdict(TrustLevel::Trusted);
                return Ok(StepOutcome::emitted(TrustLevel::Trusted));
            }

            let monitor_verdict = self
                .statistical_monitor
                .evaluate_observations(
                    &synchronized_gps_sample.aligned_predicted_state,
                    &synchronized_gps_sample.gps_observation,
                    synchronized_gps_sample.barometer_observation.as_ref(),
                    synchronized_gps_sample.heading_observation.as_ref(),
                )
                .map_err(OrchestratorError::Monitor)?;
            let trust_level = monitor_verdict.trust_level;
            self.last_monitor_verdict = Some(monitor_verdict.clone());
            let evidence_packet = EvidencePacket::from_synchronized_sample(
                synchronized_gps_sample.timestamp_ns,
                synchronized_gps_sample.raw_frame.as_slice(),
                &synchronized_gps_sample,
                &monitor_verdict,
            );
            let signed_evidence = self
                .attestation_provider
                .sign_evidence(&evidence_packet)
                .map_err(OrchestratorError::Attestation)?;
            self.evidence_sink
                .persist(&signed_evidence)
                .map_err(OrchestratorError::EvidenceSink)?;
            self.record_verdict(trust_level);
            Ok(StepOutcome::emitted(trust_level))
        })();

        purge_frame_buffer(&mut synchronized_gps_sample.raw_frame);
        result
    }

    fn record_verdict(&mut self, trust_level: TrustLevel) {
        self.mission_report.verdicts_emitted += 1;
        match trust_level {
            TrustLevel::Trusted => self.mission_report.trusted_verdicts += 1,
            TrustLevel::Flagged => self.mission_report.flagged_verdicts += 1,
            TrustLevel::Rejected => self.mission_report.rejected_verdicts += 1,
        }
    }
}

#[derive(Debug)]
pub enum OrchestratorError {
    Telemetry(TelemetryError),
    Predict(PredictError),
    Monitor(MonitorError),
    Attestation(AttestationError),
    EvidenceSink(EvidenceSinkError),
}

impl fmt::Display for OrchestratorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Telemetry(error) => write!(f, "telemetry adapter error: {error}"),
            Self::Predict(error) => write!(f, "EKF propagation error: {error}"),
            Self::Monitor(error) => write!(f, "statistical monitor error: {error}"),
            Self::Attestation(error) => write!(f, "attestation error: {error}"),
            Self::EvidenceSink(error) => write!(f, "evidence sink error: {error}"),
        }
    }
}

#[derive(Debug)]
pub enum EvidenceSinkError {
    Serialization(postcard::Error),
    Io(std::io::Error),
    SerializedPacketTooLarge { packet_length: usize },
    EvidenceRoundTrip(AttestationError),
}

impl fmt::Display for EvidenceSinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serialization(error) => write!(f, "postcard serialization failed: {error:?}"),
            Self::Io(error) => write!(f, "I/O failure while persisting evidence: {error}"),
            Self::SerializedPacketTooLarge { packet_length } => write!(
                f,
                "serialized evidence packet length {packet_length} exceeds sink framing limits"
            ),
            Self::EvidenceRoundTrip(error) => {
                write!(f, "signed evidence failed round-trip validation: {error}")
            }
        }
    }
}

fn serialize_signed_evidence(
    signed_evidence: &SignedEvidencePacket,
) -> Result<Vec<u8>, EvidenceSinkError> {
    let mut buffer = [0_u8; MAX_SIGNED_EVIDENCE_PACKET_BYTES];
    postcard::to_slice(signed_evidence, &mut buffer)
        .map(|encoded| encoded.to_vec())
        .map_err(EvidenceSinkError::Serialization)
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for &byte in bytes {
        output.push(char::from(HEX[(byte >> 4) as usize]));
        output.push(char::from(HEX[(byte & 0x0F) as usize]));
    }
    output
}

fn percentile_sorted(values: &[f32], quantile: f32) -> Option<f32> {
    if values.is_empty() {
        return None;
    }

    let clamped_quantile = quantile.clamp(0.0, 1.0);
    let index = ((values.len() - 1) as f32 * clamped_quantile).floor() as usize;
    values.get(index).copied()
}

fn sample_std_scalar(values: &[f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }

    let mean = values.iter().sum::<f32>() / values.len() as f32;
    let variance = values
        .iter()
        .map(|value| {
            let centered = *value - mean;
            centered * centered
        })
        .sum::<f32>()
        / values.len() as f32;

    variance.sqrt()
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use heapless::Vec as HeaplessVec;
    use nalgebra::{UnitQuaternion, Vector3};

    use super::{
        EvidencePacket, EvidenceSink, EvidenceSinkError, LiveWarmupCalibrationConfig,
        LiveWarmupCalibrationState, Orchestrator, StepOutcome, TelemetrySource,
    };
    use crate::{
        attestation::{AttestationProvider, Ed25519AttestationProvider, MockSecureElement},
        ekf_core::state::{
            EskfState, ImuNoiseModel, ImuSample, NominalState, PredictConfig, StateCovariance,
        },
        statistical_monitor::{
            monitor::{EwmaRiskAccumulator, StatisticalMonitor},
            observation::{ChiSquareThresholdConfig, GpsObservation, TrustLevel},
        },
        telemetry_adapter::{
            MavlinkFrameBuffer, SynchronizedGpsSample, TelemetryError, TelemetryUpdate,
        },
    };

    struct RecordingSink {
        packets: Vec<Vec<u8>>,
    }

    impl RecordingSink {
        fn new() -> Self {
            Self {
                packets: Vec::new(),
            }
        }
    }

    impl EvidenceSink for RecordingSink {
        fn persist(
            &mut self,
            signed_evidence: &crate::attestation::SignedEvidencePacket,
        ) -> Result<(), EvidenceSinkError> {
            let mut buffer = [0_u8; super::MAX_SIGNED_EVIDENCE_PACKET_BYTES];
            let encoded = postcard::to_slice(signed_evidence, &mut buffer)
                .map_err(EvidenceSinkError::Serialization)?;
            self.packets.push(encoded.to_vec());
            Ok(())
        }
    }

    struct SyntheticTelemetrySource {
        events: VecDeque<TelemetryUpdate>,
        pending_gps_samples: VecDeque<SynchronizedGpsSample>,
        gps_ready: bool,
        has_recorded_state: bool,
    }

    impl SyntheticTelemetrySource {
        fn new(
            events: VecDeque<TelemetryUpdate>,
            pending_gps_samples: VecDeque<SynchronizedGpsSample>,
        ) -> Self {
            Self {
                events,
                pending_gps_samples,
                gps_ready: false,
                has_recorded_state: false,
            }
        }
    }

    impl TelemetrySource for SyntheticTelemetrySource {
        fn recv_next(&mut self) -> Result<TelemetryUpdate, TelemetryError> {
            let event = self
                .events
                .pop_front()
                .ok_or(TelemetryError::BufferOverflow)?;
            if matches!(event, TelemetryUpdate::GpsObservationQueued { .. }) {
                self.gps_ready = true;
            }
            Ok(event)
        }

        fn record_predicted_state(&mut self, state: &EskfState) {
            self.has_recorded_state = true;
            if let Some(pending_sample) = self.pending_gps_samples.front_mut() {
                pending_sample.aligned_predicted_state = state.clone();
            }
        }

        fn try_dequeue_synchronized_gps(
            &mut self,
        ) -> Result<Option<SynchronizedGpsSample>, TelemetryError> {
            if !self.gps_ready || !self.has_recorded_state {
                return Ok(None);
            }

            self.gps_ready = false;
            Ok(self.pending_gps_samples.pop_front())
        }
    }

    #[test]
    fn full_loop_processes_signs_and_reports_rejection() {
        let imu_sample = ImuSample::new(0.01, Vector3::new(0.0, 0.0, -9.80665), Vector3::zeros());
        let mut imu_frame = MavlinkFrameBuffer::new();
        imu_frame
            .extend_from_slice(&[0xFD, 0x15, 0x01, 0x69])
            .unwrap();

        let mut gps_frame = MavlinkFrameBuffer::new();
        gps_frame
            .extend_from_slice(&[0xFD, 0x21, 0x02, 0x33, 0x44, 0x55])
            .unwrap();

        let events = VecDeque::from([
            TelemetryUpdate::Imu {
                sample: imu_sample,
                raw_frame: imu_frame,
            },
            TelemetryUpdate::GpsObservationQueued {
                timestamp_s: 0.01,
                queue_depth: 1,
            },
        ]);
        let pending_gps_sample = SynchronizedGpsSample {
            timestamp_ns: 1_725_897_123_456_789_000,
            gps_observation: GpsObservation::from_accuracy_metrics(
                0.01,
                Vector3::new(120.0, -85.0, 18.0),
                Vector3::new(12.0, -7.0, 2.0),
                1.5,
                2.0,
                0.4,
                0.5,
            ),
            barometer_observation: None,
            heading_observation: None,
            aligned_predicted_state: EskfState::default(),
            raw_frame: gps_frame,
        };
        let pending_gps_samples = VecDeque::from([pending_gps_sample]);
        let telemetry_source = SyntheticTelemetrySource::new(events, pending_gps_samples);

        let initial_state = EskfState::new(
            NominalState {
                timestamp_s: 0.0,
                position_ned_m: Vector3::zeros(),
                velocity_ned_mps: Vector3::zeros(),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            StateCovariance::identity() * 1.0e-3,
        );
        let predict_config = PredictConfig::new(
            Vector3::new(0.0, 0.0, 9.80665),
            0.02,
            ImuNoiseModel::new(
                Vector3::new(0.05, 0.05, 0.05),
                Vector3::new(0.002, 0.002, 0.002),
                Vector3::new(0.0002, 0.0002, 0.0002),
                Vector3::new(0.00002, 0.00002, 0.00002),
            ),
        );
        let monitor = StatisticalMonitor::new(
            ChiSquareThresholdConfig::new(12.592, 22.458),
            EwmaRiskAccumulator::new(1.0),
        );
        let secure_element = MockSecureElement::from_secret_key_bytes([7_u8; 32]);
        let attestation_provider = Ed25519AttestationProvider::new(secure_element);
        let evidence_sink = RecordingSink::new();

        let mut orchestrator = Orchestrator::new(
            telemetry_source,
            initial_state,
            predict_config,
            monitor,
            attestation_provider,
            evidence_sink,
        );

        let first_step = orchestrator.step().unwrap();
        assert_eq!(first_step, StepOutcome::no_evidence());

        let second_step = orchestrator.step().unwrap();
        assert_eq!(second_step.trust_level, Some(TrustLevel::Rejected));
        assert!(second_step.evidence_emitted);

        let report = orchestrator.mission_report();
        assert_eq!(report.total_packets_processed, 2);
        assert_eq!(report.imu_packets_processed, 1);
        assert_eq!(report.gps_packets_processed, 1);
        assert_eq!(report.verdicts_emitted, 1);
        assert_eq!(report.rejected_verdicts, 1);
        assert_eq!(report.rejected_percentage(), 100.0);
    }

    #[test]
    fn gps_before_first_imu_waits_for_state_before_emitting_evidence() {
        let mut gps_frame = MavlinkFrameBuffer::new();
        gps_frame
            .extend_from_slice(&[0xFD, 0x21, 0x02, 0x33, 0x44, 0x55])
            .unwrap();
        let mut imu_frame = MavlinkFrameBuffer::new();
        imu_frame
            .extend_from_slice(&[0xFD, 0x15, 0x01, 0x69])
            .unwrap();

        let events = VecDeque::from([
            TelemetryUpdate::GpsObservationQueued {
                timestamp_s: 0.01,
                queue_depth: 1,
            },
            TelemetryUpdate::Imu {
                sample: ImuSample::new(0.01, Vector3::new(0.0, 0.0, -9.80665), Vector3::zeros()),
                raw_frame: imu_frame,
            },
        ]);
        let pending_gps_samples = VecDeque::from([SynchronizedGpsSample {
            timestamp_ns: 1_725_897_123_456_789_000,
            gps_observation: GpsObservation::from_accuracy_metrics(
                0.01,
                Vector3::new(120.0, -85.0, 18.0),
                Vector3::new(12.0, -7.0, 2.0),
                1.5,
                2.0,
                0.4,
                0.5,
            ),
            barometer_observation: None,
            heading_observation: None,
            aligned_predicted_state: EskfState::default(),
            raw_frame: gps_frame,
        }]);
        let telemetry_source = SyntheticTelemetrySource::new(events, pending_gps_samples);

        let initial_state = EskfState::new(
            NominalState {
                timestamp_s: 0.0,
                position_ned_m: Vector3::zeros(),
                velocity_ned_mps: Vector3::zeros(),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            StateCovariance::identity() * 1.0e-3,
        );
        let predict_config = PredictConfig::new(
            Vector3::new(0.0, 0.0, 9.80665),
            0.02,
            ImuNoiseModel::new(
                Vector3::new(0.05, 0.05, 0.05),
                Vector3::new(0.002, 0.002, 0.002),
                Vector3::new(0.0002, 0.0002, 0.0002),
                Vector3::new(0.00002, 0.00002, 0.00002),
            ),
        );
        let monitor = StatisticalMonitor::new(
            ChiSquareThresholdConfig::new(12.592, 22.458),
            EwmaRiskAccumulator::new(1.0),
        );
        let secure_element = MockSecureElement::from_secret_key_bytes([8_u8; 32]);
        let attestation_provider = Ed25519AttestationProvider::new(secure_element);
        let evidence_sink = RecordingSink::new();

        let mut orchestrator = Orchestrator::new(
            telemetry_source,
            initial_state,
            predict_config,
            monitor,
            attestation_provider,
            evidence_sink,
        );

        let first_step = orchestrator.step().unwrap();
        assert_eq!(first_step, StepOutcome::no_evidence());
        assert_eq!(orchestrator.mission_report().verdicts_emitted, 0);

        let second_step = orchestrator.step().unwrap();
        assert!(second_step.evidence_emitted);
        assert_eq!(orchestrator.mission_report().verdicts_emitted, 1);
    }

    #[test]
    fn recorded_evidence_round_trips_through_postcard() {
        let secure_element = MockSecureElement::from_secret_key_bytes([11_u8; 32]);
        let provider = Ed25519AttestationProvider::new(secure_element);

        let predicted_state = EskfState::new(
            NominalState {
                timestamp_s: 1.0,
                position_ned_m: Vector3::new(1.0, 2.0, 3.0),
                velocity_ned_mps: Vector3::new(0.2, -0.1, 0.0),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            StateCovariance::identity(),
        );
        let sample = SynchronizedGpsSample {
            timestamp_ns: 1_700_000_000_000_000_000,
            gps_observation: GpsObservation::from_accuracy_metrics(
                1.0,
                Vector3::new(1.0, 2.0, 3.0),
                Vector3::zeros(),
                1.0,
                1.0,
                0.2,
                0.2,
            ),
            barometer_observation: None,
            heading_observation: None,
            aligned_predicted_state: predicted_state,
            raw_frame: {
                let mut frame = HeaplessVec::new();
                frame.extend_from_slice(&[0xFD, 0x42, 0x01]).unwrap();
                frame
            },
        };
        let verdict = crate::statistical_monitor::observation::MonitorVerdict {
            squared_mahalanobis_distance: 0.5,
            gps_squared_mahalanobis_distance: 0.5,
            barometer_squared_mahalanobis_distance: None,
            heading_squared_mahalanobis_distance: None,
            clock_bias_squared_mahalanobis_distance: None,
            clock_bias_persistent_score: None,
            horizontal_residual_persistent_score: None,
            accumulated_risk: 0.5,
            innovation: crate::statistical_monitor::observation::ObservationVector::zeros(),
            barometer_residual_m: None,
            heading_residual_rad: None,
            clock_bias_residual_m: None,
            trust_level: TrustLevel::Trusted,
        };
        let evidence = EvidencePacket::from_synchronized_sample(
            sample.timestamp_ns,
            sample.raw_frame.as_slice(),
            &sample,
            &verdict,
        );
        let signed = provider.sign_evidence(&evidence).unwrap();

        let mut sink = RecordingSink::new();
        sink.persist(&signed).unwrap();
        let restored = crate::attestation::deserialize_evidence(&sink.packets[0]).unwrap();

        assert_eq!(restored.evidence.timestamp_ns, signed.evidence.timestamp_ns);
        assert_eq!(restored.public_key, signed.public_key);
    }

    #[test]
    fn live_warmup_calibration_uses_conservative_floors() {
        let mut state = LiveWarmupCalibrationState::new(
            LiveWarmupCalibrationConfig::new(4)
                .with_minimum_horizontal_innovation_std_m(1.0)
                .with_minimum_horizontal_cusum_slack_sigma(0.2)
                .with_minimum_horizontal_cusum_threshold(3.0),
        );

        assert!(state.record_horizontal_residual_m(0.01).is_none());
        assert!(state.record_horizontal_residual_m(0.02).is_none());
        assert!(state.record_horizontal_residual_m(0.03).is_none());
        let report = state.record_horizontal_residual_m(0.04).unwrap();

        assert_eq!(report.warmup_verdicts, 4);
        assert_eq!(report.horizontal_innovation_std_m, 1.0);
        assert!(report.horizontal_cusum_slack_sigma >= 0.2);
        assert!(report.horizontal_cusum_threshold >= 3.0);
    }

    #[test]
    fn live_warmup_zero_variance_is_guarded() {
        let mut state = LiveWarmupCalibrationState::new(
            LiveWarmupCalibrationConfig::new(3)
                .with_minimum_horizontal_innovation_std_m(1.0)
                .with_minimum_horizontal_cusum_slack_sigma(0.2)
                .with_minimum_horizontal_cusum_threshold(3.0),
        );

        assert!(state.record_horizontal_residual_m(0.0).is_none());
        assert!(state.record_horizontal_residual_m(0.0).is_none());
        let report = state.record_horizontal_residual_m(0.0).unwrap();

        assert_eq!(report.horizontal_innovation_std_m, 1.0);
        assert_eq!(report.horizontal_cusum_slack_sigma, 0.2);
        assert_eq!(report.horizontal_cusum_threshold, 3.0);
    }
}
