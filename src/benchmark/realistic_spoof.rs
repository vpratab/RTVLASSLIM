use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::MonitorDatasetRow;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Eq, PartialEq)]
pub enum DirectionMode {
    North,
    East,
    South,
    West,
    AlongTrack,
    CrossTrackRight,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Eq, PartialEq)]
pub enum ProfileShape {
    RampHold,
    IntermittentRampHold,
    FreezePosition,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RealisticSpoofCase {
    pub label: String,
    pub family: String,
    pub source_basis: String,
    pub description: String,
    pub shape: ProfileShape,
    pub direction_mode: DirectionMode,
    pub onset_time_s: f64,
    pub ramp_duration_s: f64,
    pub final_position_offset_m: f32,
    pub final_velocity_offset_mps: f32,
    pub final_clock_bias_m: f32,
    pub intermittent_period_s: Option<f64>,
    pub intermittent_duty_cycle: Option<f64>,
}

impl RealisticSpoofCase {
    #[allow(clippy::too_many_arguments)]
    pub fn ramp_hold(
        label: impl Into<String>,
        family: impl Into<String>,
        source_basis: impl Into<String>,
        description: impl Into<String>,
        direction_mode: DirectionMode,
        onset_time_s: f64,
        ramp_duration_s: f64,
        final_position_offset_m: f32,
        final_velocity_offset_mps: f32,
        final_clock_bias_m: f32,
    ) -> Self {
        Self {
            label: label.into(),
            family: family.into(),
            source_basis: source_basis.into(),
            description: description.into(),
            shape: ProfileShape::RampHold,
            direction_mode,
            onset_time_s,
            ramp_duration_s,
            final_position_offset_m,
            final_velocity_offset_mps,
            final_clock_bias_m,
            intermittent_period_s: None,
            intermittent_duty_cycle: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn intermittent(
        label: impl Into<String>,
        family: impl Into<String>,
        source_basis: impl Into<String>,
        description: impl Into<String>,
        direction_mode: DirectionMode,
        onset_time_s: f64,
        ramp_duration_s: f64,
        final_position_offset_m: f32,
        final_velocity_offset_mps: f32,
        final_clock_bias_m: f32,
        intermittent_period_s: f64,
        intermittent_duty_cycle: f64,
    ) -> Self {
        Self {
            label: label.into(),
            family: family.into(),
            source_basis: source_basis.into(),
            description: description.into(),
            shape: ProfileShape::IntermittentRampHold,
            direction_mode,
            onset_time_s,
            ramp_duration_s,
            final_position_offset_m,
            final_velocity_offset_mps,
            final_clock_bias_m,
            intermittent_period_s: Some(intermittent_period_s),
            intermittent_duty_cycle: Some(intermittent_duty_cycle),
        }
    }

    pub fn freeze_position(
        label: impl Into<String>,
        family: impl Into<String>,
        source_basis: impl Into<String>,
        description: impl Into<String>,
        onset_time_s: f64,
    ) -> Self {
        Self {
            label: label.into(),
            family: family.into(),
            source_basis: source_basis.into(),
            description: description.into(),
            shape: ProfileShape::FreezePosition,
            direction_mode: DirectionMode::AlongTrack,
            onset_time_s,
            ramp_duration_s: 0.0,
            final_position_offset_m: 0.0,
            final_velocity_offset_mps: 0.0,
            final_clock_bias_m: 0.0,
            intermittent_period_s: None,
            intermittent_duty_cycle: None,
        }
    }
}

pub fn built_in_realistic_spoof_cases() -> Vec<RealisticSpoofCase> {
    vec![
        RealisticSpoofCase::ramp_hold(
            "texbat_ds1_static_takeover",
            "TEXBAT-like",
            "TEXBAT static receiver spoofing battery category",
            "abrupt navigation-solution takeover with a large horizontal offset",
            DirectionMode::North,
            2.0,
            0.0,
            80.0,
            8.0,
            0.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "texbat_ds2_overpowered_time_push",
            "TEXBAT-like",
            "TEXBAT ds2-style overpowered carry-off/time-push category",
            "position and receiver-clock error ramp together after takeover",
            DirectionMode::AlongTrack,
            2.0,
            20.0,
            25.0,
            1.5,
            180.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "texbat_ds3_matched_power_slow_carryoff",
            "TEXBAT-like",
            "TEXBAT ds3-style gradual low-magnitude carry-off category",
            "slow coherent position and clock drift intended to stay under one-shot thresholds",
            DirectionMode::CrossTrackRight,
            2.0,
            40.0,
            30.0,
            0.75,
            80.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "texbat_ds7_phase_aligned_time_push",
            "TEXBAT-like",
            "TEXBAT ds7-style subtle phase-aligned time-push category",
            "small position drift paired with persistent receiver-clock bias",
            DirectionMode::AlongTrack,
            2.0,
            60.0,
            15.0,
            0.25,
            45.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "uav_sdr_takeover_30m_10s",
            "UAV attack dataset-like",
            "public UAV attack dataset / GPS-SDR-SIM + SDR spoofing category",
            "moderate carry-off similar to a software-defined spoofer takeover",
            DirectionMode::East,
            2.0,
            10.0,
            30.0,
            3.0,
            0.0,
        ),
        RealisticSpoofCase::freeze_position(
            "uav_freeze_or_hold_last_fix",
            "UAV attack dataset-like",
            "GPS disruption followed by stale or held navigation solution category",
            "GPS position is frozen at spoof onset while the inertial reference keeps moving",
            2.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "nav_wrong_turn_cross_track",
            "navigation-deception",
            "wrong-turn / turn-by-turn navigation spoofing category",
            "cross-track displacement that would pull guidance toward a wrong turn",
            DirectionMode::CrossTrackRight,
            2.0,
            15.0,
            45.0,
            2.0,
            0.0,
        ),
        RealisticSpoofCase::ramp_hold(
            "nav_overshoot_along_track",
            "navigation-deception",
            "overshoot / route-progress manipulation category",
            "along-track displacement that makes the vehicle believe it is farther along route",
            DirectionMode::AlongTrack,
            2.0,
            8.0,
            60.0,
            5.0,
            0.0,
        ),
        RealisticSpoofCase::intermittent(
            "intermittent_pulsed_carryoff",
            "adaptive-spoofer",
            "intermittent spoofing category used to probe detector reset behavior",
            "carry-off is pulsed on and off to test whether persistence survives gaps",
            DirectionMode::North,
            2.0,
            20.0,
            35.0,
            1.0,
            60.0,
            6.0,
            0.5,
        ),
    ]
}

pub fn apply_realistic_spoof_case(
    rows: &[MonitorDatasetRow],
    case: &RealisticSpoofCase,
) -> Vec<MonitorDatasetRow> {
    let mut frozen_position_ned_m = None;
    let start_time_s = rows.first().map(|row| row.timestamp_s).unwrap_or(0.0);

    rows.iter()
        .cloned()
        .map(|mut row| {
            let elapsed_time_s = row.timestamp_s - start_time_s;
            match case.shape {
                ProfileShape::RampHold | ProfileShape::IntermittentRampHold => {
                    let active_scale = active_profile_scale(elapsed_time_s, case);
                    if active_scale > 0.0 {
                        let direction = direction_vector(&row, case.direction_mode);
                        row.gps_px_ned_m +=
                            direction.x * case.final_position_offset_m * active_scale;
                        row.gps_py_ned_m +=
                            direction.y * case.final_position_offset_m * active_scale;
                        row.gps_pz_ned_m +=
                            direction.z * case.final_position_offset_m * active_scale;
                        row.gps_vx_ned_mps +=
                            direction.x * case.final_velocity_offset_mps * active_scale;
                        row.gps_vy_ned_mps +=
                            direction.y * case.final_velocity_offset_mps * active_scale;
                        row.gps_vz_ned_mps +=
                            direction.z * case.final_velocity_offset_mps * active_scale;
                        inject_clock_bias(&mut row, case.final_clock_bias_m * active_scale);
                        row.label_spoofed = true;
                    } else {
                        row.label_spoofed = false;
                    }
                }
                ProfileShape::FreezePosition => {
                    if elapsed_time_s >= case.onset_time_s {
                        let frozen = *frozen_position_ned_m.get_or_insert(Vector3::new(
                            row.gps_px_ned_m,
                            row.gps_py_ned_m,
                            row.gps_pz_ned_m,
                        ));
                        row.gps_px_ned_m = frozen.x;
                        row.gps_py_ned_m = frozen.y;
                        row.gps_pz_ned_m = frozen.z;
                        row.gps_vx_ned_mps = 0.0;
                        row.gps_vy_ned_mps = 0.0;
                        row.gps_vz_ned_mps = 0.0;
                        row.label_spoofed = true;
                    } else {
                        row.label_spoofed = false;
                    }
                }
            }
            row
        })
        .collect()
}

fn active_profile_scale(elapsed_time_s: f64, case: &RealisticSpoofCase) -> f32 {
    if elapsed_time_s < case.onset_time_s {
        return 0.0;
    }

    if case.shape == ProfileShape::IntermittentRampHold
        && !intermittent_phase_active(elapsed_time_s, case)
    {
        return 0.0;
    }

    if case.ramp_duration_s <= 0.0 {
        1.0
    } else {
        ((elapsed_time_s - case.onset_time_s) / case.ramp_duration_s).clamp(0.0, 1.0) as f32
    }
}

fn intermittent_phase_active(elapsed_time_s: f64, case: &RealisticSpoofCase) -> bool {
    let Some(period_s) = case.intermittent_period_s else {
        return true;
    };
    let duty_cycle = case.intermittent_duty_cycle.unwrap_or(1.0).clamp(0.0, 1.0);
    if period_s <= f64::EPSILON || duty_cycle >= 1.0 {
        return true;
    }
    let phase = (elapsed_time_s - case.onset_time_s).rem_euclid(period_s);
    phase <= period_s * duty_cycle
}

fn direction_vector(row: &MonitorDatasetRow, direction_mode: DirectionMode) -> Vector3<f32> {
    match direction_mode {
        DirectionMode::North => Vector3::new(1.0, 0.0, 0.0),
        DirectionMode::East => Vector3::new(0.0, 1.0, 0.0),
        DirectionMode::South => Vector3::new(-1.0, 0.0, 0.0),
        DirectionMode::West => Vector3::new(0.0, -1.0, 0.0),
        DirectionMode::AlongTrack => horizontal_track(row, false),
        DirectionMode::CrossTrackRight => horizontal_track(row, true),
    }
}

fn horizontal_track(row: &MonitorDatasetRow, cross_track_right: bool) -> Vector3<f32> {
    let velocity = Vector3::new(row.state_vx_ned_mps, row.state_vy_ned_mps, 0.0);
    let norm = velocity.norm();
    let along_track = if norm > 1.0e-3 {
        velocity / norm
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };

    if cross_track_right {
        Vector3::new(along_track.y, -along_track.x, 0.0)
    } else {
        along_track
    }
}

fn inject_clock_bias(row: &mut MonitorDatasetRow, bias_delta_m: f32) {
    if bias_delta_m.abs() <= f32::EPSILON {
        return;
    }

    let reference_clock_bias_m = row.reference_clock_bias_m.unwrap_or(0.0);
    row.reference_clock_bias_m = Some(reference_clock_bias_m);
    row.observed_clock_bias_m = Some(reference_clock_bias_m + bias_delta_m);
    row.clock_bias_std_m = Some(row.clock_bias_std_m.unwrap_or(5.0).max(1.0e-3));
}

#[cfg(test)]
mod tests {
    use super::{
        DirectionMode, RealisticSpoofCase, apply_realistic_spoof_case,
        built_in_realistic_spoof_cases,
    };
    use crate::benchmark::MonitorDatasetRow;

    #[test]
    fn built_in_cases_include_real_attack_families() {
        let cases = built_in_realistic_spoof_cases();

        assert!(cases.iter().any(|case| case.family == "TEXBAT-like"));
        assert!(
            cases
                .iter()
                .any(|case| case.family == "UAV attack dataset-like")
        );
        assert!(
            cases
                .iter()
                .any(|case| case.label == "intermittent_pulsed_carryoff")
        );
    }

    #[test]
    fn time_push_profile_injects_clock_bias_without_requiring_existing_clock_fields() {
        let first = test_row(10.0, 0.0);
        let row = test_row(13.0, 10.0);
        let case = RealisticSpoofCase::ramp_hold(
            "clock",
            "test",
            "test",
            "clock ramp",
            DirectionMode::North,
            2.0,
            1.0,
            0.0,
            0.0,
            90.0,
        );

        let rows = apply_realistic_spoof_case(&[first, row], &case);

        assert!(!rows[0].label_spoofed);
        assert!(rows[1].label_spoofed);
        assert_eq!(rows[1].reference_clock_bias_m, Some(0.0));
        assert_eq!(rows[1].observed_clock_bias_m, Some(90.0));
        assert_eq!(rows[1].clock_bias_std_m, Some(5.0));
    }

    #[test]
    fn freeze_profile_holds_first_active_gps_position() {
        let rows = vec![
            test_row(10.0, 5.0),
            test_row(11.0, 10.0),
            test_row(12.0, 25.0),
            test_row(13.0, 40.0),
        ];
        let case = RealisticSpoofCase::freeze_position("freeze", "test", "test", "freeze", 2.0);

        let rows = apply_realistic_spoof_case(&rows, &case);

        assert!(!rows[0].label_spoofed);
        assert!(!rows[1].label_spoofed);
        assert!(rows[2].label_spoofed);
        assert!(rows[3].label_spoofed);
        assert_eq!(rows[2].gps_px_ned_m, 25.0);
        assert_eq!(rows[3].gps_px_ned_m, 25.0);
        assert_eq!(rows[2].gps_vx_ned_mps, 0.0);
        assert_eq!(rows[3].gps_vx_ned_mps, 0.0);
    }

    fn test_row(timestamp_s: f64, gps_px_ned_m: f32) -> MonitorDatasetRow {
        MonitorDatasetRow {
            timestamp_s,
            state_px_ned_m: gps_px_ned_m,
            state_py_ned_m: 0.0,
            state_pz_ned_m: 0.0,
            state_vx_ned_mps: 1.0,
            state_vy_ned_mps: 0.0,
            state_vz_ned_mps: 0.0,
            state_qw: 1.0,
            state_qx: 0.0,
            state_qy: 0.0,
            state_qz: 0.0,
            cov_pxx: 0.01,
            cov_pxy: 0.0,
            cov_pxz: 0.0,
            cov_pxvx: 0.0,
            cov_pxvy: 0.0,
            cov_pxvz: 0.0,
            cov_pyy: 0.01,
            cov_pyz: 0.0,
            cov_pyvx: 0.0,
            cov_pyvy: 0.0,
            cov_pyvz: 0.0,
            cov_pzz: 0.01,
            cov_pzvx: 0.0,
            cov_pzvy: 0.0,
            cov_pzvz: 0.0,
            cov_vxvx: 0.01,
            cov_vxvy: 0.0,
            cov_vxvz: 0.0,
            cov_vyvy: 0.01,
            cov_vyvz: 0.0,
            cov_vzvz: 0.01,
            cov_yaw_yaw: 0.01,
            gps_px_ned_m,
            gps_py_ned_m: 0.0,
            gps_pz_ned_m: 0.0,
            gps_vx_ned_mps: 1.0,
            gps_vy_ned_mps: 0.0,
            gps_vz_ned_mps: 0.0,
            gps_horizontal_position_std_m: 1.5,
            gps_vertical_position_std_m: 2.0,
            gps_horizontal_velocity_std_mps: 0.3,
            gps_vertical_velocity_std_mps: 0.5,
            barometer_altitude_ned_down_m: None,
            barometer_std_m: None,
            heading_rad: None,
            heading_std_rad: None,
            reference_clock_bias_m: None,
            observed_clock_bias_m: None,
            clock_bias_std_m: None,
            label_spoofed: false,
        }
    }
}
