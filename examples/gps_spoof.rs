use rtvlas::{
    BarometerSample, Ed25519Signer, GpsFix, ImuSample, MagnetometerSample, MonitorConfig,
    TelemetryFrame, TrustMonitor, Vec3,
};

fn main() {
    let signer = Ed25519Signer::from_secret_bytes([42_u8; 32]);
    let mut monitor = TrustMonitor::new(MonitorConfig::default(), signer);

    for step in 0..20 {
        let time_s = step as f64 * 0.1;
        let spoofed = step >= 12;
        let gps_position = if spoofed {
            Vec3::new(80.0, 0.0, 0.0)
        } else {
            Vec3::ZERO
        };

        let output = monitor.process_frame(&TelemetryFrame {
            imu: ImuSample {
                timestamp_s: time_s,
                specific_force_body_mps2: Vec3::new(0.0, 0.0, -9.81),
                gyro_body_rps: Vec3::ZERO,
            },
            gps: Some(GpsFix {
                timestamp_s: time_s,
                position_ned_m: gps_position,
                velocity_ned_mps: Vec3::ZERO,
                horizontal_accuracy_m: 1.5,
                vertical_accuracy_m: 2.0,
                speed_accuracy_mps: 0.2,
            }),
            barometer: Some(BarometerSample {
                timestamp_s: time_s,
                relative_altitude_m: 0.0,
                accuracy_m: 0.8,
            }),
            magnetometer: Some(MagnetometerSample {
                timestamp_s: time_s,
                heading_rad: 0.0,
                accuracy_rad: 0.05,
            }),
        });

        println!(
            "t={time_s:>4.1}s verdict={:?} risk={:.2} pos_mahal={:.2} reasons={:?}",
            output.assessment.verdict,
            output.assessment.risk_score,
            output.assessment.position_mahalanobis,
            output.assessment.reasons
        );
        println!(
            "{}",
            serde_json::to_string_pretty(&output.evidence_event)
                .expect("example evidence event should serialize")
        );
    }
}
