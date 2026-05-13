use core::fmt;

use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::{
    ekf_core::state::EskfState,
    statistical_monitor::observation::{MonitorVerdict, TrustLevel},
};

pub const STATE_SNAPSHOT_LEN: usize = 16;
const MAX_EVIDENCE_PACKET_BYTES: usize = 256;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EvidencePacket {
    pub timestamp_ns: u64,
    pub telemetry_hash: [u8; 32],
    pub physics_verdict: bool,
    pub state_snapshot: [f32; STATE_SNAPSHOT_LEN],
}

impl EvidencePacket {
    pub fn from_state_snapshot(
        timestamp_ns: u64,
        raw_mavlink_message: &[u8],
        predicted_state: &EskfState,
        monitor_verdict: &MonitorVerdict,
    ) -> Self {
        Self {
            timestamp_ns,
            telemetry_hash: sha256_digest(raw_mavlink_message),
            physics_verdict: matches!(monitor_verdict.trust_level, TrustLevel::Trusted),
            state_snapshot: nominal_state_snapshot(predicted_state),
        }
    }

    #[cfg(feature = "telemetry")]
    pub fn from_synchronized_sample(
        timestamp_ns: u64,
        raw_mavlink_message: &[u8],
        synchronized_gps_sample: &crate::telemetry_adapter::mavlink_handler::SynchronizedGpsSample,
        monitor_verdict: &MonitorVerdict,
    ) -> Self {
        Self::from_state_snapshot(
            timestamp_ns,
            raw_mavlink_message,
            &synchronized_gps_sample.aligned_predicted_state,
            monitor_verdict,
        )
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignedEvidencePacket {
    pub evidence: EvidencePacket,
    #[serde(with = "signature_bytes_serde")]
    pub signature: [u8; 64],
    pub public_key: [u8; 32],
}

#[cfg(feature = "std")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FramedEvidenceSummary {
    pub total_packets: u64,
    pub trusted_verdicts: u64,
    pub flagged_or_rejected_verdicts: u64,
    pub first_timestamp_ns: Option<u64>,
    pub last_timestamp_ns: Option<u64>,
}

pub trait SecureElement {
    fn public_key_bytes(&self) -> [u8; 32];
    fn sign_bytes(&self, message: &[u8]) -> Result<[u8; 64], AttestationError>;
}

pub trait AttestationProvider {
    fn public_key_bytes(&self) -> [u8; 32];
    fn sign_message(&self, message: &[u8]) -> Result<[u8; 64], AttestationError>;

    fn sign_evidence(
        &self,
        evidence: &EvidencePacket,
    ) -> Result<SignedEvidencePacket, AttestationError> {
        let mut serialization_buffer = [0_u8; MAX_EVIDENCE_PACKET_BYTES];
        let serialized_packet = serialize_evidence(evidence, &mut serialization_buffer)?;
        let signature = self.sign_message(serialized_packet)?;

        Ok(SignedEvidencePacket {
            evidence: evidence.clone(),
            signature,
            public_key: self.public_key_bytes(),
        })
    }
}

#[derive(Clone)]
pub struct Ed25519AttestationProvider<S> {
    secure_element: S,
}

impl<S> Ed25519AttestationProvider<S> {
    pub const fn new(secure_element: S) -> Self {
        Self { secure_element }
    }
}

impl<S> AttestationProvider for Ed25519AttestationProvider<S>
where
    S: SecureElement,
{
    fn public_key_bytes(&self) -> [u8; 32] {
        self.secure_element.public_key_bytes()
    }

    fn sign_message(&self, message: &[u8]) -> Result<[u8; 64], AttestationError> {
        self.secure_element.sign_bytes(message)
    }
}

#[derive(Debug)]
pub enum AttestationError {
    Serialization(postcard::Error),
    InvalidVerifyingKey,
    SignatureVerificationFailed,
    #[cfg(feature = "std")]
    FramedEvidenceTruncated {
        offset: usize,
        expected_bytes: usize,
        available_bytes: usize,
    },
    SecureElementConfiguration {
        reason: &'static str,
    },
    #[cfg(feature = "std")]
    SecureElementIo(std::io::Error),
    InvalidSecretKeyLength {
        expected_bytes: usize,
        actual_bytes: usize,
    },
    InvalidHexEncoding,
}

impl fmt::Display for AttestationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serialization(error) => write!(f, "postcard serialization failed: {error:?}"),
            Self::InvalidVerifyingKey => write!(f, "public key bytes could not be parsed"),
            Self::SignatureVerificationFailed => {
                write!(f, "evidence signature verification failed")
            }
            #[cfg(feature = "std")]
            Self::FramedEvidenceTruncated {
                offset,
                expected_bytes,
                available_bytes,
            } => write!(
                f,
                "framed evidence stream truncated at byte {offset}: expected {expected_bytes} bytes, found {available_bytes}"
            ),
            Self::SecureElementConfiguration { reason } => {
                write!(f, "mock secure element configuration error: {reason}")
            }
            #[cfg(feature = "std")]
            Self::SecureElementIo(error) => write!(f, "mock secure element I/O error: {error}"),
            Self::InvalidSecretKeyLength {
                expected_bytes,
                actual_bytes,
            } => write!(
                f,
                "invalid secret key length: expected {expected_bytes} bytes, got {actual_bytes}"
            ),
            Self::InvalidHexEncoding => write!(f, "secret material was not valid hexadecimal"),
        }
    }
}

pub fn verify_evidence(signed_evidence: &SignedEvidencePacket) -> Result<(), AttestationError> {
    let mut serialization_buffer = [0_u8; MAX_EVIDENCE_PACKET_BYTES];
    let serialized_packet =
        serialize_evidence(&signed_evidence.evidence, &mut serialization_buffer)?;

    let verifying_key = VerifyingKey::from_bytes(&signed_evidence.public_key)
        .map_err(|_| AttestationError::InvalidVerifyingKey)?;
    let signature = Signature::from_bytes(&signed_evidence.signature);

    verifying_key
        .verify(serialized_packet, &signature)
        .map_err(|_| AttestationError::SignatureVerificationFailed)
}

pub fn serialize_evidence<'a>(
    evidence: &EvidencePacket,
    buffer: &'a mut [u8],
) -> Result<&'a [u8], AttestationError> {
    postcard::to_slice(evidence, buffer)
        .map(|bytes| &bytes[..])
        .map_err(AttestationError::Serialization)
}

pub fn deserialize_evidence(bytes: &[u8]) -> Result<SignedEvidencePacket, AttestationError> {
    postcard::from_bytes(bytes).map_err(AttestationError::Serialization)
}

#[cfg(feature = "std")]
pub fn decode_framed_evidence_bytes(
    bytes: &[u8],
) -> Result<Vec<SignedEvidencePacket>, AttestationError> {
    let mut offset = 0_usize;
    let mut packets = Vec::new();

    while offset < bytes.len() {
        let prefix_end = offset + 4;
        if prefix_end > bytes.len() {
            return Err(AttestationError::FramedEvidenceTruncated {
                offset,
                expected_bytes: 4,
                available_bytes: bytes.len().saturating_sub(offset),
            });
        }

        let packet_length = u32::from_le_bytes([
            bytes[offset],
            bytes[offset + 1],
            bytes[offset + 2],
            bytes[offset + 3],
        ]) as usize;
        offset = prefix_end;

        let packet_end = offset + packet_length;
        if packet_end > bytes.len() {
            return Err(AttestationError::FramedEvidenceTruncated {
                offset,
                expected_bytes: packet_length,
                available_bytes: bytes.len().saturating_sub(offset),
            });
        }

        packets.push(deserialize_evidence(&bytes[offset..packet_end])?);
        offset = packet_end;
    }

    Ok(packets)
}

#[cfg(feature = "std")]
pub fn verify_framed_evidence_bytes(
    bytes: &[u8],
) -> Result<FramedEvidenceSummary, AttestationError> {
    let packets = decode_framed_evidence_bytes(bytes)?;
    let mut summary = FramedEvidenceSummary::default();

    for packet in packets {
        verify_evidence(&packet)?;
        summary.total_packets += 1;
        if packet.evidence.physics_verdict {
            summary.trusted_verdicts += 1;
        } else {
            summary.flagged_or_rejected_verdicts += 1;
        }

        summary.first_timestamp_ns = Some(
            summary
                .first_timestamp_ns
                .map_or(packet.evidence.timestamp_ns, |current| {
                    current.min(packet.evidence.timestamp_ns)
                }),
        );
        summary.last_timestamp_ns = Some(
            summary
                .last_timestamp_ns
                .map_or(packet.evidence.timestamp_ns, |current| {
                    current.max(packet.evidence.timestamp_ns)
                }),
        );
    }

    Ok(summary)
}

mod signature_bytes_serde {
    use core::fmt;

    use serde::{
        Deserializer, Serialize, Serializer,
        de::{Error, SeqAccess, Visitor},
    };

    pub fn serialize<S>(signature: &[u8; 64], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        signature.as_slice().serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<[u8; 64], D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_bytes(SignatureVisitor)
    }

    struct SignatureVisitor;

    impl<'de> Visitor<'de> for SignatureVisitor {
        type Value = [u8; 64];

        fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
            formatter.write_str("a 64-byte Ed25519 signature")
        }

        fn visit_bytes<E>(self, value: &[u8]) -> Result<Self::Value, E>
        where
            E: Error,
        {
            if value.len() != 64 {
                return Err(E::invalid_length(value.len(), &self));
            }

            let mut signature = [0_u8; 64];
            signature.copy_from_slice(value);
            Ok(signature)
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
        where
            A: SeqAccess<'de>,
        {
            let mut signature = [0_u8; 64];
            for (index, slot) in signature.iter_mut().enumerate() {
                *slot = seq
                    .next_element()?
                    .ok_or_else(|| A::Error::invalid_length(index, &self))?;
            }
            Ok(signature)
        }
    }
}

fn nominal_state_snapshot(state: &EskfState) -> [f32; STATE_SNAPSHOT_LEN] {
    let quaternion = state.nominal.attitude_body_to_ned.quaternion();
    [
        state.nominal.position_ned_m.x,
        state.nominal.position_ned_m.y,
        state.nominal.position_ned_m.z,
        state.nominal.velocity_ned_mps.x,
        state.nominal.velocity_ned_mps.y,
        state.nominal.velocity_ned_mps.z,
        quaternion.w,
        quaternion.i,
        quaternion.j,
        quaternion.k,
        state.nominal.accel_bias_mps2.x,
        state.nominal.accel_bias_mps2.y,
        state.nominal.accel_bias_mps2.z,
        state.nominal.gyro_bias_rps.x,
        state.nominal.gyro_bias_rps.y,
        state.nominal.gyro_bias_rps.z,
    ]
}

fn sha256_digest(bytes: &[u8]) -> [u8; 32] {
    let digest = Sha256::digest(bytes);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&digest);
    output
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::{
        AttestationProvider, Ed25519AttestationProvider, EvidencePacket, SecureElement,
        decode_framed_evidence_bytes, verify_evidence, verify_framed_evidence_bytes,
    };
    use crate::{
        ekf_core::state::{EskfState, NominalState, StateCovariance},
        statistical_monitor::observation::{MonitorVerdict, ObservationVector, TrustLevel},
    };

    #[derive(Clone)]
    struct InMemorySecureElement {
        signing_key: ed25519_dalek::SigningKey,
    }

    impl InMemorySecureElement {
        fn new(secret_key: [u8; 32]) -> Self {
            Self {
                signing_key: ed25519_dalek::SigningKey::from_bytes(&secret_key),
            }
        }
    }

    impl SecureElement for InMemorySecureElement {
        fn public_key_bytes(&self) -> [u8; 32] {
            self.signing_key.verifying_key().to_bytes()
        }

        fn sign_bytes(&self, message: &[u8]) -> Result<[u8; 64], super::AttestationError> {
            use ed25519_dalek::Signer;

            Ok(self.signing_key.sign(message).to_bytes())
        }
    }

    #[test]
    fn evidence_round_trip_signs_and_verifies() {
        let secure_element = InMemorySecureElement::new([9_u8; 32]);
        let provider = Ed25519AttestationProvider::new(secure_element);

        let evidence = build_evidence_packet(TrustLevel::Trusted);
        let signed_evidence = provider.sign_evidence(&evidence).unwrap();

        verify_evidence(&signed_evidence).unwrap();
        assert!(signed_evidence.evidence.physics_verdict);
        assert_eq!(signed_evidence.evidence, evidence);
    }

    #[test]
    fn tampering_breaks_signature_validation() {
        let secure_element = InMemorySecureElement::new([5_u8; 32]);
        let provider = Ed25519AttestationProvider::new(secure_element);

        let evidence = build_evidence_packet(TrustLevel::Rejected);
        let mut signed_evidence = provider.sign_evidence(&evidence).unwrap();
        signed_evidence.evidence.physics_verdict = true;

        assert!(verify_evidence(&signed_evidence).is_err());
    }

    #[test]
    fn framed_evidence_stream_round_trips_and_verifies() {
        let secure_element = InMemorySecureElement::new([3_u8; 32]);
        let provider = Ed25519AttestationProvider::new(secure_element);
        let signed_evidence_a = provider
            .sign_evidence(&build_evidence_packet(TrustLevel::Trusted))
            .unwrap();
        let signed_evidence_b = provider
            .sign_evidence(&build_evidence_packet(TrustLevel::Rejected))
            .unwrap();

        let bytes = framed_bytes(&[signed_evidence_a.clone(), signed_evidence_b.clone()]);
        let restored = decode_framed_evidence_bytes(&bytes).unwrap();
        let summary = verify_framed_evidence_bytes(&bytes).unwrap();

        assert_eq!(restored, vec![signed_evidence_a, signed_evidence_b]);
        assert_eq!(summary.total_packets, 2);
        assert_eq!(summary.trusted_verdicts, 1);
        assert_eq!(summary.flagged_or_rejected_verdicts, 1);
    }

    #[test]
    fn framed_evidence_stream_rejects_tampering() {
        let secure_element = InMemorySecureElement::new([4_u8; 32]);
        let provider = Ed25519AttestationProvider::new(secure_element);
        let signed_evidence = provider
            .sign_evidence(&build_evidence_packet(TrustLevel::Trusted))
            .unwrap();
        let mut bytes = framed_bytes(&[signed_evidence]);
        let final_payload_index = bytes.len() - 1;
        bytes[final_payload_index] ^= 0x01;

        assert!(verify_framed_evidence_bytes(&bytes).is_err());
    }

    fn build_evidence_packet(trust_level: TrustLevel) -> EvidencePacket {
        let predicted_state = EskfState::new(
            NominalState {
                timestamp_s: 12.0,
                position_ned_m: Vector3::new(99.5, -24.5, 4.2),
                velocity_ned_mps: Vector3::new(2.1, 0.4, -0.2),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::new(0.01, -0.02, 0.005),
                gyro_bias_rps: Vector3::new(0.001, 0.002, -0.001),
                geodetic_reference: None,
            },
            StateCovariance::identity(),
        );
        let monitor_verdict = MonitorVerdict {
            squared_mahalanobis_distance: if matches!(trust_level, TrustLevel::Trusted) {
                2.0
            } else {
                50.0
            },
            gps_squared_mahalanobis_distance: if matches!(trust_level, TrustLevel::Trusted) {
                2.0
            } else {
                50.0
            },
            barometer_squared_mahalanobis_distance: None,
            heading_squared_mahalanobis_distance: None,
            clock_bias_squared_mahalanobis_distance: None,
            clock_bias_persistent_score: None,
            horizontal_residual_persistent_score: None,
            accumulated_risk: if matches!(trust_level, TrustLevel::Trusted) {
                1.5
            } else {
                30.0
            },
            innovation: ObservationVector::zeros(),
            barometer_residual_m: None,
            heading_residual_rad: None,
            clock_bias_residual_m: None,
            trust_level,
        };

        EvidencePacket::from_state_snapshot(
            1_725_897_123_456_789_000,
            &[0xFE, 0x09, 0x01, 0x21, 0x33, 0x44],
            &predicted_state,
            &monitor_verdict,
        )
    }

    fn framed_bytes(packets: &[super::SignedEvidencePacket]) -> Vec<u8> {
        let mut output = Vec::new();
        for packet in packets {
            let mut buffer = [0_u8; 384];
            let encoded = postcard::to_slice(packet, &mut buffer).unwrap();
            output.extend_from_slice(&(encoded.len() as u32).to_le_bytes());
            output.extend_from_slice(&encoded);
        }
        output
    }
}
