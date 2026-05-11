pub mod evidence;

pub use evidence::{
    AttestationError, AttestationProvider, Ed25519AttestationProvider, EvidencePacket,
    STATE_SNAPSHOT_LEN, SecureElement, SignedEvidencePacket, deserialize_evidence,
    serialize_evidence, verify_evidence,
};

#[cfg(feature = "std")]
pub use secure_element::MockSecureElement;

#[cfg(feature = "std")]
pub mod secure_element {
    use std::{env, fs, path::Path};

    use ed25519_dalek::{Signer, SigningKey};

    use super::evidence::{AttestationError, SecureElement};

    #[derive(Clone)]
    pub struct MockSecureElement {
        signing_key: SigningKey,
    }

    impl MockSecureElement {
        pub fn from_env(variable_name: &str) -> Result<Self, AttestationError> {
            let raw_value = env::var(variable_name).map_err(|_| {
                AttestationError::SecureElementConfiguration {
                    reason: "requested environment variable was not present",
                }
            })?;
            let secret_key_bytes = decode_secret_material(raw_value.as_bytes())?;
            Ok(Self::from_secret_key_bytes(secret_key_bytes))
        }

        pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, AttestationError> {
            let raw_value = fs::read(path).map_err(AttestationError::SecureElementIo)?;
            let secret_key_bytes = decode_secret_material(&raw_value)?;
            Ok(Self::from_secret_key_bytes(secret_key_bytes))
        }

        pub fn from_secret_key_bytes(secret_key_bytes: [u8; 32]) -> Self {
            Self {
                signing_key: SigningKey::from_bytes(&secret_key_bytes),
            }
        }
    }

    impl SecureElement for MockSecureElement {
        fn public_key_bytes(&self) -> [u8; 32] {
            self.signing_key.verifying_key().to_bytes()
        }

        fn sign_bytes(&self, message: &[u8]) -> Result<[u8; 64], AttestationError> {
            Ok(self.signing_key.sign(message).to_bytes())
        }
    }

    fn decode_secret_material(raw_value: &[u8]) -> Result<[u8; 32], AttestationError> {
        let trimmed = trim_ascii_whitespace(raw_value);

        if trimmed.len() == 32 {
            let mut secret_key_bytes = [0_u8; 32];
            secret_key_bytes.copy_from_slice(trimmed);
            return Ok(secret_key_bytes);
        }

        decode_hex_secret_key(trimmed)
    }

    fn trim_ascii_whitespace(raw_value: &[u8]) -> &[u8] {
        let start = raw_value
            .iter()
            .position(|byte| !byte.is_ascii_whitespace())
            .unwrap_or(raw_value.len());
        let end = raw_value
            .iter()
            .rposition(|byte| !byte.is_ascii_whitespace())
            .map(|index| index + 1)
            .unwrap_or(start);
        &raw_value[start..end]
    }

    fn decode_hex_secret_key(hex_bytes: &[u8]) -> Result<[u8; 32], AttestationError> {
        if hex_bytes.len() != 64 {
            return Err(AttestationError::InvalidSecretKeyLength {
                expected_bytes: 32,
                actual_bytes: hex_bytes.len(),
            });
        }

        let mut secret_key_bytes = [0_u8; 32];
        for (index, chunk) in hex_bytes.chunks_exact(2).enumerate() {
            let high = decode_hex_nibble(chunk[0])?;
            let low = decode_hex_nibble(chunk[1])?;
            secret_key_bytes[index] = (high << 4) | low;
        }

        Ok(secret_key_bytes)
    }

    fn decode_hex_nibble(value: u8) -> Result<u8, AttestationError> {
        match value {
            b'0'..=b'9' => Ok(value - b'0'),
            b'a'..=b'f' => Ok(value - b'a' + 10),
            b'A'..=b'F' => Ok(value - b'A' + 10),
            _ => Err(AttestationError::InvalidHexEncoding),
        }
    }
}
