# Forensics And Evidence

RTVLAS-Slim emits signed evidence packets so that verdicts can be checked after a run without rerunning the detector. This document describes what is implemented today, what is verified by the current binary, and what is not implemented.

## Current Evidence Chain

The current source implements:

- A compact `EvidencePacket`.
- A SHA-256 hash of the raw telemetry bytes associated with the verdict.
- Ed25519 signing over the serialized evidence packet.
- A length-framed evidence stream written by the file sink.
- A separate verifier binary that parses the framed stream and verifies every packet signature.

The current source does not implement a Merkle tree, Merkle root, or append-only Merkle accumulator. If that capability is added later, it should be documented as a new measured feature, not inferred from the current evidence stream.

## Evidence Packet Fields

Each packet contains:

| Field | Meaning |
| --- | --- |
| `timestamp_ns` | verdict timestamp in nanoseconds |
| `telemetry_hash` | SHA-256 digest of the raw telemetry bytes used for the verdict |
| `physics_verdict` | boolean summary, true for `Trusted`, false for `Flagged` or `Rejected` |
| `state_snapshot` | 16-element nominal-state snapshot: position, velocity, quaternion, accelerometer bias, gyro bias |

The signed wrapper adds:

| Field | Meaning |
| --- | --- |
| `signature` | 64-byte Ed25519 signature |
| `public_key` | 32-byte Ed25519 verifying key |

The detector also computes Mahalanobis distance, CUSUM scores, and innovation vectors internally. The current `EvidencePacket` stores the boolean verdict and state snapshot, not the full monitor diagnostic vector. If full monitor diagnostics are needed in evidence, that is future schema work.

## Telemetry Hash

The telemetry hash is:

```text
telemetry_hash = SHA256(raw_mavlink_message_bytes)
```

This binds the evidence packet to the raw bytes presented to the monitor at that verdict. If the telemetry bytes are altered later, the stored hash no longer matches the altered bytes.

The current verifier checks packet signatures. It does not independently re-read a separate raw MAVLink capture and compare it to `telemetry_hash`; that would require archiving the raw telemetry stream as an additional artifact.

## Ed25519 Signing

The signing flow is:

```text
serialized_packet = postcard(EvidencePacket)
signature = Ed25519_sign(serialized_packet)
SignedEvidencePacket = EvidencePacket + signature + public_key
```

Verification is:

```text
serialized_packet = postcard(EvidencePacket)
Ed25519_verify(public_key, serialized_packet, signature)
```

If any signed field is changed, verification fails. The library tests include a tampering check that flips packet contents and expects signature verification failure.

## Framed Evidence Stream

The file sink writes a sequence of signed packets. Each packet is prefixed by a little-endian 32-bit length, followed by the postcard-serialized `SignedEvidencePacket`.

```text
[len_0][signed_packet_0][len_1][signed_packet_1]...
```

This framing lets the verifier detect truncated packet boundaries and verify every signature in sequence. It is an append sequence, but it is not a cryptographic append-only log because there is no hash chain or Merkle root in the current implementation.

## Verification Command

Command:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Measured output from the current live-spoof evidence artifact:

```text
Evidence file: artifacts/wsl_px4_live_spoof_evidence.bin
  packets verified: 30
  trusted verdicts: 13
  flagged/rejected verdicts: 17
  first timestamp (ns): 3796000000
  last timestamp (ns): 6700000000
```

This verifies the evidence signatures and frame integrity without running the detector or replaying PX4.

## What This Proves

The current evidence chain proves:

- The evidence file can be parsed as a complete framed packet stream.
- Every packet in the measured file has a valid Ed25519 signature over its serialized packet contents.
- Tampering with signed packet bytes is detected by signature verification.
- The measured live-spoof file contains 30 packets, 13 trusted verdicts, and 17 flagged or rejected verdicts.

## What This Does Not Prove

The current evidence chain does not prove:

- Raw telemetry was preserved elsewhere.
- The `telemetry_hash` can be checked against an independent raw MAVLink archive.
- Packets cannot be deleted from the middle of a file by someone who controls file storage.
- A Merkle root was anchored externally.
- A hardware secure element protected the signing key.

Those are useful next steps, but they are not current measured capabilities.

## Operational Use Cases

The current evidence stream is useful for:

- Post-mission engineering review of when verdicts changed.
- Rechecking that evidence files were not modified after signing.
- Demonstrating an auditable signing path for an SBIR Phase I prototype.

Potential future use cases after raw telemetry archival, hardware key storage, and Merkle anchoring:

- FAA incident review.
- Insurance claim support.
- DoD post-mission integrity review.
- Integration with a secure enclave or hardware-backed verifier.
