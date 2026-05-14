# Forensics And Evidence

RTVLAS-Slim emits signed evidence packets so that verdicts can be checked after a run without rerunning the detector. This document describes what is implemented today, what is verified by the current binary, and what is not implemented.

## Current Evidence Chain

The current source implements:

- A compact `EvidencePacket`.
- A SHA-256 hash of the raw telemetry bytes associated with the verdict.
- Ed25519 signing over the serialized evidence packet.
- A length-framed evidence stream written by the file sink.
- A separate verifier binary that parses the framed stream and verifies every packet signature.
- A deterministic SHA-256 chain root computed by the verifier over the signed packet sequence.

The current source does not implement a Merkle tree, Merkle root, external timestamp anchor, or hardware-backed signing key. The implemented chain root is a lightweight audit root over the packet sequence, not a full transparency log.

## Evidence Packet Fields

Each packet contains:

| Field | Meaning |
| --- | --- |
| `timestamp_ns` | verdict timestamp in nanoseconds |
| `telemetry_hash` | SHA-256 digest of the raw telemetry bytes used for the verdict |
| `physics_verdict` | boolean summary, true for `Trusted`, false for `Flagged` or `Rejected` |
| `state_snapshot` | 16-element nominal-state snapshot: position, velocity, quaternion, accelerometer bias, gyro bias |
| `diagnostics` | optional monitor diagnostic snapshot for newly generated evidence |

The signed wrapper adds:

| Field | Meaning |
| --- | --- |
| `signature` | 64-byte Ed25519 signature |
| `public_key` | 32-byte Ed25519 verifying key |

The diagnostic snapshot contains the trust-level code, total and GPS Mahalanobis scores, accumulated risk, six-element GPS innovation vector, optional barometer/heading/clock residuals, and clock/horizontal/velocity/stale persistence scores. Older evidence files in this repository were generated before this field existed; the verifier supports both the legacy compact packet shape and the newer diagnostic packet shape.

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

This framing lets the verifier detect truncated packet boundaries and verify every signature in sequence. The verifier also computes a chain root:

```text
root_0 = 32 zero bytes
root_{k+1} = SHA256(root_k || little_endian_len(packet_k) || packet_k)
```

This root changes if a packet is modified, reordered, or deleted. It can be copied into a lab notebook, proposal appendix, external timestamp service, or later transparency log. It is still not a Merkle tree and it is not externally anchored by the current repository.

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
  diagnostic packets: 0
  first timestamp (ns): 3796000000
  last timestamp (ns): 6700000000
  evidence chain root: aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36
```

This verifies the evidence signatures and frame integrity without running the detector or replaying PX4. `diagnostic packets: 0` means this specific artifact is a legacy evidence stream; newly generated evidence packets include signed diagnostics.

## What This Proves

The current evidence chain proves:

- The evidence file can be parsed as a complete framed packet stream.
- Every packet in the measured file has a valid Ed25519 signature over its serialized packet contents.
- Tampering with signed packet bytes is detected by signature verification.
- Packet reordering or deletion changes the verifier-computed chain root.
- The measured live-spoof file contains 30 packets, 13 trusted verdicts, and 17 flagged or rejected verdicts.
- The current measured chain root for that file is `aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36`.

## What This Does Not Prove

The current evidence chain does not prove:

- Raw telemetry was preserved elsewhere.
- The `telemetry_hash` can be checked against an independent raw MAVLink archive.
- Packets cannot be deleted from the middle of a file by someone who controls file storage unless the expected chain root was recorded independently.
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
