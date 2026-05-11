# Contributing

This repository is still an early-stage prototype, so the contribution bar is mostly about keeping claims honest and keeping the verified paths reproducible.

## Ground Rules

- Do not add benchmark or detection claims that have not been rerun locally.
- Keep the core crate `no_std` compatible where it is already intended to be `no_std`.
- Keep simulator-only, processed-data, and real-world validation claims clearly separated.
- Prefer small, reviewable changes over broad rewrites.

## Useful Commands

```powershell
cargo fmt --all
cargo check --no-default-features --lib
cargo check --all-targets
cargo test --lib
cargo check --examples
```

## PX4 / WSL Paths

The verified PX4 flows currently depend on a local ignored PX4 checkout under `external/PX4-Autopilot`.

Useful entry points:

- `scripts/wsl_inline_live.sh`
- `scripts/wsl_px4_benchmark.sh`
- `scripts/wsl_px4_live_spoof.sh`

## Data Hygiene

- `artifacts/` is intentionally ignored and should remain local output only.
- `external/` is intentionally ignored and should not be committed as part of this repository.
