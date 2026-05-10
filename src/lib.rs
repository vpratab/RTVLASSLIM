#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

pub mod ekf_core;
pub mod statistical_monitor;
pub mod telemetry_adapter;

#[cfg(feature = "attestation")]
pub mod attestation;

#[cfg(all(feature = "telemetry", feature = "attestation", feature = "std"))]
pub mod orchestrator;

#[cfg(all(feature = "validation", feature = "std"))]
pub mod validation;

#[cfg(all(feature = "validation", feature = "std"))]
pub mod benchmark;
