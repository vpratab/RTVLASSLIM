pub mod conversions;

#[cfg(feature = "telemetry")]
pub mod mavlink_handler;

pub use conversions::{
    ConversionError, GeodeticPosition, HomePosition, centimetres_per_second_to_metres_per_second,
    geodetic_to_ned, microseconds_to_seconds, millimetres_to_metres, milliseconds_to_seconds,
    offset_geodetic_position_ned, scaled_degrees_e7_to_radians,
};

#[cfg(feature = "telemetry")]
pub use mavlink_handler::{
    AuxiliaryObservationConfig, DEFAULT_MAVLINK_UDP_ADDRESS, GpsNoiseModel,
    MAX_MAVLINK_FRAME_BYTES, MavlinkFrameBuffer, MavlinkSubscriber, MavlinkSubscriberConfig,
    PendingGpsSample, SynchronizedGpsSample, TelemetryError, TelemetryUpdate, purge_frame_buffer,
};
