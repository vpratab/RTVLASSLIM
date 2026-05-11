use core::f64::consts::PI;

use libm::{cos, sin, sqrt};
use nalgebra::{Matrix3, Vector3};

const DEGREES_E7_TO_RADIANS: f64 = PI / (180.0 * 10_000_000.0);
const MILLIMETRES_TO_METRES: f32 = 1.0e-3;
const CENTIMETRES_PER_SECOND_TO_METRES_PER_SECOND: f32 = 1.0e-2;
const MICROSECONDS_TO_SECONDS: f64 = 1.0e-6;
const MILLISECONDS_TO_SECONDS: f64 = 1.0e-3;

const WGS84_SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
const WGS84_FLATTENING: f64 = 1.0 / 298.257_223_563;
const WGS84_ECCENTRICITY_SQUARED: f64 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConversionError {
    LatitudeOutOfRange { latitude_rad: f64 },
    LongitudeOutOfRange { longitude_rad: f64 },
    FloatRangeExceeded,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPosition {
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub altitude_m: f64,
}

impl GeodeticPosition {
    pub fn new(
        latitude_rad: f64,
        longitude_rad: f64,
        altitude_m: f64,
    ) -> Result<Self, ConversionError> {
        if !(-PI * 0.5..=PI * 0.5).contains(&latitude_rad) {
            return Err(ConversionError::LatitudeOutOfRange { latitude_rad });
        }
        if !(-PI..=PI).contains(&longitude_rad) {
            return Err(ConversionError::LongitudeOutOfRange { longitude_rad });
        }

        Ok(Self {
            latitude_rad,
            longitude_rad,
            altitude_m,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HomePosition {
    pub geodetic: GeodeticPosition,
    ecef_origin_m: Vector3<f64>,
    ecef_to_ned_rotation: Matrix3<f64>,
}

impl HomePosition {
    pub fn new(geodetic: GeodeticPosition) -> Self {
        let ecef_origin_m = geodetic_to_ecef(geodetic);
        let ecef_to_ned_rotation =
            ecef_to_ned_rotation(geodetic.latitude_rad, geodetic.longitude_rad);

        Self {
            geodetic,
            ecef_origin_m,
            ecef_to_ned_rotation,
        }
    }
}

pub fn scaled_degrees_e7_to_radians(value: i32) -> Result<f64, ConversionError> {
    let radians = f64::from(value) * DEGREES_E7_TO_RADIANS;
    if !radians.is_finite() {
        return Err(ConversionError::FloatRangeExceeded);
    }
    Ok(radians)
}

pub fn millimetres_to_metres(value: i32) -> Result<f32, ConversionError> {
    let metres = (value as f64) * f64::from(MILLIMETRES_TO_METRES);
    if !metres.is_finite() || metres.abs() > f64::from(f32::MAX) {
        return Err(ConversionError::FloatRangeExceeded);
    }
    Ok(metres as f32)
}

pub fn centimetres_per_second_to_metres_per_second(value: i16) -> f32 {
    f32::from(value) * CENTIMETRES_PER_SECOND_TO_METRES_PER_SECOND
}

pub fn microseconds_to_seconds(value: u64) -> f64 {
    (value as f64) * MICROSECONDS_TO_SECONDS
}

pub fn milliseconds_to_seconds(value: u32) -> f64 {
    f64::from(value) * MILLISECONDS_TO_SECONDS
}

pub fn geodetic_to_ned(
    home_position: HomePosition,
    geodetic_position: GeodeticPosition,
) -> Result<Vector3<f32>, ConversionError> {
    let target_ecef_m = geodetic_to_ecef(geodetic_position);
    let delta_ecef_m = target_ecef_m - home_position.ecef_origin_m;
    let ned_m = home_position.ecef_to_ned_rotation * delta_ecef_m;

    if ned_m.x.abs() > f64::from(f32::MAX)
        || ned_m.y.abs() > f64::from(f32::MAX)
        || ned_m.z.abs() > f64::from(f32::MAX)
    {
        return Err(ConversionError::FloatRangeExceeded);
    }

    Ok(Vector3::new(ned_m.x as f32, ned_m.y as f32, ned_m.z as f32))
}

pub fn offset_geodetic_position_ned(
    geodetic_position: GeodeticPosition,
    ned_offset_m: Vector3<f64>,
) -> Result<GeodeticPosition, ConversionError> {
    let sin_lat = sin(geodetic_position.latitude_rad);
    let cos_lat = cos(geodetic_position.latitude_rad);
    let eccentricity_term = 1.0 - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat;
    let prime_vertical_radius_m = WGS84_SEMI_MAJOR_AXIS_M / sqrt(eccentricity_term);
    let meridian_radius_m = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_ECCENTRICITY_SQUARED)
        / (eccentricity_term * sqrt(eccentricity_term));

    let latitude_rad = geodetic_position.latitude_rad
        + ned_offset_m.x / (meridian_radius_m + geodetic_position.altitude_m);
    let longitude_denominator_m = (prime_vertical_radius_m + geodetic_position.altitude_m)
        * cos_lat.abs().max(1.0e-9);
    let longitude_delta_rad = if cos_lat >= 0.0 {
        ned_offset_m.y / longitude_denominator_m
    } else {
        -ned_offset_m.y / longitude_denominator_m
    };
    let longitude_rad = normalise_longitude_rad(geodetic_position.longitude_rad + longitude_delta_rad);
    let altitude_m = geodetic_position.altitude_m - ned_offset_m.z;

    GeodeticPosition::new(latitude_rad, longitude_rad, altitude_m)
}

fn geodetic_to_ecef(geodetic_position: GeodeticPosition) -> Vector3<f64> {
    let sin_lat = sin(geodetic_position.latitude_rad);
    let cos_lat = cos(geodetic_position.latitude_rad);
    let sin_lon = sin(geodetic_position.longitude_rad);
    let cos_lon = cos(geodetic_position.longitude_rad);

    let radius_of_curvature =
        WGS84_SEMI_MAJOR_AXIS_M / sqrt(1.0 - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
    let altitude = geodetic_position.altitude_m;

    let x = (radius_of_curvature + altitude) * cos_lat * cos_lon;
    let y = (radius_of_curvature + altitude) * cos_lat * sin_lon;
    let z = (radius_of_curvature * (1.0 - WGS84_ECCENTRICITY_SQUARED) + altitude) * sin_lat;

    Vector3::new(x, y, z)
}

fn ecef_to_ned_rotation(latitude_rad: f64, longitude_rad: f64) -> Matrix3<f64> {
    let sin_lat = sin(latitude_rad);
    let cos_lat = cos(latitude_rad);
    let sin_lon = sin(longitude_rad);
    let cos_lon = cos(longitude_rad);

    Matrix3::new(
        -sin_lat * cos_lon,
        -sin_lat * sin_lon,
        cos_lat,
        -sin_lon,
        cos_lon,
        0.0,
        -cos_lat * cos_lon,
        -cos_lat * sin_lon,
        -sin_lat,
    )
}

fn normalise_longitude_rad(longitude_rad: f64) -> f64 {
    let wrapped = (longitude_rad + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped == -PI {
        PI
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{GeodeticPosition, HomePosition, geodetic_to_ned, offset_geodetic_position_ned};

    #[test]
    fn projecting_home_position_returns_zero() {
        let home_geodetic = GeodeticPosition::new(0.652_534, -2.136_622, 132.0).unwrap();
        let home_position = HomePosition::new(home_geodetic);
        let ned = geodetic_to_ned(home_position, home_geodetic).unwrap();

        assert!(ned.norm() < 1.0e-3);
    }

    #[test]
    fn ned_offset_round_trips_through_geodetic_projection() {
        let home_geodetic = GeodeticPosition::new(0.652_534, -2.136_622, 132.0).unwrap();
        let home_position = HomePosition::new(home_geodetic);
        let injected_offset_m = Vector3::new(87.5_f64, -43.0_f64, 12.0_f64);
        let offset_geodetic =
            offset_geodetic_position_ned(home_geodetic, injected_offset_m).unwrap();

        let projected_offset_m = geodetic_to_ned(home_position, offset_geodetic).unwrap();
        let projected_offset_m = projected_offset_m.cast::<f64>();

        assert!((projected_offset_m - injected_offset_m).norm() < 0.25);
    }
}
