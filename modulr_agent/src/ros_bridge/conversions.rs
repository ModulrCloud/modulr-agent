/// Convert euler angles (yaw, pitch, roll) to a quaternion (x, y, z, w).
///
/// Uses the ZYX (yaw-pitch-roll) convention standard in ROS.
pub fn euler_to_quaternion(yaw: f64, pitch: f64, roll: f64) -> (f64, f64, f64, f64) {
    let (sy, cy) = (yaw / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sr, cr) = (roll / 2.0).sin_cos();

    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;
    let w = cr * cp * cy + sr * sp * sy;

    (x, y, z, w)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_returns_unit_quaternion() {
        let (x, y, z, w) = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!((x).abs() < 1e-10);
        assert!((y).abs() < 1e-10);
        assert!((z).abs() < 1e-10);
        assert!((w - 1.0).abs() < 1e-10);
    }

    #[test]
    fn yaw_90_degrees() {
        let (x, y, z, w) = euler_to_quaternion(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!((x).abs() < 1e-10);
        assert!((y).abs() < 1e-10);
        assert!((z - expected).abs() < 1e-10);
        assert!((w - expected).abs() < 1e-10);
    }
}
