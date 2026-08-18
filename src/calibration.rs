use crate::{FlowMotionQuat as Quat, FlowMotionVec3 as Vec3, normalized, quat_mul, rotate};
use std::collections::HashMap;
use std::env;
use std::f32::consts::FRAC_1_SQRT_2;
use std::fs;
use std::path::PathBuf;

const DEFAULT_SECONDS: f32 = 1.0;
const DEFAULT_COUNTDOWN_SECONDS: f32 = 3.0;
const DEFAULT_CONTROLLER_CENTER_DOWN_METRES: f32 = 0.03;
const PALM_DOWN_ROLL: Quat = Quat {
    x: 0.0,
    y: 0.0,
    z: 1.0,
    w: 0.0,
};
// A strict OpenXR grip frame needs 90 degrees, but Index-style VRChat hands
// visually over-rotate; hardware testing selected a 12.5-degree model bias.
const GRIP_POSE_ALIGNMENT_DEGREES: f32 = 65.0;

pub(crate) fn hand_pose_roll(is_left: bool) -> Quat {
    Quat {
        x: 0.0,
        y: 0.0,
        z: if is_left {
            FRAC_1_SQRT_2
        } else {
            -FRAC_1_SQRT_2
        },
        w: FRAC_1_SQRT_2,
    }
}

pub(crate) fn grip_pose_alignment() -> Quat {
    let half_angle = GRIP_POSE_ALIGNMENT_DEGREES.to_radians() / 2.0;
    Quat {
        x: half_angle.sin(),
        y: 0.0,
        z: 0.0,
        w: half_angle.cos(),
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Pose {
    pub(crate) orientation: Quat,
    pub(crate) position: Vec3,
}

#[derive(Clone, Debug)]
struct StoredCalibration {
    serial: String,
    pose: Pose,
}

pub(crate) struct Calibration {
    requested_hand: Option<bool>,
    path: Option<PathBuf>,
    status_path: Option<PathBuf>,
    offsets: [Pose; 2],
    stored: [Option<StoredCalibration>; 2],
    controller_center_down: f32,
}

fn hand_index(is_left: bool) -> usize {
    usize::from(!is_left)
}

fn hand_name(is_left: bool) -> &'static str {
    if is_left { "LEFT" } else { "RIGHT" }
}

fn env_float(name: &str, default: f32) -> f32 {
    env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .filter(|value: &f32| value.is_finite())
        .unwrap_or(default)
}

fn default_pose() -> Pose {
    Pose {
        orientation: normalized(Quat {
            x: env_float("FMC_OFFSET_QX", 0.0),
            y: env_float("FMC_OFFSET_QY", 0.0),
            z: env_float("FMC_OFFSET_QZ", 0.0),
            w: env_float("FMC_OFFSET_QW", 1.0),
        }),
        position: Vec3 {
            x: env_float("FMC_OFFSET_X", 0.0),
            y: env_float("FMC_OFFSET_Y", 0.0),
            z: env_float("FMC_OFFSET_Z", 0.0),
        },
    }
}

fn calibration_path() -> Option<PathBuf> {
    if let Some(path) = env::var_os("FMC_CALIBRATION_FILE") {
        return Some(path.into());
    }
    if let Some(directory) = env::var_os("XDG_CONFIG_HOME") {
        return Some(PathBuf::from(directory).join("flow-motion-controller/calibration.conf"));
    }
    env::var_os("HOME")
        .map(|home| PathBuf::from(home).join(".config/flow-motion-controller/calibration.conf"))
}

fn parse_file(path: Option<&PathBuf>) -> HashMap<String, String> {
    let Some(path) = path else {
        return HashMap::new();
    };
    let contents = match fs::read_to_string(path) {
        Ok(contents) => contents,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return HashMap::new(),
        Err(error) => {
            eprintln!(
                "Ignoring unreadable calibration file {}: {error}",
                path.display()
            );
            return HashMap::new();
        }
    };
    let mut values = HashMap::new();
    for (line_number, line) in contents.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            eprintln!(
                "Ignoring malformed calibration line {} in {}",
                line_number + 1,
                path.display()
            );
            continue;
        };
        values.insert(key.trim().to_owned(), value.trim().to_owned());
    }
    values
}

fn stored_hand(is_left: bool, values: &HashMap<String, String>) -> Option<StoredCalibration> {
    let hand = hand_name(is_left);
    let prefix = format!("{hand}_");
    if !values.keys().any(|key| key.starts_with(&prefix)) {
        return None;
    }
    let required = |suffix: &str| -> Result<&str, String> {
        values
            .get(&format!("{hand}_{suffix}"))
            .map(String::as_str)
            .filter(|value| !value.is_empty())
            .ok_or_else(|| format!("missing {hand}_{suffix}"))
    };
    let number = |suffix: &str| -> Result<f32, String> {
        let value = required(suffix)?;
        value
            .parse::<f32>()
            .ok()
            .filter(|value| value.is_finite())
            .ok_or_else(|| format!("invalid {hand}_{suffix} value '{value}'"))
    };
    let parsed = (|| {
        let serial = required("SERIAL")?.to_owned();
        let orientation = Quat {
            x: number("OFFSET_QX")?,
            y: number("OFFSET_QY")?,
            z: number("OFFSET_QZ")?,
            w: number("OFFSET_QW")?,
        };
        let length_squared = orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w;
        if length_squared < f32::EPSILON {
            return Err(format!("{hand} orientation quaternion is zero"));
        }
        Ok(StoredCalibration {
            serial,
            pose: Pose {
                orientation: normalized(orientation),
                position: Vec3 {
                    x: number("OFFSET_X")?,
                    y: number("OFFSET_Y")?,
                    z: number("OFFSET_Z")?,
                },
            },
        })
    })();
    match parsed {
        Ok(stored) => Some(stored),
        Err(error) => {
            eprintln!(
                "Ignoring invalid {} hand calibration: {error}",
                hand.to_ascii_lowercase()
            );
            None
        }
    }
}

fn effective_pose(is_left: bool, stored: Option<&StoredCalibration>, fallback: Pose) -> Pose {
    let hand = hand_name(is_left);
    let configured_serial = env::var(format!("FMC_{hand}_TRACKER_SERIAL")).unwrap_or_default();
    let stored_pose = stored.and_then(|stored| {
        if !configured_serial.is_empty() && stored.serial == configured_serial {
            Some(stored.pose)
        } else {
            eprintln!(
                "Ignoring {} calibration for tracker {}: configured tracker is {}",
                hand.to_ascii_lowercase(),
                stored.serial,
                configured_serial
            );
            None
        }
    });
    let base = stored_pose.unwrap_or(fallback);
    let value =
        |suffix: &str, default: f32| env_float(&format!("FMC_{hand}_OFFSET_{suffix}"), default);
    Pose {
        orientation: normalized(Quat {
            x: value("QX", base.orientation.x),
            y: value("QY", base.orientation.y),
            z: value("QZ", base.orientation.z),
            w: value("QW", base.orientation.w),
        }),
        position: Vec3 {
            x: value("X", base.position.x),
            y: value("Y", base.position.y),
            z: value("Z", base.position.z),
        },
    }
}

fn average_orientation(samples: &[Quat]) -> Option<Quat> {
    let reference = normalized(*samples.first()?);
    let mut sum = Quat::default();
    for sample in samples {
        let mut sample = normalized(*sample);
        let dot = reference.x * sample.x
            + reference.y * sample.y
            + reference.z * sample.z
            + reference.w * sample.w;
        if dot < 0.0 {
            sample.x = -sample.x;
            sample.y = -sample.y;
            sample.z = -sample.z;
            sample.w = -sample.w;
        }
        sum.x += sample.x;
        sum.y += sample.y;
        sum.z += sample.z;
        sum.w += sample.w;
    }
    Some(normalized(sum))
}

fn headset_yaw(samples: &[Quat]) -> Option<Quat> {
    let mut forward_x = 0.0;
    let mut forward_z = 0.0;
    for sample in samples {
        let forward = rotate(
            normalized(*sample),
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            },
        );
        let horizontal_length = forward.x.hypot(forward.z);
        if horizontal_length > 0.1 {
            forward_x += forward.x / horizontal_length;
            forward_z += forward.z / horizontal_length;
        }
    }
    if forward_x.hypot(forward_z) < f32::EPSILON {
        return None;
    }
    let half_yaw = 0.5 * (-forward_x).atan2(-forward_z);
    Some(Quat {
        x: 0.0,
        y: half_yaw.sin(),
        z: 0.0,
        w: half_yaw.cos(),
    })
}

impl Calibration {
    pub(crate) fn load() -> Result<Self, String> {
        let requested_hand = match env::var("FMC_CALIBRATE_HAND") {
            Err(_) => None,
            Ok(value) if value.eq_ignore_ascii_case("left") => Some(true),
            Ok(value) if value.eq_ignore_ascii_case("right") => Some(false),
            Ok(value) => {
                return Err(format!(
                    "FMC_CALIBRATE_HAND must be 'left' or 'right', not '{value}'"
                ));
            }
        };
        let path = calibration_path();
        let values = parse_file(path.as_ref());
        let stored = [stored_hand(true, &values), stored_hand(false, &values)];
        let fallback = default_pose();
        let offsets = [
            effective_pose(true, stored[0].as_ref(), fallback),
            effective_pose(false, stored[1].as_ref(), fallback),
        ];
        Ok(Self {
            requested_hand,
            path,
            status_path: env::var_os("FMC_CALIBRATION_STATUS_FILE").map(PathBuf::from),
            offsets,
            stored,
            controller_center_down: env_float(
                "FMC_CONTROLLER_CENTER_DOWN",
                DEFAULT_CONTROLLER_CENTER_DOWN_METRES,
            ),
        })
    }

    pub(crate) fn requested(&self, is_left: bool) -> bool {
        self.requested_hand == Some(is_left)
    }

    pub(crate) fn is_requested(&self) -> bool {
        self.requested_hand.is_some()
    }

    pub(crate) fn offset(&self, is_left: bool) -> Pose {
        let mut offset = self.offsets[hand_index(is_left)];
        let center_offset = rotate(
            offset.orientation,
            Vec3 {
                x: 0.0,
                y: self.controller_center_down,
                z: 0.0,
            },
        );
        offset.position.x += center_offset.x;
        offset.position.y += center_offset.y;
        offset.position.z += center_offset.z;
        offset
    }

    pub(crate) fn duration_ms() -> u32 {
        (env_float("FMC_CALIBRATION_SECONDS", DEFAULT_SECONDS).clamp(0.1, 10.0) * 1000.0) as u32
    }

    pub(crate) fn countdown_ms() -> u32 {
        (env_float(
            "FMC_CALIBRATION_COUNTDOWN_SECONDS",
            DEFAULT_COUNTDOWN_SECONDS,
        )
        .clamp(0.0, 10.0)
            * 1000.0) as u32
    }

    pub(crate) fn save(
        &self,
        is_left: bool,
        serial: &str,
        tracker_samples: &[Quat],
        head_samples: &[Quat],
    ) -> Result<(), String> {
        if serial.is_empty() {
            return Err("tracker serial is empty".to_owned());
        }
        let tracker_orientation =
            average_orientation(tracker_samples).ok_or("no tracked tracker orientation samples")?;
        let head_yaw = headset_yaw(head_samples)
            .ok_or("headset forward direction is vertical or unavailable")?;
        let mut stored = self.stored.clone();
        let mut pose = self.offsets[hand_index(is_left)];
        let tracker_inverse = Quat {
            x: -tracker_orientation.x,
            y: -tracker_orientation.y,
            z: -tracker_orientation.z,
            w: tracker_orientation.w,
        };
        let controller_orientation = quat_mul(head_yaw, PALM_DOWN_ROLL);
        pose.orientation = normalized(quat_mul(tracker_inverse, controller_orientation));
        stored[hand_index(is_left)] = Some(StoredCalibration {
            serial: serial.to_owned(),
            pose,
        });
        let path = self
            .path
            .as_ref()
            .ok_or("set FMC_CALIBRATION_FILE because HOME and XDG_CONFIG_HOME are unavailable")?;
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|error| error.to_string())?;
        }
        let mut contents = String::from(
            "# Flow Motion Controller mount calibration\n# Position is in metres; orientation is quaternion x, y, z, w.\n",
        );
        for (is_left, stored) in [(true, &stored[0]), (false, &stored[1])] {
            let Some(stored) = stored else {
                continue;
            };
            let hand = hand_name(is_left);
            let pose = stored.pose;
            contents.push_str(&format!(
                "{hand}_SERIAL={}\n{hand}_OFFSET_X={:.9}\n{hand}_OFFSET_Y={:.9}\n{hand}_OFFSET_Z={:.9}\n{hand}_OFFSET_QX={:.9}\n{hand}_OFFSET_QY={:.9}\n{hand}_OFFSET_QZ={:.9}\n{hand}_OFFSET_QW={:.9}\n",
                stored.serial,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ));
        }
        let temporary = path.with_extension("tmp");
        fs::write(&temporary, contents).map_err(|error| error.to_string())?;
        fs::rename(&temporary, path).map_err(|error| error.to_string())?;
        eprintln!(
            "Saved {} hand calibration to {}",
            hand_name(is_left).to_ascii_lowercase(),
            path.display()
        );
        Ok(())
    }

    pub(crate) fn write_status(&self, success: bool) -> bool {
        let Some(path) = &self.status_path else {
            return true;
        };
        fs::write(path, if success { "success\n" } else { "error\n" })
            .map_err(|error| eprintln!("Failed to write calibration status: {error}"))
            .is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_angle(x: f32, y: f32, z: f32, angle: f32) -> Quat {
        let half = angle * 0.5;
        Quat {
            x: x * half.sin(),
            y: y * half.sin(),
            z: z * half.sin(),
            w: half.cos(),
        }
    }

    fn inverse(q: Quat) -> Quat {
        Quat {
            x: -q.x,
            y: -q.y,
            z: -q.z,
            w: q.w,
        }
    }

    fn assert_same_rotation(actual: Quat, expected: Quat) {
        let dot = actual.x * expected.x
            + actual.y * expected.y
            + actual.z * expected.z
            + actual.w * expected.w;
        assert!((1.0 - dot.abs()).abs() < 1e-5, "{actual:?} != {expected:?}");
    }

    fn assert_vec3_close(actual: Vec3, expected: Vec3) {
        assert!((actual.x - expected.x).abs() < 1e-6);
        assert!((actual.y - expected.y).abs() < 1e-6);
        assert!((actual.z - expected.z).abs() < 1e-6);
    }

    #[test]
    fn headset_yaw_ignores_pitch_and_roll() {
        let yaw = axis_angle(0.0, 1.0, 0.0, 1.2);
        let pitch = axis_angle(1.0, 0.0, 0.0, -0.7);
        let roll = axis_angle(0.0, 0.0, 1.0, 0.4);
        let head = normalized(quat_mul(quat_mul(yaw, pitch), roll));

        assert_same_rotation(headset_yaw(&[head]).unwrap(), yaw);
    }

    #[test]
    fn mount_offset_is_yaw_independent_and_preserves_forward() {
        let mount = normalized(quat_mul(
            axis_angle(1.0, 0.0, 0.0, 0.3),
            axis_angle(0.0, 0.0, 1.0, -0.8),
        ));
        let offset_for = |yaw: Quat| {
            let tracker = normalized(quat_mul(yaw, mount));
            let target = quat_mul(yaw, PALM_DOWN_ROLL);
            normalized(quat_mul(inverse(tracker), target))
        };
        let first_yaw = axis_angle(0.0, 1.0, 0.0, 0.4);
        let second_yaw = axis_angle(0.0, 1.0, 0.0, -1.1);
        assert_same_rotation(offset_for(first_yaw), offset_for(second_yaw));

        let forward = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        };
        let up = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let target = quat_mul(first_yaw, PALM_DOWN_ROLL);
        assert_vec3_close(rotate(target, forward), rotate(first_yaw, forward));
        let rolled_up = rotate(target, up);
        let unrolled_up = rotate(first_yaw, up);
        assert_vec3_close(
            rolled_up,
            Vec3 {
                x: -unrolled_up.x,
                y: -unrolled_up.y,
                z: -unrolled_up.z,
            },
        );
    }

    #[test]
    fn controller_center_moves_down_in_the_calibration_pose() {
        let pose = Pose {
            orientation: PALM_DOWN_ROLL,
            position: Vec3::default(),
        };
        let calibration = Calibration {
            requested_hand: None,
            path: None,
            status_path: None,
            offsets: [pose; 2],
            stored: [None, None],
            controller_center_down: DEFAULT_CONTROLLER_CENTER_DOWN_METRES,
        };

        let offset = calibration.offset(true);
        assert_vec3_close(
            offset.position,
            Vec3 {
                x: 0.0,
                y: -DEFAULT_CONTROLLER_CENTER_DOWN_METRES,
                z: 0.0,
            },
        );
    }

    #[test]
    fn pose_alignments_preserve_aim_and_apply_grip_bias() {
        let yaw = axis_angle(0.0, 1.0, 0.0, 0.7);
        let forward = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        };
        let down = Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        };

        let grip_fingers = Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        };
        let expected_grip_alignment = axis_angle(1.0, 0.0, 0.0, 77.5_f32.to_radians());

        for (is_left, outward_palm) in [
            (
                true,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
            ),
            (
                false,
                Vec3 {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
            ),
        ] {
            let aim = quat_mul(quat_mul(yaw, PALM_DOWN_ROLL), hand_pose_roll(is_left));
            let grip = quat_mul(aim, grip_pose_alignment());
            let expected_grip = quat_mul(aim, expected_grip_alignment);

            assert_vec3_close(rotate(aim, outward_palm), down);
            assert_vec3_close(rotate(aim, forward), rotate(yaw, forward));
            assert_vec3_close(rotate(grip, outward_palm), down);
            assert_vec3_close(
                rotate(grip, grip_fingers),
                rotate(expected_grip, grip_fingers),
            );
            assert_vec3_close(rotate(grip, forward), rotate(expected_grip, forward));
        }
    }
}
