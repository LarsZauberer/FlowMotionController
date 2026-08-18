#![allow(clippy::missing_safety_doc)]

mod calibration;
#[path = "../shared/protocol.rs"]
#[allow(dead_code)]
mod protocol;

use calibration::{Calibration, grip_pose_alignment, hand_pose_roll};
use protocol::{
    BUTTON_A, BUTTON_B, BUTTON_BOTH_TRIGGERS, BUTTON_GRIP, BUTTON_JOYSTICK, BUTTON_SWITCH_HAND,
    BUTTON_SYSTEM, BUTTON_TRIGGER, FRAME_SIZE, InputFrame,
};
use std::env;
use std::ffi::{CStr, c_char, c_void};
use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::{slice, thread};

const INPUT_TIMEOUT: Duration = Duration::from_millis(500);
const SWITCH_DEBOUNCE: Duration = Duration::from_millis(20);

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FlowMotionInput {
    pub trigger: u8,
    pub a: u8,
    pub b: u8,
    pub grip: u8,
    pub system: u8,
    pub joystick_button: u8,
    pub joystick_x: f32,
    pub joystick_y: f32,
}

#[derive(Clone, Copy, Debug)]
struct ReceivedFrame {
    frame: InputFrame,
    received_at: Instant,
}

struct SharedState {
    latest: Option<ReceivedFrame>,
    peer: Option<SocketAddr>,
    active_right: bool,
    switch_was_pressed: bool,
    switch_changed_at: Option<Instant>,
}

impl SharedState {
    fn new() -> Self {
        Self {
            latest: None,
            peer: None,
            active_right: true,
            switch_was_pressed: false,
            switch_changed_at: None,
        }
    }
}

struct Runtime {
    shared: Arc<Mutex<SharedState>>,
    stop: Arc<AtomicBool>,
    server: Mutex<Option<JoinHandle<()>>>,
    calibration: Calibration,
}

impl Drop for Runtime {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(server) = self.server.lock().expect("server lock poisoned").take() {
            let _ = server.join();
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FlowMotionVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FlowMotionQuat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FlowMotionRelation {
    pub relation_flags: i32,
    pub orientation: FlowMotionQuat,
    pub position: FlowMotionVec3,
    pub linear_velocity: FlowMotionVec3,
    pub angular_velocity: FlowMotionVec3,
}

type Vec3 = FlowMotionVec3;
type Quat = FlowMotionQuat;

fn env_string(name: &str, default: &str) -> String {
    env::var(name).unwrap_or_else(|_| default.to_owned())
}

pub(crate) fn normalized(q: Quat) -> Quat {
    let length = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    if length == 0.0 {
        Quat {
            w: 1.0,
            ..Quat::default()
        }
    } else {
        Quat {
            x: q.x / length,
            y: q.y / length,
            z: q.z / length,
            w: q.w / length,
        }
    }
}

pub(crate) fn quat_mul(a: Quat, b: Quat) -> Quat {
    Quat {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    }
}

pub(crate) fn rotate(q: Quat, v: Vec3) -> Vec3 {
    let qv = Vec3 {
        x: q.y * v.z - q.z * v.y,
        y: q.z * v.x - q.x * v.z,
        z: q.x * v.y - q.y * v.x,
    };
    let qqv = Vec3 {
        x: q.y * qv.z - q.z * qv.y,
        y: q.z * qv.x - q.x * qv.z,
        z: q.x * qv.y - q.y * qv.x,
    };
    Vec3 {
        x: v.x + 2.0 * (q.w * qv.x + qqv.x),
        y: v.y + 2.0 * (q.w * qv.y + qqv.y),
        z: v.z + 2.0 * (q.w * qv.z + qqv.z),
    }
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn handle_frame(shared: &Arc<Mutex<SharedState>>, peer: SocketAddr, frame: InputFrame) {
    handle_frame_at(shared, peer, frame, Instant::now());
}

fn handle_frame_at(
    shared: &Arc<Mutex<SharedState>>,
    peer: SocketAddr,
    frame: InputFrame,
    now: Instant,
) {
    let mut shared = shared.lock().expect("input lock poisoned");
    let session_expired = shared
        .latest
        .is_none_or(|previous| now.duration_since(previous.received_at) > INPUT_TIMEOUT);
    if !session_expired && shared.peer != Some(peer) {
        return;
    }
    if session_expired {
        shared.peer = Some(peer);
        shared.switch_was_pressed = false;
        shared.switch_changed_at = None;
    } else if let Some(previous) = shared.latest
        && !sequence_is_newer(frame.sequence, previous.frame.sequence)
    {
        return;
    }
    let pressed = frame.buttons & BUTTON_SWITCH_HAND != 0;
    if pressed != shared.switch_was_pressed
        && shared
            .switch_changed_at
            .is_none_or(|changed_at| now.duration_since(changed_at) >= SWITCH_DEBOUNCE)
    {
        shared.switch_was_pressed = pressed;
        shared.switch_changed_at = Some(now);
        if pressed {
            shared.active_right = !shared.active_right;
        }
    }
    shared.latest = Some(ReceivedFrame {
        frame,
        received_at: now,
    });
}

fn sequence_is_newer(sequence: u32, previous: u32) -> bool {
    let difference = sequence.wrapping_sub(previous);
    difference != 0 && difference < (1 << 31)
}

fn run_server(socket: UdpSocket, shared: Arc<Mutex<SharedState>>, stop: Arc<AtomicBool>) {
    let mut receive = [0_u8; 256];
    while !stop.load(Ordering::Acquire) {
        match socket.recv_from(&mut receive) {
            Ok((FRAME_SIZE, peer)) => {
                if let Some(frame) = InputFrame::decode(&receive[..FRAME_SIZE]) {
                    handle_frame(&shared, peer, frame);
                }
            }
            Ok(_) => {}
            Err(error)
                if matches!(
                    error.kind(),
                    ErrorKind::Interrupted | ErrorKind::TimedOut | ErrorKind::WouldBlock
                ) => {}
            Err(error) => {
                eprintln!("Flow Motion Controller socket stopped: {error}");
                return;
            }
        }
    }
}

fn new_runtime() -> Option<Runtime> {
    let calibration = match Calibration::load() {
        Ok(calibration) => calibration,
        Err(error) => {
            eprintln!("{error}");
            return None;
        }
    };
    let shared = Arc::new(Mutex::new(SharedState::new()));
    let stop = Arc::new(AtomicBool::new(false));
    let server = if calibration.is_requested() {
        None
    } else {
        let address = env_string("FMC_LISTEN_ADDR", "0.0.0.0:4242");
        let socket = UdpSocket::bind(&address).ok()?;
        socket
            .set_read_timeout(Some(Duration::from_millis(100)))
            .ok()?;
        let server_shared = Arc::clone(&shared);
        let server_stop = Arc::clone(&stop);
        let server = thread::Builder::new()
            .name("flow-motion-controller".to_owned())
            .spawn(move || run_server(socket, server_shared, server_stop))
            .ok()?;
        eprintln!("Flow Motion Controller listening for UDP input on {address}");
        Some(server)
    };
    Some(Runtime {
        shared,
        stop,
        server: Mutex::new(server),
        calibration,
    })
}

fn poll_runtime(runtime: &Runtime, is_left: bool) -> FlowMotionInput {
    let shared = runtime.shared.lock().expect("input lock poisoned");
    let Some(received) = shared.latest else {
        return FlowMotionInput::default();
    };

    if received.received_at.elapsed() > INPUT_TIMEOUT {
        return FlowMotionInput::default();
    }

    input_for_hand(received.frame, shared.active_right, is_left)
}

fn input_for_hand(frame: InputFrame, active_right: bool, is_left: bool) -> FlowMotionInput {
    let both_triggers = frame.buttons & BUTTON_BOTH_TRIGGERS != 0;
    if active_right == is_left {
        return FlowMotionInput {
            trigger: u8::from(both_triggers),
            ..FlowMotionInput::default()
        };
    }

    let buttons = frame.buttons;
    FlowMotionInput {
        trigger: u8::from(buttons & BUTTON_TRIGGER != 0 || both_triggers),
        a: u8::from(buttons & BUTTON_A != 0),
        b: u8::from(buttons & BUTTON_B != 0),
        grip: u8::from(buttons & BUTTON_GRIP != 0),
        system: u8::from(buttons & BUTTON_SYSTEM != 0),
        joystick_button: u8::from(buttons & BUTTON_JOYSTICK != 0),
        joystick_x: f32::from(frame.joystick_x) / f32::from(i16::MAX),
        joystick_y: f32::from(frame.joystick_y) / f32::from(i16::MAX),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn flow_motion_rust_create() -> *mut c_void {
    new_runtime()
        .map(|runtime| Box::into_raw(Box::new(Arc::new(runtime))).cast())
        .unwrap_or(std::ptr::null_mut())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_clone(state: *const c_void) -> *mut c_void {
    if state.is_null() {
        return std::ptr::null_mut();
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    Box::into_raw(Box::new(Arc::clone(runtime))).cast()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_destroy(state: *mut c_void) {
    if !state.is_null() {
        unsafe { drop(Box::from_raw(state.cast::<Arc<Runtime>>())) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_poll(
    state: *const c_void,
    is_left: bool,
    out_input: *mut FlowMotionInput,
) -> bool {
    if state.is_null() || out_input.is_null() {
        return false;
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    unsafe { *out_input = poll_runtime(runtime, is_left) };
    true
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_should_calibrate(
    state: *const c_void,
    is_left: bool,
) -> bool {
    if state.is_null() {
        return false;
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    runtime.calibration.requested(is_left)
}

#[unsafe(no_mangle)]
pub extern "C" fn flow_motion_rust_calibration_duration_ms() -> u32 {
    Calibration::duration_ms()
}

#[unsafe(no_mangle)]
pub extern "C" fn flow_motion_rust_calibration_countdown_ms() -> u32 {
    Calibration::countdown_ms()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_save_calibration(
    state: *const c_void,
    is_left: bool,
    serial: *const c_char,
    tracker_samples: *const FlowMotionQuat,
    head_samples: *const FlowMotionQuat,
    sample_count: usize,
) -> bool {
    if state.is_null()
        || serial.is_null()
        || tracker_samples.is_null()
        || head_samples.is_null()
        || sample_count == 0
    {
        return false;
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    let serial = unsafe { CStr::from_ptr(serial) };
    let Ok(serial) = serial.to_str() else {
        eprintln!("Tracker serial is not valid UTF-8");
        return false;
    };
    let tracker_samples = unsafe { slice::from_raw_parts(tracker_samples, sample_count) };
    let head_samples = unsafe { slice::from_raw_parts(head_samples, sample_count) };
    match runtime
        .calibration
        .save(is_left, serial, tracker_samples, head_samples)
    {
        Ok(()) => true,
        Err(error) => {
            eprintln!("Failed to save calibration: {error}");
            false
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_write_calibration_status(
    state: *const c_void,
    success: bool,
) -> bool {
    if state.is_null() {
        return false;
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    runtime.calibration.write_status(success)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flow_motion_rust_apply_offset(
    state: *const c_void,
    is_left: bool,
    is_grip: bool,
    relation: *mut FlowMotionRelation,
) {
    if state.is_null() || relation.is_null() {
        return;
    }
    let runtime = unsafe { &*state.cast::<Arc<Runtime>>() };
    let relation = unsafe { &mut *relation };
    let offset = runtime.calibration.offset(is_left);
    let world_offset = rotate(relation.orientation, offset.position);
    relation.position.x += world_offset.x;
    relation.position.y += world_offset.y;
    relation.position.z += world_offset.z;
    relation.orientation = normalized(quat_mul(relation.orientation, offset.orientation));
    relation.orientation = normalized(quat_mul(relation.orientation, hand_pose_roll(is_left)));
    if is_grip {
        relation.orientation = normalized(quat_mul(relation.orientation, grip_pose_alignment()));
    }
    let tangential_velocity = cross(relation.angular_velocity, world_offset);
    relation.linear_velocity.x += tangential_velocity.x;
    relation.linear_velocity.y += tangential_velocity.y;
    relation.linear_velocity.z += tangential_velocity.z;
}

#[cfg(test)]
mod input_tests {
    use super::*;

    #[test]
    fn sequence_order_handles_duplicates_and_wraparound() {
        assert!(sequence_is_newer(11, 10));
        assert!(!sequence_is_newer(10, 10));
        assert!(!sequence_is_newer(9, 10));
        assert!(sequence_is_newer(0, u32::MAX));
    }

    #[test]
    fn both_triggers_reaches_both_hands_without_other_inputs() {
        let frame = InputFrame {
            buttons: BUTTON_BOTH_TRIGGERS | BUTTON_A,
            ..InputFrame::default()
        };

        let right = input_for_hand(frame, true, false);
        let left = input_for_hand(frame, true, true);
        assert_eq!(right.trigger, 1);
        assert_eq!(right.a, 1);
        assert_eq!(left.trigger, 1);
        assert_eq!(left.a, 0);
    }

    #[test]
    fn switch_bounce_does_not_toggle_hands_twice() {
        let shared = Arc::new(Mutex::new(SharedState::new()));
        let start = Instant::now();
        let peer = "127.0.0.1:4242".parse().unwrap();
        let frame = |sequence, pressed| InputFrame {
            sequence,
            buttons: if pressed { BUTTON_SWITCH_HAND } else { 0 },
            ..InputFrame::default()
        };

        handle_frame_at(&shared, peer, frame(0, true), start);
        handle_frame_at(
            &shared,
            peer,
            frame(1, false),
            start + Duration::from_millis(1),
        );
        handle_frame_at(
            &shared,
            peer,
            frame(2, true),
            start + Duration::from_millis(2),
        );
        handle_frame_at(
            &shared,
            peer,
            frame(3, false),
            start + Duration::from_millis(21),
        );
        handle_frame_at(
            &shared,
            peer,
            frame(4, true),
            start + Duration::from_millis(22),
        );
        handle_frame_at(
            &shared,
            peer,
            frame(5, false),
            start + Duration::from_millis(23),
        );
        assert!(!shared.lock().unwrap().active_right);

        handle_frame_at(
            &shared,
            peer,
            frame(6, true),
            start + Duration::from_millis(42),
        );
        assert!(shared.lock().unwrap().active_right);
    }

    #[test]
    fn another_udp_sender_waits_for_the_active_session_to_expire() {
        let shared = Arc::new(Mutex::new(SharedState::new()));
        let start = Instant::now();
        let first_peer = "127.0.0.1:4242".parse().unwrap();
        let second_peer = "127.0.0.1:4243".parse().unwrap();

        handle_frame_at(
            &shared,
            first_peer,
            InputFrame {
                sequence: 10,
                ..InputFrame::default()
            },
            start,
        );
        handle_frame_at(
            &shared,
            second_peer,
            InputFrame {
                sequence: 11,
                ..InputFrame::default()
            },
            start + Duration::from_millis(1),
        );
        assert_eq!(shared.lock().unwrap().latest.unwrap().frame.sequence, 10);

        handle_frame_at(
            &shared,
            second_peer,
            InputFrame {
                sequence: 0,
                ..InputFrame::default()
            },
            start + INPUT_TIMEOUT + Duration::from_millis(1),
        );
        let state = shared.lock().unwrap();
        assert_eq!(state.peer, Some(second_peer));
        assert_eq!(state.latest.unwrap().frame.sequence, 0);
    }
}
