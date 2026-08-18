#[path = "../../shared/protocol.rs"]
#[allow(dead_code)]
mod protocol;

use protocol::{
    BUTTON_A, BUTTON_B, BUTTON_BOTH_TRIGGERS, BUTTON_GRIP, BUTTON_JOYSTICK, BUTTON_SWITCH_HAND,
    BUTTON_SYSTEM, BUTTON_TRIGGER, FRAME_SIZE, InputFrame,
};
use std::env;
use std::net::UdpSocket;

const KNOWN_BUTTONS: u16 = BUTTON_TRIGGER
    | BUTTON_SWITCH_HAND
    | BUTTON_A
    | BUTTON_B
    | BUTTON_GRIP
    | BUTTON_SYSTEM
    | BUTTON_BOTH_TRIGGERS
    | BUTTON_JOYSTICK;

fn action_names(buttons: u16) -> String {
    let actions = [
        (BUTTON_TRIGGER, "Trigger"),
        (BUTTON_SWITCH_HAND, "SwitchHand"),
        (BUTTON_A, "A"),
        (BUTTON_B, "B"),
        (BUTTON_GRIP, "Grip"),
        (BUTTON_SYSTEM, "System"),
        (BUTTON_BOTH_TRIGGERS, "BothTriggers"),
        (BUTTON_JOYSTICK, "JoystickButton"),
    ];
    let mut names = actions
        .into_iter()
        .filter_map(|(mask, name)| (buttons & mask != 0).then_some(name.to_owned()))
        .collect::<Vec<_>>();
    if buttons & !KNOWN_BUTTONS != 0 {
        names.push(format!("Unknown(0x{:04x})", buttons & !KNOWN_BUTTONS));
    }
    if names.is_empty() {
        "none".to_owned()
    } else {
        names.join(", ")
    }
}

fn axis(value: i16) -> f32 {
    f32::from(value) / f32::from(i16::MAX)
}

fn print_frame(peer: &str, frame: InputFrame) {
    println!(
        "{peer}: seq={} actions=[{}] joystick=({:.3}, {:.3})",
        frame.sequence,
        action_names(frame.buttons),
        axis(frame.joystick_x),
        axis(frame.joystick_y),
    );
}

fn main() {
    let address = env::var("FMC_DEBUG_LISTEN_ADDR")
        .or_else(|_| env::var("FMC_LISTEN_ADDR"))
        .unwrap_or_else(|_| "0.0.0.0:4242".to_owned());
    let socket = UdpSocket::bind(&address)
        .unwrap_or_else(|error| panic!("cannot bind debug server to {address}: {error}"));
    println!("Flow Motion debug server listening for UDP input on {address}");

    let mut receive = [0_u8; 256];
    loop {
        match socket.recv_from(&mut receive) {
            Ok((FRAME_SIZE, peer)) => {
                if let Some(frame) = InputFrame::decode(&receive[..FRAME_SIZE]) {
                    print_frame(&peer.to_string(), frame);
                } else {
                    eprintln!("{peer}: discarded invalid input frame");
                }
            }
            Ok((size, peer)) => eprintln!("{peer}: discarded {size}-byte input datagram"),
            Err(error) => eprintln!("failed to receive controller input: {error}"),
        }
    }
}
