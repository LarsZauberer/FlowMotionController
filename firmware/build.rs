use std::fs;
use std::path::PathBuf;

#[path = "../shared/protocol.rs"]
#[allow(dead_code)]
mod protocol;

fn setting<'a>(source: &'a str, key: &str) -> &'a str {
    source
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .find_map(|line| {
            let (name, value) = line.split_once('=')?;
            (name.trim() == key).then_some(value.trim().trim_matches('"'))
        })
        .unwrap_or_else(|| panic!("firmware-config.toml is missing {key}"))
}

fn pin(source: &str, key: &str) -> u8 {
    let value: u8 = setting(source, key)
        .parse()
        .unwrap_or_else(|_| panic!("{key} must be an RP2040 GPIO number"));
    assert!(
        value <= 29,
        "{key} must be an RP2040 GPIO number from 0 to 29"
    );
    value
}

fn boolean(source: &str, key: &str) -> bool {
    match setting(source, key) {
        "true" => true,
        "false" => false,
        value => panic!("{key} must be true or false, not {value:?}"),
    }
}

fn action(source: &str, key: &str) -> u16 {
    match setting(source, key) {
        "unmapped" => 0,
        "trigger" => protocol::BUTTON_TRIGGER,
        "switch_hand" => protocol::BUTTON_SWITCH_HAND,
        "a" => protocol::BUTTON_A,
        "b" => protocol::BUTTON_B,
        "grip" => protocol::BUTTON_GRIP,
        "system" => protocol::BUTTON_SYSTEM,
        "both_triggers" => protocol::BUTTON_BOTH_TRIGGERS,
        "joystick_button" => protocol::BUTTON_JOYSTICK,
        value => panic!("unsupported {key} action {value:?}"),
    }
}

fn main() {
    println!("cargo:rerun-if-changed=firmware-config.toml");
    println!("cargo:rerun-if-changed=memory.x");
    println!(
        "cargo:rustc-link-search={}",
        std::env::var("CARGO_MANIFEST_DIR").unwrap()
    );
    let source =
        fs::read_to_string("firmware-config.toml").expect("firmware-config.toml must be readable");
    let ssid = setting(&source, "wifi_ssid");
    let password = setting(&source, "wifi_password");
    let host: Vec<u8> = setting(&source, "monado_host")
        .split('.')
        .map(|part| part.parse().expect("monado_host must be an IPv4 address"))
        .collect();
    assert!(host.len() == 4, "monado_host must be an IPv4 address");
    let port: u16 = setting(&source, "monado_port")
        .parse()
        .expect("monado_port must be a number");
    let calibration_seconds: u32 = setting(&source, "joystick_calibration_seconds")
        .parse()
        .expect("joystick_calibration_seconds must be a positive number");
    assert!(
        calibration_seconds > 0,
        "joystick_calibration_seconds must be a positive number"
    );
    let joystick_deadzone: f32 = setting(&source, "joystick_deadzone")
        .parse()
        .expect("joystick_deadzone must be a number from 0 to less than 1");
    assert!(
        joystick_deadzone.is_finite() && (0.0..1.0).contains(&joystick_deadzone),
        "joystick_deadzone must be a number from 0 to less than 1"
    );
    let joystick_deadzone = (joystick_deadzone * 32768.0).round() as i32;
    let joystick_x_inverted = boolean(&source, "joystick_x_inverted");
    let joystick_y_inverted = boolean(&source, "joystick_y_inverted");
    let pins = [
        ("trigger1_pin", pin(&source, "trigger1_pin")),
        ("trigger2_pin", pin(&source, "trigger2_pin")),
        ("button_a_pin", pin(&source, "button_a_pin")),
        ("button_b_pin", pin(&source, "button_b_pin")),
        ("grip_pin", pin(&source, "grip_pin")),
        ("system1_pin", pin(&source, "system1_pin")),
        ("system2_pin", pin(&source, "system2_pin")),
        ("joystick_button_pin", pin(&source, "joystick_button_pin")),
        ("joystick_x_pin", pin(&source, "joystick_x_pin")),
        ("joystick_y_pin", pin(&source, "joystick_y_pin")),
    ];
    for (index, (name, value)) in pins.iter().enumerate() {
        assert!(
            !pins[..index].iter().any(|(_, other)| other == value),
            "{name} duplicates another configured GPIO"
        );
        assert!(
            !matches!(*value, 23 | 24 | 25 | 29),
            "{name} uses a GPIO reserved by the Pico WH Wi-Fi interface"
        );
    }
    for (name, value) in pins
        .iter()
        .filter(|(name, _)| matches!(*name, "joystick_x_pin" | "joystick_y_pin"))
    {
        assert!(
            (26..=29).contains(value),
            "{name} must use an RP2040 ADC GPIO from 26 to 29"
        );
    }

    let generated = format!(
        r#"
pub const WIFI_SSID: &str = {ssid:?};
pub const WIFI_PASSWORD: &str = {password:?};
pub const MONADO_HOST: [u8; 4] = [{}, {}, {}, {}];
pub const MONADO_PORT: u16 = {port};
pub const JOYSTICK_CALIBRATION_SECONDS: u32 = {calibration_seconds};
pub const JOYSTICK_DEADZONE: i32 = {joystick_deadzone};
pub const JOYSTICK_X_INVERTED: bool = {joystick_x_inverted};
pub const JOYSTICK_Y_INVERTED: bool = {joystick_y_inverted};
pub const TRIGGER1_ACTION: u16 = {};
pub const TRIGGER2_ACTION: u16 = {};
pub const BUTTON_A_ACTION: u16 = {};
pub const BUTTON_B_ACTION: u16 = {};
pub const GRIP_ACTION: u16 = {};
pub const SYSTEM1_ACTION: u16 = {};
pub const SYSTEM2_ACTION: u16 = {};
pub const JOYSTICK_BUTTON_ACTION: u16 = {};

macro_rules! configured_io {{
    ($p:ident) => {{
        (
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::gpio::Input::new($p.PIN_{}, embassy_rp::gpio::Pull::Up),
            embassy_rp::adc::Channel::new_pin($p.PIN_{}, embassy_rp::gpio::Pull::None),
            embassy_rp::adc::Channel::new_pin($p.PIN_{}, embassy_rp::gpio::Pull::None),
        )
    }};
}}
"#,
        host[0],
        host[1],
        host[2],
        host[3],
        action(&source, "trigger1_action"),
        action(&source, "trigger2_action"),
        action(&source, "button_a_action"),
        action(&source, "button_b_action"),
        action(&source, "grip_action"),
        action(&source, "system1_action"),
        action(&source, "system2_action"),
        action(&source, "joystick_button_action"),
        pins[0].1,
        pins[1].1,
        pins[2].1,
        pins[3].1,
        pins[4].1,
        pins[5].1,
        pins[6].1,
        pins[7].1,
        pins[8].1,
        pins[9].1,
    );
    let output = PathBuf::from(std::env::var_os("OUT_DIR").unwrap()).join("config.rs");
    fs::write(output, generated).expect("generated firmware config must be writable");
}
