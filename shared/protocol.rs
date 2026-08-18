pub const FRAME_MAGIC: [u8; 4] = *b"FMC1";
pub const FRAME_SIZE: usize = 16;

pub const BUTTON_TRIGGER: u16 = 1 << 0;
pub const BUTTON_SWITCH_HAND: u16 = 1 << 1;
pub const BUTTON_A: u16 = 1 << 2;
pub const BUTTON_B: u16 = 1 << 3;
pub const BUTTON_GRIP: u16 = 1 << 4;
pub const BUTTON_SYSTEM: u16 = 1 << 5;
pub const BUTTON_BOTH_TRIGGERS: u16 = 1 << 6;
pub const BUTTON_JOYSTICK: u16 = 1 << 7;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputFrame {
    pub sequence: u32,
    pub buttons: u16,
    pub joystick_x: i16,
    pub joystick_y: i16,
}

impl InputFrame {
    pub fn encode(self, output: &mut [u8; FRAME_SIZE]) {
        output[..4].copy_from_slice(&FRAME_MAGIC);
        output[4..8].copy_from_slice(&self.sequence.to_le_bytes());
        output[8..10].copy_from_slice(&self.buttons.to_le_bytes());
        output[10..12].fill(0);
        output[12..14].copy_from_slice(&self.joystick_x.to_le_bytes());
        output[14..16].copy_from_slice(&self.joystick_y.to_le_bytes());
    }

    pub fn decode(input: &[u8]) -> Option<Self> {
        if input.len() < FRAME_SIZE || input[..4] != FRAME_MAGIC {
            return None;
        }

        Some(Self {
            sequence: u32::from_le_bytes(input[4..8].try_into().ok()?),
            buttons: u16::from_le_bytes(input[8..10].try_into().ok()?),
            joystick_x: i16::from_le_bytes(input[12..14].try_into().ok()?),
            joystick_y: i16::from_le_bytes(input[14..16].try_into().ok()?),
        })
    }
}
