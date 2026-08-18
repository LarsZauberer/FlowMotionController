#![no_std]
#![no_main]

use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use cyw43_setup::{CLM, FW, NVRAM};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, Ipv4Address, StackResources};
use embassy_rp::adc::{Adc, InterruptHandler as AdcInterruptHandler};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

#[path = "../../shared/protocol.rs"]
#[allow(dead_code)]
mod protocol;

use protocol::{FRAME_SIZE, InputFrame};

include!(concat!(env!("OUT_DIR"), "/config.rs"));

use panic_halt as _;

const INPUT_SAMPLE_INTERVAL: Duration = Duration::from_millis(1);
const INPUT_KEEPALIVE_INTERVAL: Duration = Duration::from_millis(100);
const CHANGE_TRANSMIT_COUNT: u8 = 3;

#[unsafe(link_section = ".bi_entries")]
#[used]
static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Flow Motion Controller Pico WH"),
    embassy_rp::binary_info::rp_program_description!(
        c"Reads configurable controller buttons and sends them to Monado over Wi-Fi."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn network_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

fn axis(value: u16, center: u16) -> i16 {
    let value = i32::from(value);
    let center = i32::from(center);
    let scaled = if value < center {
        (value - center) * (i32::from(i16::MAX) + 1) / center.max(1)
    } else {
        (value - center) * i32::from(i16::MAX) / (4095 - center).max(1)
    };
    scaled.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

fn deadzone(value: i16) -> i16 {
    let value = i32::from(value);
    let magnitude = value.abs();
    if magnitude <= JOYSTICK_DEADZONE {
        return 0;
    }

    let full_scale = if value < 0 {
        i32::from(i16::MIN).abs()
    } else {
        i32::from(i16::MAX)
    };
    let scaled = (magnitude - JOYSTICK_DEADZONE) * full_scale / (full_scale - JOYSTICK_DEADZONE);
    (if value < 0 { -scaled } else { scaled }) as i16
}

fn joystick_axis(value: u16, center: u16, inverted: bool) -> i16 {
    let value = deadzone(axis(value, center));
    if inverted {
        value.saturating_neg()
    } else {
        value
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let (
        trigger1,
        trigger2,
        button_a,
        button_b,
        grip,
        system1,
        system2,
        joystick_button,
        mut joystick_x,
        mut joystick_y,
    ) = configured_io!(p);
    let mut adc = Adc::new(p.ADC, Irqs, Default::default());
    let mut joystick_x_sum = 0u64;
    let mut joystick_y_sum = 0u64;
    let mut calibration_samples = 0u64;
    let calibration_end =
        Instant::now() + Duration::from_secs(u64::from(JOYSTICK_CALIBRATION_SECONDS));
    while Instant::now() < calibration_end {
        joystick_x_sum += u64::from(adc.read(&mut joystick_x).await.unwrap_or(2048));
        joystick_y_sum += u64::from(adc.read(&mut joystick_y).await.unwrap_or(2048));
        calibration_samples += 1;
        Timer::after(Duration::from_millis(1)).await;
    }
    let joystick_x_center = (joystick_x_sum / calibration_samples) as u16;
    let joystick_y_center = (joystick_y_sum / calibration_samples) as u16;

    let power = Output::new(p.PIN_23, Level::Low);
    let chip_select = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        chip_select,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    static WIFI_STATE: StaticCell<cyw43::State> = StaticCell::new();
    let wifi_state = WIFI_STATE.init(cyw43::State::new());
    let (network_device, mut control, wifi_runner) =
        cyw43::new(wifi_state, power, spi, &FW, &NVRAM).await;
    spawner.spawn(cyw43_task(wifi_runner).unwrap());
    control.init(CLM).await;

    for _ in 0..5 {
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(100)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(100)).await;
    }

    loop {
        if control
            .join(WIFI_SSID, cyw43::JoinOptions::new(WIFI_PASSWORD.as_bytes()))
            .await
            .is_ok()
        {
            break;
        }

        for _ in 0..3 {
            control.gpio_set(0, true).await;
            Timer::after(Duration::from_millis(100)).await;
            control.gpio_set(0, false).await;
            Timer::after(Duration::from_millis(100)).await;
        }
        Timer::after(Duration::from_millis(1500)).await;
    }

    control.gpio_set(0, true).await;

    let config = Config::dhcpv4(Default::default());
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        network_device,
        config,
        RESOURCES.init(StackResources::new()),
        0x464c4f575f4d4f54,
    );
    spawner.spawn(network_task(runner).unwrap());
    stack.wait_config_up().await;
    let mut led_on = true;

    let host = Ipv4Address::new(
        MONADO_HOST[0],
        MONADO_HOST[1],
        MONADO_HOST[2],
        MONADO_HOST[3],
    );
    let mut next_sequence: u32 = 0;
    let mut transmit_sequence = 0;
    let mut transmits_left = 0;
    let mut last_input = None;
    let mut last_sent = Instant::now();
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buffer = [0; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buffer = [0; FRAME_SIZE];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket.bind(0).unwrap();

    loop {
        let mut buttons = 0;
        if trigger1.is_low() {
            buttons |= TRIGGER1_ACTION;
        }
        if trigger2.is_low() {
            buttons |= TRIGGER2_ACTION;
        }
        if button_a.is_low() {
            buttons |= BUTTON_A_ACTION;
        }
        if button_b.is_low() {
            buttons |= BUTTON_B_ACTION;
        }
        if grip.is_low() {
            buttons |= GRIP_ACTION;
        }
        if system1.is_low() {
            buttons |= SYSTEM1_ACTION;
        }
        if system2.is_low() {
            buttons |= SYSTEM2_ACTION;
        }
        if joystick_button.is_low() {
            buttons |= JOYSTICK_BUTTON_ACTION;
        }
        let joystick_x = joystick_axis(
            adc.read(&mut joystick_x).await.unwrap_or(2048),
            joystick_x_center,
            JOYSTICK_X_INVERTED,
        );
        let joystick_y = joystick_axis(
            adc.read(&mut joystick_y).await.unwrap_or(2048),
            joystick_y_center,
            JOYSTICK_Y_INVERTED,
        );
        let input = (buttons, joystick_x, joystick_y);
        if last_input != Some(input) {
            last_input = Some(input);
            transmit_sequence = next_sequence;
            next_sequence = next_sequence.wrapping_add(1);
            transmits_left = CHANGE_TRANSMIT_COUNT;
        } else if transmits_left == 0 && last_sent.elapsed() >= INPUT_KEEPALIVE_INTERVAL {
            transmit_sequence = next_sequence;
            next_sequence = next_sequence.wrapping_add(1);
            transmits_left = 1;
        }
        if transmits_left > 0 {
            let frame = InputFrame {
                sequence: transmit_sequence,
                buttons,
                joystick_x,
                joystick_y,
            };
            let mut packet = [0; FRAME_SIZE];
            frame.encode(&mut packet);
            if socket.send_to(&packet, (host, MONADO_PORT)).await.is_err() {
                if led_on {
                    control.gpio_set(0, false).await;
                    led_on = false;
                }
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
            if !led_on {
                control.gpio_set(0, true).await;
                led_on = true;
            }
            last_sent = Instant::now();
            transmits_left -= 1;
        }
        Timer::after(INPUT_SAMPLE_INTERVAL).await;
    }
}
