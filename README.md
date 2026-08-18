# Flow Motion Controller

> [!WARNING]
> This is a personal, proof of concept, vibe coding project. This is a project
> for the monado patch to support my custom built OpenXR controllers. At some
> point, I will make a clean and polished version of this project.
>
> Please note that also all further README and tutorial documentation is all
> AI written and therefore might include poorly written explanations.

This repository builds a proof-of-concept Flow Motion Controller from two
Lighthouse Vive Trackers and one configurable Raspberry Pi Pico WH controller
block.

## Quick start

1. Edit [`firmware/firmware-config.toml`](firmware/firmware-config.toml) with
   the Wi-Fi credentials, Monado host, port, GPIO pinout, and button actions.
2. Build the patched Monado package and the firmware:

   ```sh
   nix build .#monado
   FMC_FIRMWARE_CONFIG="$PWD/firmware/firmware-config.toml" \
     nix build --impure .#firmware -o firmware-result
   ```

   The firmware configuration stays ignored by Git. `FMC_FIRMWARE_CONFIG`
   explicitly passes that local file to the otherwise isolated Nix build, and
   `--impure` permits Nix to read it. The resulting firmware contains the Wi-Fi
   credentials, so do not upload it or its Nix closure to a binary cache.

   An example config looks like this

   ```toml
    wifi_ssid                    = ""
    wifi_password                = ""
    monado_host                  = ""
    monado_port                  = 4242
    joystick_calibration_seconds = 1
    joystick_deadzone            = 0.1
    joystick_x_inverted          = true
    joystick_y_inverted          = false

    trigger1_pin        = 2
    trigger2_pin        = 3
    button_a_pin        = 4
    button_b_pin        = 5
    grip_pin            = 1
    system1_pin         = 6
    system2_pin         = 7
    joystick_button_pin = 0
    joystick_x_pin      = 27
    joystick_y_pin      = 26

    trigger1_action        = "trigger"
    trigger2_action        = "switch_hand"
    button_a_action        = "a"
    button_b_action        = "b"
    grip_action            = "grip"
    system1_action         = "system"
    system2_action         = "both_triggers"
    joystick_button_action = "joystick_button"
   ```

3. Put the Pico in BOOTSEL mode and flash the resulting ELF:

   ```sh
   sudo picotool load -u -v -x -t elf \
     firmware-result/flow_motion_controller_firmware
   ```

4. Discover the exact serial numbers of the left and right Vive Trackers,
   then start the service:

   ```sh
   FMC_ENABLE=1 \
   STEAMVR_LH_ENABLE=1 \
   FMC_LEFT_TRACKER_SERIAL=LHR-LEFT_SERIAL \
   FMC_RIGHT_TRACKER_SERIAL=LHR-RIGHT_SERIAL \
   FMC_LISTEN_ADDR=0.0.0.0:4242 \
   ./result/bin/monado-service
   ```

The Pico samples input every 1 ms and sends a 16-byte UDP frame immediately
when a button or joystick value changes. It repeats a changed state three times
at 1 ms intervals, while a newer state supersedes pending repeats. An
unchanged-state frame every 100 ms keeps held inputs alive. Monado exposes two
Index-controller devices: each keeps the pose of its configured tracker, while
only the selected hand receives the normal button and joystick values. The
switch button starts in right-hand mode and toggles to the left hand and back.
`System2` sends the trigger action to both hands simultaneously for VRChat FBT
calibration, and the grip button supplies both Index squeeze value and force so
WayVR can grab windows.

The complete explanation, configuration reference, protocol, Monado patch,
firmware build, and troubleshooting guide are in
[the tutorial](docs/tutorial.md).

To test the Pico connection without Monado, stop the Monado service and run:

```sh
FMC_DEBUG_LISTEN_ADDR=0.0.0.0:4242 \
nix develop -c cargo run --bin flow-motion-debug-server
```

The debug server prints every decoded datagram, its source, action names, and
joystick values.

### Tracker mount calibration

After deploying the patched Monado package, calibrate each tracker once:

```sh
nix run .#calibrate -- left
nix run .#calibrate -- right
```

During the logged sampling period, hold the selected hand flat with the palm
down and keep its fingers pointing in the same horizontal direction as the
headset. Looking down at the hand is fine because headset pitch is ignored; do
not turn your head left or right relative to the fingers. The command
temporarily restarts the user Monado service in
one-shot calibration mode. Monado samples both the tracker and headset
orientation for one second, uses only the headset yaw, and writes the per-hand
result to `~/.config/flow-motion-controller/calibration.conf`, exits, and
restores the service if it was running before calibration. Normal starts only
read this file, so calibration is needed again only after the tracker mounting
changes.

The generated file also contains tracker-local XYZ offsets in metres. Rotation
is calibrated automatically; measure or tune the positional offsets manually.
The virtual controller center is additionally moved 3 cm into the hand along
the calibrated palm-down direction. Override this distance with
`FMC_CONTROLLER_CENTER_DOWN` in metres.

### Status LED patterns

The Pico WH onboard LED reports the controller's network state:

- Boot: five rapid flashes, with 100 ms on and 100 ms off.
- Wi-Fi search: three short flashes, with 100 ms on and 100 ms off, followed
  by a 1.5 s pause. This repeats while the firmware retries the Wi-Fi join.
- UDP send failure: off while the network stack cannot queue input datagrams.
- Ready: solid on while Wi-Fi is up and input datagrams can be sent.

UDP cannot report whether a remote listener exists, so solid on proves local
Wi-Fi and sending readiness rather than a running Monado service.

## Build targets

```sh
nix build .#monado
FMC_FIRMWARE_CONFIG="$PWD/firmware/firmware-config.toml" \
  nix build --impure .#firmware -o firmware-result
nix develop
```

The host library has focused tests for calibration, pose composition, UDP
ordering and source ownership, dual-trigger routing, and switch debounce. Real
Lighthouse tracking, the Pico GPIO wiring, Wi-Fi, and OpenXR/OpenVR action
routing still need verification on the hardware.
