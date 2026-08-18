# Building and understanding the Flow Motion Controller

This guide follows one input from a physical button to an OpenXR action. It
assumes a Raspberry Pi Pico WH, two Vive Trackers visible to Monado's
SteamVR Lighthouse driver, and a Linux machine running the patched Monado service.

## 1. The complete path

The controller has two independent data sources:

```text
left Vive Tracker  ───────────────┐
                                   ├─ Monado Flow Motion xrt_device ── OpenXR
right Vive Tracker ───────────────┘

Pico GPIOs ── Pico firmware ── Wi-Fi/UDP ── Rust socket thread in Monado
```

The trackers already work as Lighthouse devices. The Flow Motion patch does
not replace their tracking driver. It borrows each tracker and creates a thin
Index-controller device whose grip and aim poses come from that tracker.

The Pico is responsible for electrical input and transport only. It reads the
configured buttons and two analog joystick channels, packs them into a small
binary frame, and sends changes immediately plus a low-rate keepalive. Rust in
Monado owns the socket, decodes frames, applies the hand-selection state
machine, and presents a C ABI to the small Monado adapter.

This split is deliberate. GPIO and Wi-Fi code belongs on the Pico; OpenXR
device creation belongs in Monado; shared protocol and state handling stay in
Rust instead of growing a second C implementation.

## 2. Repository layout

```text
src/lib.rs                              Rust Monado-side static library
shared/protocol.rs                      Shared 16-byte frame definition
monado/flow_motion_controller.c         xrt_device adapter
monado/flow_motion_controller.h         Monado-facing interface
monado/flow_motion_controller_bindings.h cbindgen output
monado/integration.patch                Monado CMake and builder patch
firmware/firmware-config.toml           Wi-Fi, pin, and action configuration
firmware/build.rs                       Generates typed firmware constants
firmware/src/main.rs                    Pico WH Embassy application
flake.nix                               Development shell and packages
```

The generated C header is checked in because Monado's package build should not
need a Rust toolchain or a network connection just to regenerate a header. If
the exported Rust ABI changes, regenerate it from the repository root:

```sh
cbindgen --config cbindgen.toml --crate flow_motion_controller \
  --output monado/flow_motion_controller_bindings.h
```

The flake's Rust development shell supplies the compiler and normal build
tools. The header itself is generated with cbindgen from the Rust exports.

## 3. Configure the controller block

Open `firmware/firmware-config.toml`. It is the file to change when the
unknown controller-block pinout becomes available. No firmware source edit is
needed for the normal wiring and mapping changes.

### Wi-Fi endpoint

```toml
wifi_ssid = "my-vr-network"
wifi_password = "a-password"
monado_host = "192.168.1.10"
monado_port = 4242
joystick_calibration_seconds = 1
joystick_deadzone = 0.1
joystick_x_inverted = true
joystick_y_inverted = false
```

The Pico uses DHCP after joining the network. `monado_host` is the computer
running Monado, not the Pico's own address. The Monado listener binds to
`FMC_LISTEN_ADDR`, which defaults to `0.0.0.0:4242`.

### GPIO pins

Button inputs use the Pico's internal pull-up. Wire each button between its
GPIO and ground, so an electrically low pin means pressed:

```toml
trigger1_pin = 2
trigger2_pin = 3
button_a_pin = 4
button_b_pin = 5
grip_pin = 6
system1_pin = 7
system2_pin = 8
joystick_button_pin = 9
joystick_x_pin = 26
joystick_y_pin = 27
```

The two joystick pins are ADC inputs. RP2040 ADC-capable pins are required for
those two settings. The firmware converts a 12-bit-style reading centered at
the measured boot-time center into the OpenXR-style `[-1, 1]` range and sends
it as a signed 16-bit integer. Keep the stick untouched and centered while the
firmware boots: it samples for `joystick_calibration_seconds` (1 second by
default) and uses the average as the `(0, 0)` position. Samples are taken about
once per millisecond, spreading calibration across the configured duration
instead of one short burst. The lower and upper ADC ranges are then
interpolated independently to the signed output limits.
`joystick_deadzone` removes the inner fraction of that range (`0.1` removes
the inner 10% by default) and rescales the remaining movement, so the edge of
the stick still produces the full output value. Set either
`joystick_x_inverted` or `joystick_y_inverted` to `true` to reverse that axis.
The controller block's horizontal X axis is inverted; its vertical Y axis is
not.

After initializing the CYW43439, the firmware uses `WL_GPIO0` as a status LED.
It stays on while Wi-Fi is up and UDP datagrams can be queued, and turns off
while a UDP send fails. Because UDP has no connection handshake, the LED cannot
tell whether Monado is listening at the configured address.

The configuration is compiled into the firmware by `build.rs`. This is still
a reproducible build-time configuration: the generated constants are visible
in Cargo's `OUT_DIR`, and changing the TOML file automatically rebuilds the
firmware.

### Logical actions

Each physical button has an action name. The generated firmware turns that
name into a protocol bit:

| Configuration action | Monado/OpenXR meaning |
| --- | --- |
| `trigger` | Index trigger click and value |
| `switch_hand` | Toggle active hand; not exposed as an OpenXR action |
| `a` | Index A click |
| `b` | Index B click |
| `grip` | Index squeeze/grip value and force |
| `system` | Index system click |
| `both_triggers` | Press both virtual Index triggers simultaneously |
| `joystick_button` | Index thumbstick click |
| `unmapped` | Ignore the physical input |

The default mapping is the requested one:

```toml
trigger1_action = "trigger"
trigger2_action = "switch_hand"
button_a_action = "a"
button_b_action = "b"
grip_action = "grip"
system1_action = "system"
system2_action = "both_triggers"
joystick_button_action = "joystick_button"
```

`System2` uses the special `both_triggers` action for VRChat FBT calibration.
Unlike normal actions, it bypasses the selected-hand switch only for the
trigger: both virtual controllers report their trigger click and value while
the button is held. Other inputs still go only to the selected hand.

## 4. Build and flash the Pico

Enter the development shell from the firmware directory. The directory's
Cargo configuration selects `thumbv6m-none-eabi`, enables `build-std = ["core"]`,
and supplies the RP2040 linker arguments:

```sh
cd firmware
nix develop -c cargo build --release
```

The ELF is at:

```text
target/thumbv6m-none-eabi/release/flow_motion_controller_firmware
```

Hold BOOTSEL while connecting the Pico, then run:

```sh
nix develop -c cargo run --release
```

The configured Cargo runner invokes `picotool`. A manual equivalent is:

```sh
sudo picotool load -u -v -x -t elf \
  target/thumbv6m-none-eabi/release/flow_motion_controller_firmware
```

Rebuild after changing either the TOML configuration or `memory.x`.

## 5. Understand the binary protocol

The Pico samples buttons and joystick channels every 1 ms. It sends a fixed-size
UDP datagram immediately when their logical state changes, then repeats that
state twice at 1 ms intervals with the same sequence number. A newer input state
supersedes any pending repeats, so a release cannot be followed by stale press
retries. While the state is unchanged, it sends a keepalive every 100 ms. This
reduces idle traffic by 90% while preserving held buttons, and one lost datagram
does not erase a short button event.

All integer values are little-endian:

| Bytes | Field | Meaning |
| --- | --- | --- |
| `0..4` | magic | ASCII `FMC1` |
| `4..8` | sequence | Wrapping frame counter |
| `8..10` | buttons | Bitmask of configured logical actions |
| `10..12` | reserved | Zero for forward compatibility |
| `12..14` | joystick X | Signed range `[-32768, 32767]` |
| `14..16` | joystick Y | Signed range `[-32768, 32767]` |

The frame definition and encoder/decoder are in `shared/protocol.rs`. The
same source file is included by the host Rust library and the `no_std` Pico
binary, so their framing cannot silently diverge.

The server uses the wrapping sequence number to reject duplicate retransmits
and out-of-order UDP datagrams and keeps only the newest frame. After 500 ms
without a frame it accepts a restarted sequence and makes all button and
joystick values zero. This keeps a disconnected or crashed Pico from leaving a
virtual button held forever. The 100 ms keepalive leaves several retry
opportunities before that timeout. The first valid sender owns the input
session until this timeout; datagrams from other source addresses are ignored
instead of being allowed to advance the controller's global sequence.

The listener is intentionally a local-LAN POC endpoint. It has no
authentication or encryption. Keep `FMC_LISTEN_ADDR` on a trusted VR LAN or
place it behind an appropriate firewall before treating this as a general
network service.

## Debugging the firmware connection without Monado

Stop Monado so its listener releases the port, then run the standalone debug
server:

```sh
FMC_DEBUG_LISTEN_ADDR=0.0.0.0:4242 \
nix develop -c cargo run --bin flow-motion-debug-server
```

`FMC_DEBUG_LISTEN_ADDR` defaults to `0.0.0.0:4242`. If it is omitted, the
server also accepts the normal `FMC_LISTEN_ADDR` setting. It prints the source
and contents of every decoded 16-byte UDP datagram. Action names include
`Trigger`, `BothTriggers`, `SwitchHand`, `A`, `B`, `Grip`, `System`, and
`JoystickButton`.

## 6. Understand hand switching

The Rust runtime starts with `active_right = true`. Every decoded frame has a
sequence number, so the runtime processes the switch button at most once per
frame even though Monado polls the left and right virtual devices separately.

The switch is edge-triggered:

```text
right active, switch released  -> right receives buttons
right active, switch pressed   -> active hand becomes left
left active, switch held       -> stays left
left active, switch released   -> stays left
left active, switch pressed    -> active hand becomes right
```

The first switch edge takes effect immediately. A 20 ms lockout after each
accepted press or release suppresses mechanical contact bounce without adding
latency to a normal hand switch.

The inactive controller still exists and still exposes the selected tracker's
pose. Its buttons are zero except while `both_triggers` is held. This allows
OpenXR applications to keep both controller devices and poses stable while the
single physical block changes which hand receives its actions.

## 7. How Monado is patched

The integration patch makes three small changes:

1. It adds a static imported Rust library and compiles the C `xrt_device`
   adapter when the Qwerty driver is enabled.
2. It links that adapter into the common target list.
3. It adds optional creation logic to the SteamVR builder.

The SteamVR builder already owns the devices discovered by SteamVR Lighthouse.
The added logic searches those devices by exact serial number:

```text
FMC_LEFT_TRACKER_SERIAL   -> left Flow Motion device
FMC_RIGHT_TRACKER_SERIAL  -> right Flow Motion device
```

The created device borrows the SteamVR tracker's `tracking_origin`, reads its
generic tracker pose, applies the configurable Rust pose offset, and assigns
the Index binding profiles. The tracker remains an ordinary Lighthouse device,
while the Flow Motion device is the application-facing controller role.

### Index squeeze compatibility

The Valve Index interaction profile has separate `squeeze/value` and
`squeeze/force` inputs. The physical Flow Motion grip is a button, so the
adapter publishes `1.0` to both while it is held and `0.0` to both when it is
released. WayVR's Index profile binds window grabbing to `squeeze/force`; the
force input is therefore required even though other applications may use only
the value input.

### Grip and aim are different poses

OpenXR does not define grip and aim as two names for one transform. The
[standard pose definitions](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html)
give them different jobs:

- **Grip** places an object held in the hand. Its position is near the palm
  centroid. Local `+X` is the palm normal (away from the left palm and into the
  right palm), local `-Z` points from little finger to thumb, and `+Y` follows
  from the right-hand rule.
- **Aim** is a pointing ray. Its conventional axes are local `+Y` up, `+X`
  right, and `-Z` forward; the exact ray origin and direction are selected by
  the runtime for the input source.

Flow Motion has one tracker per hand rather than separate physical grip and aim
origins, so both poses use the same position: the tracker position plus the
rotated tracker-local calibration offset and controller-center adjustment.
Only their final orientations differ:

| Output pose | Final orientation | Compatibility reason |
| --- | --- | --- |
| Aim | calibrated mount orientation, then the mirrored hand-model roll | Keeps local `-Z` along the fingers and orients UEVR hands correctly |
| Grip | aim orientation, then `77.5` degrees around local `+X` | Approximates the OpenXR grip frame while matching the observed VRChat/OpenVR Index hand model |

The aim mapping deliberately rolls the frame around its still-correct forward
axis to fit the virtual hand. Forcing the canonical aim `+Y` direction instead
would undo the palm correction that Satisfactory needs.

The composition order is:

```text
calibrated base = tracker orientation * saved mount rotation
aim orientation = calibrated base * mirrored hand-model roll
grip orientation = aim orientation * local-X rotation(77.5 degrees)
```

Index-style hand models expose the left palm along local `+X` and the right
palm along local `-X`. In the calibration pose those axes initially point
sideways. The mirrored hand-model roll (`+90` degrees around local `Z` for the
left hand and `-90` degrees for the right) maps both palms to world-down without
changing local `-Z` forward. It is applied to both output poses.

A strict grip-axis conversion would then rotate `90` degrees around local `X`,
keeping the palm fixed while moving grip `-Z` from forward to little-finger-to-
thumb. Hardware testing showed that the VRChat/OpenVR Index hand model slightly
over-rotates at that value, so Flow Motion backs it off by `12.5` degrees to
`77.5` degrees. This is an intentional model-compatibility bias, not part of
mount calibration.

This split exists because applications consume the poses differently. UEVR's
attached virtual hands in Satisfactory use grip position but aim rotation;
therefore changing grip orientation must not affect the working Satisfactory
hand. The VRChat/OpenVR hand model observed with this controller follows grip
orientation. Applying the palm correction only to grip left Satisfactory's
palms sideways, while making grip and aim identical left VRChat nearly
thumb-forward. The common roll plus grip-only local-X rotation fixes both paths
independently.

The Rust runtime is reference-counted. Both virtual devices share the same
socket server and hand-selection state. Each device owns one Rust handle, and
the final handle shuts down the server thread during Monado device teardown.

## 8. Run Monado

Build the patched service:

```sh
nix build .#monado
```

Use the exact tracker serials reported by the Lighthouse backend. A typical
launch is:

```sh
FMC_ENABLE=1 \
STEAMVR_LH_ENABLE=1 \
FMC_LEFT_TRACKER_SERIAL=LHR-LEFT_SERIAL \
FMC_RIGHT_TRACKER_SERIAL=LHR-RIGHT_SERIAL \
FMC_LISTEN_ADDR=0.0.0.0:4242 \
./result/bin/monado-service
```

The service must be able to see both trackers before the builder runs. If a
serial is empty or not found, Monado logs an error and does not create that
hand's virtual controller.

### Pose calibration

The Lighthouse pose describes the tracker housing, not the hand underneath it.
Flow Motion therefore applies one persistent tracker-local transform to each
hand. This is mount calibration: changing the SteamVR room setup does not
normally require recalibrating, but moving a tracker within its strap does.

First deploy the patched Monado package through the NixOS configuration so the
user service understands calibration mode. Hold the hand flat, palm down, with
the fingers pointing in the same horizontal direction as the headset. Looking
down at the hand is fine because pitch is ignored; do not turn your head left
or right relative to the fingers. Then run:

```sh
nix run .#calibrate -- left
nix run .#calibrate -- right
```

The launcher remembers whether `monado.service` was active, sets
`FMC_CALIBRATE_HAND` in the systemd user manager, and starts Monado. After
SteamVR discovers the devices, Monado prints a three-second countdown, samples
the selected tracker and headset orientations for one second, and exits. The
launcher then removes calibration mode and restores the service if it was
previously running. No OpenXR application, firmware connection, or calibration
socket is involved.

Tracker quaternion samples are sign-aligned and averaged. Headset samples are
used only to find its horizontal forward direction: headset pitch and roll are
discarded. The target mount orientation is the headset yaw followed by a
180-degree local roll around `Z`; the saved value is the rotation from the
averaged tracker orientation to that target. Because the target yaw comes from
the headset, the result does not depend on SteamVR's startup yaw. This saved
rotation describes only how the tracker is mounted on the hand. The mirrored
hand-model roll and grip-only `77.5`-degree adjustment described above are
fixed runtime corrections and are deliberately not written into the
calibration file. Results are stored by default in:

```text
~/.config/flow-motion-controller/calibration.conf
```

The generated file is intentionally simple and editable:

```text
LEFT_SERIAL=LHR-LEFT_SERIAL
LEFT_OFFSET_X=0.000000000
LEFT_OFFSET_Y=0.030000000
LEFT_OFFSET_Z=-0.080000000
LEFT_OFFSET_QX=0.000000000
LEFT_OFFSET_QY=0.000000000
LEFT_OFFSET_QZ=0.000000000
LEFT_OFFSET_QW=1.000000000
```

Both hands are written to the same file. Monado verifies the stored serial
against `FMC_LEFT_TRACKER_SERIAL` or `FMC_RIGHT_TRACKER_SERIAL`; a mismatched
entry is ignored instead of being applied to another tracker. Override the
location with `FMC_CALIBRATION_FILE` when the service should use another path.

The tracker pose alone cannot reveal the physical location of the center of the
hand. Calibration therefore updates rotation while preserving the XYZ values.
Measure or tune those tracker-local position offsets in metres. Per-hand
environment variables override the saved values when temporary adjustment is
useful:

```sh
FMC_LEFT_OFFSET_X=0.0 FMC_LEFT_OFFSET_Y=0.03 FMC_LEFT_OFFSET_Z=-0.08 \
FMC_RIGHT_OFFSET_X=0.0 FMC_RIGHT_OFFSET_Y=-0.03 FMC_RIGHT_OFFSET_Z=-0.08
```

Independently of those tracker-local offsets, the virtual controller center is
moved 3 cm into the hand along the calibrated palm-down direction. Set
`FMC_CONTROLLER_CENTER_DOWN` to another distance in metres, or to `0` to
disable this adjustment.

`FMC_CALIBRATION_SECONDS` changes the sampling duration from its one-second
default; accepted values are clamped to 0.1 through 10 seconds.
`FMC_CALIBRATION_COUNTDOWN_SECONDS` changes the three-second preparation delay
and is clamped to 0 through 10 seconds. The older shared `FMC_OFFSET_*`
variables remain defaults for both hands.

At runtime Rust rotates the local position offset by the current tracker
orientation, adds it to the tracked position, and applies the saved mount
rotation. It then applies the common mirrored hand-model roll to both poses and
the local-X compatibility rotation to grip only. The headset is read only while
calibrating; ordinary Monado starts apply the saved local rotation, so a new
startup origin does not require another calibration. Changing the fixed grip
compatibility angle also does not require recalibration. Rust additionally adds
the angular-velocity tangential term to linear velocity.

Once the service is running, point an OpenXR application at the generated
runtime manifest, for example:

```sh
XR_RUNTIME_JSON="$PWD/result/share/openxr/1/openxr_monado.json" hello_xr -G Vulkan
```

The exact application is not important for the POC; it should report Index
controllers on both configured hands. Move a tracker to verify pose tracking,
press A/B/trigger/grip/system to verify button routing, and press the switch
button to verify the active-hand behavior.

## 9. Nix workflow

The flake exposes:

| Command | Result |
| --- | --- |
| `nix develop` | Rust, Cargo, CMake, SDL2, GCC, and Pico tooling |
| `cargo test` | Focused host tests for calibration, poses, UDP state, and button routing |
| `nix build .#monado` | Patched Monado service and runtime files |
| `FMC_FIRMWARE_CONFIG="$PWD/firmware/firmware-config.toml" nix build --impure .#firmware -o firmware-result` | Release Pico ELF using the ignored local configuration |
| `FMC_FIRMWARE_CONFIG="$PWD/firmware/firmware-config.toml" nix flake check --impure` | Host Rust, Pico firmware, and Monado build checks |

The Monado derivation first builds `src/lib.rs` as a static library, then
copies that library and the checked-in C sources into the Monado source tree
before CMake configures it. This keeps the Rust code independent of Monado's
source checkout while still producing one runnable Monado package.

The Pico package uses the nightly toolchain from the Rust overlay because the
RP2040 build uses `build-std = ["core"]`. The development shell and package
both use the same target and the same Cargo lockfile.

## 10. Troubleshooting

### Monado starts but no Flow Motion controller appears

Check all of these:

- `FMC_ENABLE=1` is present.
- Both serial environment variables exactly match the discovered tracker
  serials.
- The package is `./result/bin/monado-service`, not an older system binary.
- The service was built with `nix build .#monado` after changing the patch.
- Lighthouse discovery completed before the service's device builder ran.

### The socket server fails to start

Look for the Rust log line showing the listener address. Another process may
already use the port. Change `FMC_LISTEN_ADDR` or stop the old service. The
Pico's `monado_host` and `monado_port` must point to the same address and port.

### The Pico joins Wi-Fi but buttons do nothing

Check the physical assumptions first: button inputs are active-low, the GPIO
numbers are the actual RP2040 GPIO numbers, and the joystick uses ADC-capable
pins. Then verify that the Monado host is reachable from the Pico's network.
The runtime zeros input after 500 ms without a datagram, so a stopped firmware
or blocked UDP path looks like released buttons by design.

### The wrong hand receives input

The initial mode is always right. Release the switch button, press it once,
and wait for one frame before testing the other hand. A held switch does not
toggle repeatedly; it must be released and pressed again.

### The pose is offset or rotated incorrectly

Start with the identity quaternion and zero position. Adjust one position
component at a time, then add orientation. The offset is expressed in the
tracker's local coordinate system, not world coordinates.

Use the affected application to identify the layer before changing anything:

- If both Satisfactory/UEVR and VRChat are wrong in the same way, check the
  tracker mount, serial selection, and saved calibration.
- If the palm direction is wrong in Satisfactory/UEVR, inspect the common
  mirrored hand-model roll and aim path. Do not tune the grip-only angle.
- If Satisfactory remains correct but VRChat is rotated around the palm normal,
  inspect the grip-only compatibility angle. Do not recalibrate or alter the
  shared mount rotation to compensate.
- After rebuilding, confirm the running service actually uses the new Nix store
  path. A successful `nix build .#monado` proves the package, not deployment.

## 11. Scope and next steps

This is intentionally a production-usable POC rather than a generalized
controller framework. The host library has focused unit tests for calibration,
pose composition, UDP ordering and source ownership, dual-trigger routing, and
switch debounce, but hardware tracking and application pose consumption still
require testing in the headset. It does not include WiVRn patches, a GUI
configuration editor, network authentication, or automatic tracker discovery.
Those are separate projects and should be added only when the real hardware
reveals a need for them.
