{
  description = "Rust Flow Motion Controller firmware and Monado package";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      rust-overlay,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ rust-overlay.overlays.default ];
          config.allowUnfree = true;
        };
        nightlyRust = pkgs.rust-bin.selectLatestNightlyWith (
          toolchain:
          toolchain.default.override {
            extensions = [
              "rust-src"
              "rustfmt"
              "rust-analyzer"
            ];
          }
        );
        firmwareConfigPath = builtins.getEnv "FMC_FIRMWARE_CONFIG";
        firmwareConfig =
          if firmwareConfigPath == "" then
            throw "Set FMC_FIRMWARE_CONFIG to firmware/firmware-config.toml and build the firmware with --impure"
          else
            builtins.path {
              path = firmwareConfigPath;
              name = "firmware-config.toml";
            };
        rustLibrary = pkgs.rustPlatform.buildRustPackage {
          pname = "flow-motion-controller-rust";
          version = "0.1.0";
          src = pkgs.lib.cleanSource ./.;
          cargoLock.lockFile = ./Cargo.lock;
          cargoBuildFlags = [
            "-p"
            "flow_motion_controller"
          ];
          doCheck = false;
          installPhase = ''
            mkdir -p $out/lib
            cp target/${pkgs.stdenv.hostPlatform.rust.rustcTarget}/release/libflow_motion_controller.a $out/lib/
          '';
        };
        firmware = pkgs.rustPlatform.buildRustPackage {
          pname = "flow-motion-controller-firmware";
          version = "0.1.0";
          src = pkgs.lib.cleanSource ./.;
          cargoRoot = "firmware";
          cargoLock.lockFile = ./Cargo.lock;
          nativeBuildInputs = [ nightlyRust ];
          allowSubstitutes = false;
          preferLocalBuild = true;
          doCheck = false;
          postPatch = ''
            cp Cargo.lock firmware/Cargo.lock
            cp ${firmwareConfig} firmware/firmware-config.toml
          '';
          buildPhase = ''
            mkdir -p .cargo-vendor
            cp -r $cargoDeps/. .cargo-vendor/
            for crate in ${nightlyRust}/lib/rustlib/src/rust/library/vendor/*; do
              crate_name="''${crate##*/}"
              if [ ! -e ".cargo-vendor/$crate_name" ]; then
                cp -r "$crate" .cargo-vendor/
              fi
            done
            cat > firmware/.cargo/config.toml <<EOF
            [build]
            target = "thumbv6m-none-eabi"
            [unstable]
            build-std = ["core"]
            [source.crates-io]
            replace-with = "vendored-sources"
            [source.vendored-sources]
            directory = "../.cargo-vendor"
            [target.'cfg(all(target_arch = "arm", target_os = "none"))']
            rustflags = ["-C", "link-arg=-Tlink.x", "-C", "link-arg=-Tlink-rp.x"]
            EOF
            cd firmware
            cargo build --release
          '';
          installPhase = ''
            mkdir -p $out
            cp $NIX_BUILD_TOP/source/target/thumbv6m-none-eabi/release/flow_motion_controller_firmware $out/
          '';
        };
        monado = pkgs.monado.overrideAttrs (old: {
          pname = "flow-motion-controller-monado";
          version = "25.1.0-unstable-2026-08-08";
          src = pkgs.fetchgit {
            url = "https://gitlab.freedesktop.org/monado/monado.git";
            rev = "735e29e4e7552b254528dbb20e0e96ec8f32368c";
            hash = "sha256-l4WM5SrGOrbk//6RDC0+KPoa4fVdMvvOuFJ1DI08WOA=";
          };
          patches = [ ./monado/integration.patch ];
          cmakeFlags = (old.cmakeFlags or [ ]) ++ [
            (pkgs.lib.cmakeBool "XRT_BUILD_DRIVER_QWERTY" true)
          ];
          postPatch = (old.postPatch or "") + ''
            mkdir -p src/xrt/drivers/flow_motion_controller flow_motion_controller_target/release
            cp ${./monado/flow_motion_controller.c} src/xrt/drivers/flow_motion_controller/flow_motion_controller.c
            cp ${./monado/flow_motion_controller.h} src/xrt/drivers/flow_motion_controller/flow_motion_controller.h
            cp ${./monado/flow_motion_controller_bindings.h} src/xrt/drivers/flow_motion_controller/flow_motion_controller_bindings.h
            cp ${rustLibrary}/lib/libflow_motion_controller.a flow_motion_controller_target/release/
          '';
        });
        calibrate = pkgs.writeShellApplication {
          name = "flow-motion-calibrate";
          runtimeInputs = [
            pkgs.coreutils
            pkgs.systemd
          ];
          text = ''
            hand="''${1:-}"
            case "$hand" in
              left|right) ;;
              *)
                echo "Usage: flow-motion-calibrate left|right" >&2
                exit 2
                ;;
            esac

            was_active=0
            if systemctl --user is-active --quiet monado.service; then
              was_active=1
            fi
            : "''${XDG_RUNTIME_DIR:?XDG_RUNTIME_DIR is not set}"
            status_file="$XDG_RUNTIME_DIR/flow-motion-calibration-$$.status"
            journal_pid=""

            restore() {
              if [ -n "$journal_pid" ]; then
                kill "$journal_pid" 2>/dev/null || true
                wait "$journal_pid" 2>/dev/null || true
              fi
              systemctl --user stop monado.service || true
              systemctl --user unset-environment FMC_CALIBRATE_HAND FMC_CALIBRATION_STATUS_FILE || true
              systemctl --user reset-failed monado.service || true
              rm -f "$status_file"
              if [ "$was_active" -eq 1 ]; then
                systemctl --user start monado.service || true
              fi
            }
            trap restore EXIT

            echo "Hold the $hand hand flat, palm down, with the headset and fingers facing the same horizontal direction."
            echo "Looking down is fine; do not turn your head left or right relative to the fingers."
            echo "Monado discovery, countdown, and sampling logs follow; timeout is 45 seconds."
            systemctl --user set-environment \
              FMC_CALIBRATE_HAND="$hand" \
              FMC_CALIBRATION_STATUS_FILE="$status_file"
            systemctl --user restart monado.service
            journalctl --user --unit monado.service --follow --since now --output cat &
            journal_pid="$!"
            deadline=$((SECONDS + 45))
            while true; do
              state="$(systemctl --user show monado.service --property=ActiveState --value)"
              case "$state" in
                active|activating|deactivating|reloading) sleep 0.1 ;;
                *) break ;;
              esac
              if [ "$SECONDS" -ge "$deadline" ]; then
                echo "Calibration timed out; is the patched Monado package deployed with FMC_ENABLE=1?" >&2
                exit 1
              fi
            done

            status=""
            if [ -f "$status_file" ]; then
              status="$(<"$status_file")"
            fi
            result="$(systemctl --user show monado.service --property=Result --value)"
            if [ "$status" != success ] || [ "$result" != success ]; then
              echo "Calibration failed; inspect: journalctl --user -u monado.service -n 100" >&2
              exit 1
            fi
            echo "Calibration saved successfully."
          '';
        };
      in
      {
        packages = {
          inherit monado;
          inherit firmware;
          default = monado;
        };

        checks = {
          rust = rustLibrary;
          inherit firmware;
          inherit monado;
        };

        apps.monado-service = flake-utils.lib.mkApp {
          drv = monado;
          exePath = "/bin/monado-service";
        };

        apps.calibrate = flake-utils.lib.mkApp {
          drv = calibrate;
          exePath = "/bin/flow-motion-calibrate";
        };

        devShells.default = pkgs.mkShell {
          packages = [
            nightlyRust
            pkgs.cargo
            pkgs.cmake
            pkgs.gcc
            pkgs.pkg-config
            pkgs.picotool
            pkgs.rust-cbindgen
            pkgs.SDL2
          ];
        };
      }
    );
}
