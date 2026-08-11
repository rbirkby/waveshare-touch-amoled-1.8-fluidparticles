# Top-level build orchestration for the esp32-s3-amoled repo.
#
# Two toolchains live here: ESP-IDF (idf.py) for the Waveshare ESP32-S3 board
# and the Pico SDK (cmake + ninja) for the RP2350 board.  Each sub-project
# builds out-of-tree into its own build/ directory; this file delegates to the
# appropriate toolchain without touching the sub-project CMakeLists.txt files.
#
# Prerequisites
#   ESP32-S3 targets : IDF_PATH set and the ESP-IDF virtual-env active
#   RP2350 targets   : PICO_SDK_PATH set, cmake ≥ 3.13, ninja

.DEFAULT_GOAL := help

# ── sub-project roots ────────────────────────────────────────────────────────
ESP32S3    := fluid3d/platform/esp32s3
RP2350     := fluid3d/platform/rp2350
PROBE_S3   := tools/hwprobe
PROBE_RP   := tools/hwprobe-rp2350
PROBE_DISP := tools/hwprobe-rp2350-display

# ── tunables ─────────────────────────────────────────────────────────────────
PICO_BOARD ?= pico2

# ── toolchain guards ─────────────────────────────────────────────────────────
.PHONY: _check-idf _check-pico

_check-idf:
	@command -v idf.py >/dev/null 2>&1 || \
		{ echo "error: idf.py not found — activate ESP-IDF first:"; \
		  echo "  . \$$IDF_PATH/export.sh"; exit 1; }

_check-pico:
	@test -n "$(PICO_SDK_PATH)" || \
		{ echo "error: PICO_SDK_PATH not set — set it to your Pico SDK checkout"; exit 1; }
	@command -v cmake >/dev/null 2>&1 || { echo "error: cmake not found"; exit 1; }
	@command -v ninja >/dev/null 2>&1 || { echo "error: ninja not found — brew install ninja"; exit 1; }

# ── phony targets ────────────────────────────────────────────────────────────
.PHONY: all help clean \
        fluid-esp32s3 flash-esp32s3 monitor-esp32s3 flash-monitor-esp32s3 \
        fluid-rp2350 flash-rp2350 \
        hwprobe-esp32s3 hwprobe-rp2350 hwprobe-rp2350-display

# ── primary targets ──────────────────────────────────────────────────────────
all: fluid-esp32s3 fluid-rp2350

fluid-esp32s3: _check-idf
	cd $(ESP32S3) && idf.py build

flash-esp32s3: _check-idf
	cd $(ESP32S3) && idf.py flash

monitor-esp32s3: _check-idf
	cd $(ESP32S3) && idf.py monitor

flash-monitor-esp32s3: _check-idf
	cd $(ESP32S3) && idf.py flash monitor

fluid-rp2350: _check-pico
	cmake -S $(RP2350) -B $(RP2350)/build -DPICO_BOARD=$(PICO_BOARD) -GNinja -Wno-dev
	cmake --build $(RP2350)/build

flash-rp2350: fluid-rp2350
	picotool load -f -x $(RP2350)/build/fluid3d.uf2

# ── probe / diagnostic targets ───────────────────────────────────────────────
hwprobe-esp32s3: _check-idf
	cd $(PROBE_S3) && idf.py build

hwprobe-rp2350: _check-pico
	cmake -S $(PROBE_RP) -B $(PROBE_RP)/build -DPICO_BOARD=$(PICO_BOARD) -GNinja -Wno-dev
	cmake --build $(PROBE_RP)/build

hwprobe-rp2350-display: _check-pico
	cmake -S $(PROBE_DISP) -B $(PROBE_DISP)/build -DPICO_BOARD=$(PICO_BOARD) -GNinja -Wno-dev
	cmake --build $(PROBE_DISP)/build

# ── clean ────────────────────────────────────────────────────────────────────
clean:
	rm -rf \
		$(ESP32S3)/build \
		$(RP2350)/build \
		$(PROBE_S3)/build \
		$(PROBE_RP)/build \
		$(PROBE_DISP)/build

# ── help ─────────────────────────────────────────────────────────────────────
help:
	@printf "\nUsage: make [PICO_BOARD=pico2] <target>\n\n"
	@printf "  %-30s %s\n" "all" "Build fluid-esp32s3 and fluid-rp2350"
	@printf "  %-30s %s\n" "fluid-esp32s3" "Build fluid3d for Waveshare ESP32-S3"
	@printf "  %-30s %s\n" "flash-esp32s3" "Flash ESP32-S3 (idf.py flash)"
	@printf "  %-30s %s\n" "monitor-esp32s3" "Open serial monitor for ESP32-S3"
	@printf "  %-30s %s\n" "flash-monitor-esp32s3" "Flash then open serial monitor"
	@printf "  %-30s %s\n" "fluid-rp2350" "Build fluid3d for RP2350 (PICO_BOARD=$(PICO_BOARD))"
	@printf "  %-30s %s\n" "flash-rp2350" "Build and flash RP2350 (BOOTSEL mode or picotool)"
	@printf "  %-30s %s\n" "hwprobe-esp32s3" "Build hardware probe for ESP32-S3"
	@printf "  %-30s %s\n" "hwprobe-rp2350" "Build button investigation probe"
	@printf "  %-30s %s\n" "hwprobe-rp2350-display" "Build AMOLED display verification probe"
	@printf "  %-30s %s\n" "clean" "Remove all build/ directories"
	@printf "\n"
