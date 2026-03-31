ESP32_PACKAGE_URL := https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
ARDUINO_FQBN ?= esp32:esp32:esp32
HOST_BUILD_DIR := build/host
FIRMWARE_BUILD_DIR := build/firmware
SKETCH_DIR := firmware_monocopter

.PHONY: local-ci host-tests firmware arduino-deps check-tools

local-ci: check-tools host-tests firmware

check-tools:
	cmake --version >/dev/null
	arduino-cli version >/dev/null

host-tests:
	cmake -S . -B $(HOST_BUILD_DIR)
	cmake --build $(HOST_BUILD_DIR)
	ctest --test-dir $(HOST_BUILD_DIR) --output-on-failure

arduino-deps:
	arduino-cli core update-index --additional-urls $(ESP32_PACKAGE_URL)
	arduino-cli core install esp32:esp32 --additional-urls $(ESP32_PACKAGE_URL)
	arduino-cli lib install "ESP32Servo"
	arduino-cli lib install "Adafruit BNO055"
	arduino-cli lib install "Adafruit Unified Sensor"
	arduino-cli lib install "Adafruit BusIO"

firmware: arduino-deps
	arduino-cli compile --fqbn $(ARDUINO_FQBN) --warnings none --output-dir $(FIRMWARE_BUILD_DIR) $(SKETCH_DIR)
