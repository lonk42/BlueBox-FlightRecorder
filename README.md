# BlueBox Flight Recorder

ESP32-based flight data recorder for high-speed projectiles.

## Hardware Requirements

- ESP32 Development Board
- BMP280 Barometric Pressure Sensor (I2C)
- MPU6500 Gyroscope/Accelerometer (SPI)
- PC POST Speaker
- Li-Po Battery

## Pinout

### BMP280 (I2C)
- SDA → GPIO 21
- SCL → GPIO 22
- VCC → 3.3V
- GND → GND

### MPU6500 (SPI)
- MOSI → GPIO 23
- MISO → GPIO 19
- SCK  → GPIO 18
- CS   → GPIO 5
- VCC  → 3.3V
- GND  → GND

### Speaker
- Speaker+ → GPIO 25
- Speaker- → GND

## Building and Flashing

### Prerequisites (Arch Linux)
```bash
sudo pacman -S git wget flex bison gperf python python-pip cmake ninja ccache dfu-util libusb
```

### Setup ESP-IDF
```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout release/v5.1
git submodule update --init --recursive
./install.sh esp32
```

### Build and Flash
```bash
# Set up environment (run in each terminal session)
. ~/esp/esp-idf/export.sh

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or do all at once
idf.py -p /dev/ttyUSB0 flash monitor
```

## Operation

1. **Power On**: Single beep
2. **Sensors Initialized**: Two beeps
3. **Recording**: Device records for 30 seconds max
4. **Recording Complete**: Three beeps
5. **WiFi AP Ready**: Continuous tone

### WiFi Access
- SSID: `BlueBox`
- Password: `bluebox123`
- Data endpoint: `http://192.168.4.1/data`

## System Architecture

- **Core 0**: High-speed sensor polling (MPU6500: 1kHz, BMP280: 100Hz)
- **Core 1**: State management, WiFi, HTTP server
- **Storage**: 2MB circular buffer in RAM
- **Sampling Rate**: ~1000Hz for gyro/accel, ~100Hz for pressure/temperature
