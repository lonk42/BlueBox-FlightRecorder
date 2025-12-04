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

BlueBox automatically transitions through three modes based on detected motion:

### 1. Launch Mode (Pre-Flight)
- **Start**: After sensor initialization (2 beeps)
- **Audio**: Two-tone beep (low→high) every 3 seconds
- **Behavior**: Records continuously, maintains 1-second data window
- **Transition**: Automatically enters Flight Mode when launch detected (gyro >150 deg/s for 250ms)

### 2. Flight Mode (In-Flight)
- **Audio**: Continuous 1500Hz tone
- **Behavior**: Full continuous recording (~1.3 seconds buffer capacity)
- **Transition**: Automatically enters Recovery Mode when landing detected (gyro <10 deg/s for 1 second)

### 3. Recovery Mode (Post-Flight)
- **Audio**: Triple ascending tone every 6 seconds
- **Behavior**: WiFi AP activates, data available for download
- **WiFi Access**:
  - SSID: `BlueBox`
  - Password: `bluebox123`
  - Data endpoint: `http://192.168.4.1/data`

### Startup Sequence
1. **Power On**: Single beep
2. **Sensors Initialized**: Two beeps → enters Launch Mode
3. System automatically manages all mode transitions

## System Architecture

- **Core 0**: High-speed sensor polling (MPU6500: 1kHz, BMP280: 100Hz)
- **Core 1**: State management, beep task, WiFi, HTTP server
- **Storage**: 64KB circular buffer in RAM (~1.3 seconds at 1kHz)
- **Sampling Rate**: ~1000Hz for gyro/accel, ~100Hz for pressure/temperature
- **State Machine**: Automatic transitions (Launch → Flight → Recovery) based on gyroscope data
- **Launch Detection**: Gyro magnitude >150 deg/s for 250ms
- **Landing Detection**: Gyro magnitude <10 deg/s for 1 second

## Audio Feedback

- **Single beep**: Power on
- **Two beeps**: Sensors ready, entering Launch Mode
- **Two-tone pair every 3s**: Launch Mode active
- **Continuous tone (1500Hz)**: Flight Mode active
- **Triple ascending tone every 6s**: Recovery Mode active (WiFi ready)
- **SOS pattern**: Error state
