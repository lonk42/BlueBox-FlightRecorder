# BlueBox Flight Recorder

ESP32-based flight data recorder for high-speed projectiles.

## Hardware Requirements

- ESP32 Development Board
- BMP280 Barometric Pressure Sensor (I2C)
- MPU6500 Gyroscope/Accelerometer (SPI)
- NEO-6m GPS Module (UART)
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

### NEO-6m GPS (UART)
- TX  → GPIO 16 (ESP32 RX2)
- RX  → GPIO 17 (ESP32 TX2)
- VCC → 3.3V or 5V (depending on module)
- GND → GND

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
- **Audio**:
  - Single beep every 3s = Waiting for GPS fix
  - Double beep every 3s = GPS locked, ready for flight!
- **Behavior**: Records to 86KB RAM buffer (~1 second capacity), no flash wear
- **GPS Acquisition**: Module acquires satellites during this phase (cold start: 26-30 seconds)
- **Transition**: Automatically enters Flight Mode when launch detected (gyro >150 deg/s for 250ms)

### 2. Flight Mode (In-Flight)
- **Audio**: Continuous 1500Hz tone
- **Behavior**:
  - Pre-launch RAM buffer (~1 second) flushed to flash
  - Continuous recording to flash (~26 seconds total capacity)
  - GPS data captured at 5Hz if fix acquired
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
2. **Flash Erase**: ~6 seconds (one-time initialization of 2MB partition)
3. **Sensors Initialized**: Two beeps → enters Launch Mode
4. **GPS Acquisition**: Module begins acquiring satellites
5. **Ready Indication**:
   - Single beeps = Waiting for GPS
   - Double beeps = GPS locked, ready for flight
6. System automatically manages all mode transitions

## System Architecture

- **Core 0**: High-speed sensor polling (MPU6500: 1kHz, BMP280: 100Hz, GPS: 5Hz)
- **Core 1**: State management, beep task, WiFi, HTTP server, GPS task
- **Storage Strategy**:
  - **Launch Mode**: 86KB RAM buffer (~1 second pre-launch data)
  - **Flight Mode**: 2MB flash partition (~26 seconds total capacity)
  - **Sample Size**: 86 bytes (timestamp + IMU + BMP280 + GPS)
  - **Flash Partition Layout**:
    - NVS: 16KB (WiFi/config storage)
    - OTA: 8KB (firmware updates)
    - PHY Init: 4KB (RF calibration)
    - Factory App: 1.5MB (application firmware)
    - Flight Data: 2MB (sensor recordings)
- **Sampling Rates**:
  - IMU (gyro/accel): ~1000Hz
  - Pressure/temperature: ~100Hz
  - GPS (position/velocity): 5Hz
- **State Machine**: Automatic transitions (Launch → Flight → Recovery) based on gyroscope data
- **Launch Detection**: Gyro magnitude >150 deg/s for 250ms
- **Landing Detection**: Gyro magnitude <10 deg/s for 1 second
- **GPS Configuration**: 5Hz update rate, airborne <1g dynamic model
  - Cold start acquisition: ~26-30 seconds
  - Data: lat/lon (7 decimals), altitude, speed, heading, satellites

## Audio Feedback

- **Single beep**: Power on
- **Two beeps**: Sensors ready, entering Launch Mode
- **Single beep every 3s**: Launch Mode - waiting for GPS fix
- **Double beep every 3s**: Launch Mode - GPS fix acquired (ready for flight!)
- **Continuous tone (1500Hz)**: Flight Mode active
- **Triple ascending tone every 6s**: Recovery Mode active (WiFi ready)
- **SOS pattern**: Error state
