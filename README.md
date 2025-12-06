# BlueBox Flight Recorder

A flight data recording and analysis system for projectiles, designed for ESP32 with a partnering web application for data visualization.

## Repository Structure

```
bluebox-flight-recorder/
├── firmware/              # ESP32 firmware (C)
│   ├── main/             # Application logic
│   ├── components/       # Sensor drivers
│   └── README.md         # Build & flash instructions
├── webapp/               # Web application (Python)
│   ├── backend/          # FastAPI REST API
│   ├── frontend/         # Dash visualization dashboard
│   ├── k8s/              # Kubernetes manifests
│   ├── docker-compose.yml
│   └── README.md         # Setup & deployment instructions
├── docs/                 # Additional documentation
└── README.md            # This file
```

### ESP32 Setup

Build and flash the ESP32 firmware with esp-idf:

```bash
cd firmware
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

See [firmware/README.md](firmware/README.md) for detailed instructions.

### 2. Web Application

Install with Docker Compose:

```bash
cd webapp
docker-compose up --build
```

Install with Helm:

```bash
cd helm

# Install with values
helm show values ./bluebox/ > values.yaml
helm upgrade -i bluebox ./bluebox -n bluebox --create-namespace --values values.yaml
```

Access the dashboard at: http://localhost:8050

See [webapp/README.md](webapp/README.md) for detailed instructions.

## Features

### Firmware Features
- **High-speed sensor sampling:** 1kHz MPU6500 (gyro/accel), 100Hz BMP280 (pressure/temp), 5Hz GPS
- **Automatic flight detection:** Launch and landing detection via gyroscope
- **Flash storage:** ~23 seconds of flight data stored in 2MB flash partition
- **WiFi data upload:** POST flight data to web API after landing
- **Audio feedback:** PC speaker status beeps for operational feedback
- **Three-mode operation:** Launch → Flight → Recovery

### Web Application Features
- **RESTful API:** FastAPI backend for flight data collection
- **PostgreSQL storage:** Robust database with JSONB for JSON data
- **Interactive graphs:** Plotly-powered time-series visualization
- **GPS trajectory mapping:** Flight path visualization on interactive maps
- **Flight management:** Browse, view, and delete flight recordings
- **Real-time dashboard:** Auto-refreshing flight list
- **Production-ready:** Kubernetes manifests with health checks and scaling

## Hardware

**Development Board:** ESP32-WROOM-32

**Sensors:**
- MPU6500 (6-axis gyro/accel, SPI)
- BMP280 (pressure/temperature, I2C)
- NEO-6m (GPS, UART)
- PC POST speaker (status audio)

See [firmware/README.md](firmware/README.md) for pinout and wiring diagrams.
