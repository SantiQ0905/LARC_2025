# LARC25 - High-Speed Line Following Robot 🤖

## Overview

LARC25 is a high-performance line following robot firmware for the ESP32-C6-DevKitM-1 microcontroller, designed for competitive robotics applications. The system implements a sophisticated PID controller with dynamic parameter adjustment, achieving high-speed line tracking with robust corner handling and terminal stop detection.

## Technical Specifications

### Hardware Platform
- **MCU**: ESP32-C6-DevKitM-1 (RISC-V single-core)
- **Framework**: ESP-IDF 5.5.0
- **Build System**: PlatformIO + CMake
- **Operating System**: FreeRTOS

### Peripheral Configuration
- **ADC**: 12-bit resolution, ADC1_CH1 with 12dB attenuation
- **PWM**: 20 kHz carrier frequency, 10-bit resolution (1024 levels)
- **UART**: 115200 baud, 8N1 configuration for real-time telemetry
- **GPIO**: Multiplexed sensor array, dual H-bridge motor control

## System Architecture

### Sensor Array
8-channel IR reflectance sensor array with analog multiplexing:
- **Channels**: S0-S7 (3-bit MUX addressing via GPIO 0, 4, 5)
- **Sampling**: 3-sample averaging per channel with 5μs settling time
- **Threshold**: 1650 (12-bit ADC counts)
- **Weighted Positions**: `[-5, -3, -1, 0, 0, 1, 3, 5]` with S0 2× aggressive weighting

### Motor Control
Dual DC motor H-bridge configuration:
- **PWM Channels**: LEDC_CHANNEL_0 (Motor A), LEDC_CHANNEL_1 (Motor B)
- **Direction Pins**: GPIO18/19 (Motor A), GPIO20/21 (Motor B)
- **Drive Frequency**: 20 kHz (ultrasonic, reduces audible noise)
- **Maximum Duty**: 1023 (100% - 0.1% = ~99.9%)

### Control Algorithm

#### Dynamic PID Controller
Base parameters optimized for high-speed operation:
```c
Base Speed:  1150 (of 1023 max)
Kp_base:     135.0
Ki_base:     3.5
Kd_base:     70.0
Δt:          20 ms (50 Hz control loop)
```

**Adaptive Gain Scheduling**:
- **Error > 2.0**: Kp × 1.25, Kd × 1.45
- **Error > 4.0**: Kp × 1.5,  Kd × 1.7

**Low-Sensor Derating**: When ≤2 sensors active, motor output scaled to 92% to prevent overshoot during high-speed sections.

#### Special Maneuvers
- **Sharp Turn Detection**: Single S0 activation triggers 230ms fixed-duration spin turn at 90% duty
- **Final Stop**: ≥7 sensors black for 3 consecutive cycles → 800ms creep forward at 600 PWM → full stop

### State Machine
- **Toggle Control**: GPIO14 button with 40ms debounce
- **Telemetry Output**: CSV format at 10 Hz (tick, error, L_motor, R_motor, black_count)

## Build Instructions

### Prerequisites
```bash
# Install PlatformIO Core
pip install platformio

# Or use PlatformIO IDE (VSCode extension)
```

### Compilation
```bash
# Build firmware
pio run -e esp32-c6-devkitm-1

# Build + Upload
pio run -t upload -e esp32-c6-devkitm-1

# Serial Monitor
pio device monitor
```

### ESP-IDF Native Build (Alternative)
```bash
idf.py set-target esp32c6
idf.py menuconfig  # Optional: configure sdkconfig
idf.py build
idf.py -p COM_PORT flash monitor
```

## Pin Assignments

| Function         | GPIO | Peripheral       | Notes                    |
|------------------|------|------------------|--------------------------|
| MUX_S0           | 4    | Digital Out      | LSB of 3-bit MUX address |
| MUX_S1           | 5    | Digital Out      | Mid bit                  |
| MUX_S2           | 0    | Digital Out      | MSB                      |
| MUX_Y            | 1    | ADC1_CH1         | Multiplexed analog input |
| Motor A PWM      | 18   | LEDC_CH0         | H-bridge enable A        |
| Motor A Dir      | 19   | Digital Out      | Direction control A      |
| Motor B PWM      | 20   | LEDC_CH1         | H-bridge enable B        |
| Motor B Dir      | 21   | Digital Out      | Direction control B      |
| Start Button     | 14   | Digital In (PU)  | Toggle start/stop        |
| DIP Switch 1     | 6    | Digital In (PU)  | Reserved                 |
| DIP Switch 2     | 7    | Digital In (PU)  | Reserved                 |
| UART TX          | 17   | UART1_TX         | Telemetry output         |
| UART RX          | 16   | UART1_RX         | Reserved                 |

## Telemetry Protocol

Real-time CSV stream via UART1 (115200 baud):
```
<tick>,<error>,<left_pwm>,<right_pwm>,<black_count>
```
**Example**:
```
50,-1.25,980,1150,3
55,0.75,1180,1120,2
```

Fields:
- `tick`: Control loop iteration counter
- `error`: Computed line position error (normalized)
- `left_pwm`: Left motor PWM duty (signed, -1023 to +1023)
- `right_pwm`: Right motor PWM duty (signed)
- `black_count`: Number of sensors detecting the line

## Performance Characteristics

- **Control Loop Frequency**: 50 Hz (20ms period)
- **Sensor Scan Time**: ~1.2ms (8 channels × 3 samples × 50μs)
- **Worst-Case Latency**: <2ms (sensor read → motor update)
- **Base Linear Speed**: ~1.12 m/s (calibrated for 6" wheels @ 1150 PWM)
- **Turn Response Time**: <50ms (from error detection to correction application)

## Tuning Parameters

Key constants for track-specific optimization (see [src/main.c](src/main.c)):

| Parameter           | Default | Description                              |
|---------------------|---------|------------------------------------------|
| `LINE_THRESHOLD`    | 1650    | Black/white discrimination (ADC counts)  |
| `BASE_SPEED`        | 1150    | Straight-line cruising speed             |
| `KP_BASE`           | 135.0   | Proportional gain                        |
| `KI_BASE`           | 3.5     | Integral gain (anti-windup implicit)     |
| `KD_BASE`           | 70.0    | Derivative gain                          |
| `TURN_45_MS`        | 230     | Duration of sharp turn maneuver          |
| `STOP_BLACK_MIN`    | 7       | Sensor threshold for final stop          |
| `STOP_CONFIRM_COUNT`| 3       | Consecutive readings required for stop   |

## Project Structure

```
LARC25/
├── CMakeLists.txt              # ESP-IDF project manifest
├── platformio.ini              # PlatformIO configuration
├── sdkconfig.esp32-c6-devkitm-1 # ESP-IDF Kconfig output
├── include/                    # Header files (unused in this project)
├── lib/                        # External libraries (unused)
├── src/
│   ├── CMakeLists.txt          # Source-level build config
│   └── main.c                  # Main application (342 lines)
└── test/                       # Unit tests (not implemented)
```

## Known Limitations

1. **Single-Core Constraint**: ESP32-C6 is single-core; no parallel task optimization possible
2. **ADC Noise**: No hardware filtering; relies on 3-sample averaging (could benefit from IIR)
3. **Fixed Telemetry Rate**: 10 Hz output may undersample during rapid transients
4. **Hardcoded Thresholds**: Line detection threshold not ambient-light compensated

## Future Enhancements

- [ ] Implement adaptive threshold calibration via DIP switch selection
- [ ] Add EEPROM persistence for PID parameters
- [ ] Integrate MPU6050 IMU for gyro-assisted turn rate control
- [ ] Bluetooth telemetry for wireless parameter tuning
- [ ] Track odometry via wheel encoder integration

## License

MIT License - see project repository for details.

## Authors

SQM Robotics Team - LARC 2025 Competition Entry

---

⚡ **Build Status**: Tested on ESP-IDF 5.5.0 | PlatformIO Core 6.1.15  
**Last Updated**: December 2025
