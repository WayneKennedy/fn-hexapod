# Freenove Big Hexapod - Server Code

## Project Mission

Refactor the Raspberry Pi 5 hexapod robot server code to run on **Ubuntu** instead of Raspberry Pi OS, while maintaining the same Pi 5 hardware platform. The hardware has been verified working under Raspberry Pi OS using this codebase.

## Hardware Platform

- **Board**: Raspberry Pi 5
- **Target OS**: Ubuntu (migrating from Raspberry Pi OS)
- **Robot Kit**: Freenove Big Hexapod
- **PCB Versions**: V1.0 and V2.0 (code supports both)

## Hardware Interfaces & Dependencies

### I2C Devices (Bus 1)
| Device | Address | Purpose | Library |
|--------|---------|---------|---------|
| PCA9685 | 0x40 | 16-channel PWM servo driver | `smbus` |
| ADS7830 | 0x48 | 8-channel ADC (battery voltage) | `smbus` |
| MPU6050 | 0x68 | 6-axis IMU (accelerometer/gyroscope) | `mpu6050` (custom lib in `../Libs/mpu6050`) |

### LED Control
- **WS281x LEDs**: 7 addressable RGB LEDs
- **PCB V1.0 + Pi 3/4**: Uses `rpi_ws281x` via GPIO18 (PWM) - `rpi_ledpixel.py`
- **PCB V2.0**: Uses SPI interface - `spi_ledpixel.py`
- **Note**: PCB V1.0 is NOT supported on Pi 5 (see `led.py:28`)

### Camera
- Uses `picamera2` and `libcamera`
- Supports OV5647 and IMX219 camera modules
- Streams JPEG over TCP port 8002

### Other Peripherals
- **Ultrasonic Sensor**: GPIO-based distance measurement (`ultrasonic.py`)
- **Buzzer**: GPIO control (`buzzer.py`)
- **Servos**: 18 servos for 6 legs (3 per leg) via PCA9685

## Key Files

| File | Purpose |
|------|---------|
| `main.py` | Entry point - PyQt5 UI or headless TCP server |
| `server.py` | TCP command server (port 5002) and video streaming |
| `control.py` | Hexapod gait and movement control algorithms |
| `pca9685.py` | PWM servo driver interface |
| `servo.py` | Servo abstraction layer |
| `imu.py` | IMU sensor fusion with Kalman filtering |
| `led.py` | LED control dispatcher |
| `camera.py` | Picamera2 video streaming |
| `adc.py` | Battery voltage monitoring |

## Ubuntu Migration Challenges

### Known Issues to Address

1. **I2C Access**
   - Raspberry Pi OS: `/dev/i2c-1` available by default
   - Ubuntu: May require `i2c-tools` package and kernel module loading
   - User permissions: May need `i2c` group membership

2. **WS281x LED Library**
   - `rpi-ws281x-python` (in `../Libs/rpi-ws281x-python`) has Pi-specific dependencies
   - Uses DMA/PWM hardware that differs between Pi OS and Ubuntu kernels
   - May need `lgpio` or alternative GPIO library on Ubuntu

3. **Camera Stack**
   - `picamera2` depends on `libcamera` which has different packaging on Ubuntu
   - Device tree overlays configured differently (`/boot/firmware/config.txt` vs Ubuntu methods)

4. **GPIO Access**
   - Raspberry Pi OS uses `RPi.GPIO` with `/dev/gpiomem`
   - Ubuntu on Pi 5 may need `lgpio`, `gpiod`, or `libgpiod`

5. **SPI**
   - Used by `spi_ledpixel.py` for PCB V2.0 LED control
   - Needs SPI enabled in device tree and proper permissions

### Configuration Files
- Boot config: `/boot/firmware/config.txt`
- Required overlays: SPI, I2C, camera dtoverlay

## Communication Protocol

Commands received via TCP port 5002, video streamed on port 8002. See `robot_control_communication_protocol.md` in parent directory for full protocol specification.

## Running the Server

```bash
# With PyQt5 UI
python3 main.py

# Headless with TCP server auto-start
python3 main.py -t -n
```

## Testing Strategy

Test hardware interfaces incrementally:
1. I2C bus scan: `i2cdetect -y 1`
2. Individual module tests: `python3 adc.py`, `python3 imu.py`, etc.
3. LED test: `python3 led.py`
4. Camera test: `python3 camera.py`
5. Full integration: `python3 main.py -t -n`

## Dependencies

### System Packages
- `python3-pyqt5`
- `libqt5gui5`
- `python3-dev`
- `i2c-tools`
- `python3-smbus` (or `smbus2`)
- `libcamera` related packages

### Python Packages
- `picamera2`
- `rpi-ws281x` (custom build from `../Libs/rpi-ws281x-python`)
- `mpu6050` (custom build from `../Libs/mpu6050`)

### Custom Libraries Location
- `../Libs/rpi-ws281x-python/` - WS281x LED driver
- `../Libs/mpu6050/` - MPU6050 IMU driver
