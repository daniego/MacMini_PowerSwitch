# Remote Servo Power Controller (ESP32 + W5500)

This project implements a network-controlled servo actuator to physically press the power button of a computer (e.g. Mac Mini).

## Features
- ESP32 with Ethernet (W5500 SPI) and Wi-Fi AP for setup
- Servo short press (~200 ms) and long press (~5 s)
- Web interface (Basic Auth protected) for control and configuration
- Configurable username/password, static IP, Wi-Fi AP SSID/pass
- Factory reset (hold GPIO0 during boot for 10s)
- Status LED and tactile buttons (reset + factory)

## Hardware Pin Map
- **SPI**: MOSI=GPIO23, MISO=GPIO19, SCLK=GPIO18
- **W5500**: CS=GPIO5, INT=GPIO4, RST=GPIO16
- **Servo PWM**: GPIO21 (5V powered, JST-PH-3 header)
- **Factory Button**: GPIO0 -> GND
- **Reset Button**: EN -> GND (hardware reset)
- **Status LED**: GPIO2 (active high)

## Getting Started

### 0. Install ESP-IDF
```bash
git clone --recursive https://github.com/espressif/esp-idf.git -b v5.2
cd esp-idf
./install.sh esp32
. ./export.sh
```

### 1. Clone / copy project
```bash
git clone <this repo> remote-servo
cd remote-servo
```

### 2. Configure (optional)
```bash
idf.py menuconfig
```

### 3. Build
```bash
idf.py build
```

### 4. Flash & Monitor
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### 5. First Use
- Ethernet: connect to LAN, open [http://192.168.1.50](http://192.168.1.50)
- Wi-Fi AP: connect to SSID `Device-XXXX` (password `setup-1234`), browse to [http://192.168.4.1](http://192.168.4.1)
- Login with username `admin` and password `changeme`

## Troubleshooting
- Add bulk capacitor (470–1000 µF) near servo header to prevent resets
- If W5500 fails, reduce SPI clock in `main.c`
- If AP not visible, set Wi-Fi country domain in menuconfig

## ESP tool
Get it set up
```git clone git@github.com:espressif/esptool.git
cd esptool
mkvirtualenv esptool
pip install pyserial
```
### Basic chip identity
`esptool.py --port /dev/ttyUSB0 chip_id`

### Read MAC address
`esptool.py --port /dev/ttyUSB0 read_mac`

### SPI flash manufacturer / device ID
`esptool.py --port /dev/ttyUSB0 flash_id`

### Show the activities
`screen  /dev/cu.usbserial-0001 115200`

## License
MIT
