# esp32-gps

An ESP32 Arduino sketch that reads a serial (NMEA) GPS module with [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) and prints the position to the serial monitor once a minute.

## What the sketch does

[`esp32_gps/esp32_gps.ino`](esp32_gps/esp32_gps.ino):

* Opens the serial monitor at **115200** baud.
* Opens a second UART for the GPS at **9600** baud, 8N1, with **RX on GPIO 4** and **TX on GPIO 5**.
* Every **60 seconds** (using a `Ticker`), reads whatever the GPS has sent and, if the location was updated, prints:
  * `Latitude` and `Longitude` (6 decimals)
  * `Altitude` (meters)
  * `Speed` (km/h)
  * `Satellites` used in the fix
* Optionally watches the GPS **PPS** (pulse-per-second) output on **GPIO 18** with an interrupt. The sketch records the time of the last pulse but does not use it yet.

`loop()` is empty; all work happens in the timer callback.

## Wiring

| GPS module | ESP32 |
|------------|-------|
| TX | GPIO 4 (ESP32 RX) |
| RX | GPIO 5 (ESP32 TX) |
| PPS (optional) | GPIO 18 |
| VCC / GND | Power and ground, as required by your module |

## Requirements

* [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support (`HardwareSerial` and `Ticker` come with the ESP32 core).
* The [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) library, available in the Arduino Library Manager.

## Usage

1. Open `esp32_gps/esp32_gps.ino`.
2. Adjust the pins or baud rate in `setup()` if your wiring or module differs.
3. Select your ESP32 board and port, then upload.
4. Open the serial monitor at 115200 baud. Give the module time to get a fix outdoors; output appears at most once a minute and only when the location was updated.

## Known limitations

* The GPS serial port is only read once a minute. A typical module sends NMEA sentences continuously, so the tens of kilobytes that arrive between reads overflow the UART receive buffer (256 bytes by default, plus the hardware FIFO) and most of the data is lost; the printed fix may therefore be incomplete or stale. Reading the port continuously in `loop()` and printing once a minute avoids this.

## License

[Apache License 2.0](LICENSE)
