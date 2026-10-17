# MegaCAN
Arduino library for interfacing with the Megasquirt family of ECUs via the CAN bus

# Dependencies
 * [ArduinoLogging](https://github.com/hankedan000/ArduinoLogging)
 * [mcp_can]()

# Installation
## PlatformIO
Add the library to `lib_deps` in your `platformio.ini`. Dependencies are installed automatically.
```ini
lib_deps =
    https://github.com/hankedan000/MegaCAN.git
```

## Arduino IDE
Clone this repository into your Arduino `libraries` folder and install the dependencies listed above, plus [TaskScheduler](https://github.com/arkhipenko/TaskScheduler) 3.7.0.
