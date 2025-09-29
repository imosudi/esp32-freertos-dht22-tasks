# ESP32 FreeRTOS DHT22 Tasks

This project demonstrates how to integrate the **DHT22 sensor** into a **FreeRTOS** application running on the ESP32.  
It shows the use of multiple FreeRTOS tasks, message queues, and GPIO handling on a single core.  

## Features

- **DHT22 Task**  
  - Reads temperature and humidity from the DHT22 sensor.  
  - Converts raw sensor data into human-readable format.  
  - Sends data to the serial task via a FreeRTOS message queue.  

- **Serial Task**  
  - Receives DHT22 data from the message queue.  
  - Prints sensor values to the UART serial port.  

- **Heartbeat Task**  
  - Toggles a GPIO pin every **200 ms** to indicate system activity.  

- **Fastbeat Task (High Priority)**  
  - Toggles a GPIO pin every **1 ms** to demonstrate high-priority task scheduling.  

- **Boot Button Task**  
  - Monitors the ESP32 boot button.  
  - Can be extended for user-defined actions (e.g., reset, calibration, etc.).  

- **Single-Core Execution**  
  - All tasks are pinned to the same ESP32 core for deterministic scheduling.  

## Hardware Requirements

- ESP32 Development Board  
- DHT22 Temperature & Humidity Sensor  
- Jumper wires and breadboard  
- (Optional) LEDs for heartbeat and fastbeat task visualisation  

## Connections

| Component | ESP32 Pin |
|-----------|-----------|
| DHT22 Data | GPIO (configurable) |
| Heartbeat LED | GPIO (configurable) |
| Fastbeat LED | GPIO (configurable) |
| Boot Button | Built-in (GPIO0) |

> Update pin assignments in the source code as required.

## How It Works

1. The **DHT22 task** periodically samples temperature and humidity data.  
2. Data is placed in a **FreeRTOS message queue**.  
3. The **Serial task** consumes queue data and outputs it via UART.  
4. The **Heartbeat task** toggles a GPIO pin every 200 ms.  
5. The **Fastbeat task** toggles a GPIO pin every 1 ms (highest priority).  
6. The **Boot button task** monitors the user button for input events.  

## Project Structure

```
.
├── src/
│   ├── main.c        # Application entry point
│   ├── dht22.c       # DHT22 sensor driver
│   ├── dht22.h
│   └── freertos_app.c # Task creation and scheduling
├── include/
│   └── config.h      # Pin mappings and configuration
├── README.md
└── CMakeLists.txt / platformio.ini
```

## Build & Flash

### Using PlatformIO
```bash
pio run
pio run --target upload
```

### Using ESP-IDF
```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py monitor
```

## Example Serial Output

```
[Heartbeat] Toggled pin at 200 ms
[Fastbeat] Toggled pin at 1 ms
[DHT22] Temperature: 27.4 °C, Humidity: 61.2 %
[DHT22] Temperature: 27.3 °C, Humidity: 61.4 %
[Boot Button] Press detected
```

## Future Improvements

- Add error handling for sensor read failures.  
- Implement event-driven button handling (interrupt-based).  
- Store data in non-volatile memory or send over Wi-Fi/MQTT.  
- Add power optimisation features.  

## License

This project is released under the MIT License.
