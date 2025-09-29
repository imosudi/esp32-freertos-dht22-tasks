# ESP32 FreeRTOS DHT22 Multi-Task System

This project demonstrates a **production-ready FreeRTOS application** for the ESP32-S3 that manages DHT22 sensor readings alongside multiple concurrent tasks with different priorities and timing requirements, message queues, and GPIO handling on a single core.

## Features

### Core Tasks

- **DHT22 Reader Task (Priority 1)**  
  - Reads temperature and humidity every 2 seconds using the Adafruit DHT library
  - Sends sensor data to the serial task via FreeRTOS message queue
  - Implements error handling with visual feedback (red LED on failure)
  - Non-blocking operation using `vTaskDelay()`

- **Serial Task (Priority 2)**  
  - **Exclusive Serial port access** - only task that writes to Serial
  - Receives messages from queue and formats output
  - Handles multiple message types: sensor data, user ID, and errors
  - Displays incrementing capture count with readings

- **Heartbeat Task (Priority 1)**  
  - Toggles GPIO 2 every 200 ms to indicate system is running
  - Visual health indicator for the entire system

- **Fastbeat Task (Priority 3 - Highest)**  
  - Toggles GPIO 15 every 1 ms
  - Demonstrates high-priority task preemption
  - Useful for testing real-time scheduling behavior

- **Boot Button Monitor Task (Priority 1)**  
  - Monitors ESP32-S3 BOOT button (GPIO0) with software debouncing
  - Prints user ID (`io24xxxx`) to serial when button is pressed
  - Demonstrates inter-task communication via queue

### Architecture Highlights

- **Message Queue Pattern**: Thread-safe communication between tasks
- **Single Core Execution**: All tasks pinned to `APP_CPU_NUM` for deterministic behavior
- **Priority-Based Scheduling**: Demonstrates FreeRTOS preemptive multitasking
- **Error Handling**: System halts with red LED indicator on critical failures
- **Status Indicators**: Green LED on successful initialization

## Hardware Requirements

- **ESP32-S3** Development Board (with built-in RGB LED)
- **DHT22** (AM2302) Temperature & Humidity Sensor
- 10kΩ pull-up resistor for DHT22 data line (recommended)
- Jumper wires and breadboard
- (Optional) External LEDs for better visualization of heartbeat/fastbeat

## Pin Configuration

| Component          | ESP32 Pin | Notes                        |
|--------------------|-----------|------------------------------|
| DHT22 Data         | GPIO 4 #GPIO (configurable)   | Requires 10kΩ pull-up       |
| Heartbeat LED      | GPIO 2 #GPIO (configurable)   | 200 ms toggle rate          |
| Fastbeat LED       | GPIO 15 #GPIO (configurable)   | 1 ms toggle rate            |
| Boot Button        | GPIO 0    | Built-in, active LOW        |
| Built-in RGB LED   | RGB_BUILTIN | Status: Green=OK, Red=Error |

> **Note**: Update pin assignments in `main.cpp` if using different GPIOs.

## How It Works

### System Flow

1. **Initialization**
   - RGB LED turns green indicating successful startup
   - DHT22 sensor initialized
   - Message queue created (capacity: 10 messages)
   - Five FreeRTOS tasks created and pinned to APP_CPU_NUM

2. **Runtime Operation**
   - DHT22 task reads sensor every 2 seconds and enqueues data
   - Serial task waits on queue and prints formatted output
   - Heartbeat task provides visual 200ms pulse
   - Fastbeat task provides high-speed 1ms pulse (highest priority)
   - Button task monitors for user input and sends user ID on press

3. **Error Handling**
   - Failed sensor reads trigger error message to queue
   - Critical failures halt system with red LED indicator
   - All interrupts disabled in error state for debugging

### Message Queue Structure

```cpp
typedef struct {
  MessageType_t type;    // MSG_SENSOR, MSG_USERID, or MSG_ERROR
  bool success;          // Sensor read success flag
  float temperature;     // Temperature in Celsius
  float humidity;        // Relative humidity percentage
  char errorMsg[32];     // Error description
} SensorData_t;
```

## Project Structure

```
esp32-freertos-dht22-tasks/
├── dht22-messagequeue/
│   ├── platformio.ini        # PlatformIO configuration
│   ├── src/
│   │   └── main.cpp          # Main application code
│   ├── include/              # Header files
│   ├── lib/                  # Project libraries
│   └── test/                 # Unit tests
├── LICENSE                   # BSD-3-Clause
└── README.md                 # This file
```

## Build & Flash

### Prerequisites

```bash
# Install PlatformIO
pip3 install platformio

# Or using pipx (recommended)
pipx install platformio
```

### Using PlatformIO

```bash
# Build project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio run --target monitor

# Combined: upload and monitor
pio run --target upload --target monitor
```

### Erase Flash (if needed)

```bash
pio run --target erase
```

## Example Serial Output

```
ESP32-S3 FreeRTOS DHT22 Reader with Heartbeat, Fastbeat, and Boot Button
 1: 61.2%  27.4C
 2: 61.4%  27.3C
 3: 61.3%  27.5C
io24xxxx
 4: 61.5%  27.4C
[ERROR] Failed to read DHT22
```

## Dependencies

Listed in `platformio.ini`:
- `adafruit/Adafruit Unified Sensor`
- `adafruit/DHT sensor library`

## Configuration Options

### Modify Task Priorities

In `main.cpp`, adjust priority values (higher = more important):
```cpp
xTaskCreatePinnedToCore(dhtTask,      "DHT22 Reader",   4096, NULL, 1, NULL, APP_CPU_NUM);
xTaskCreatePinnedToCore(serialTask,   "Serial Printer", 4096, NULL, 2, NULL, APP_CPU_NUM);
xTaskCreatePinnedToCore(fastbeatTask, "Fastbeat",       2048, NULL, 3, NULL, APP_CPU_NUM);
```

### Change Sensor Read Interval

Modify delay in `dhtTask()`:
```cpp
vTaskDelay(pdMS_TO_TICKS(2000)); // Currently 2000 ms
```

### Update User ID

Change the constant at top of `main.cpp`:
```cpp
const char* MY_ID = "io24xxxx"; // Your identifier
```

## Troubleshooting

**Sensor reads always fail:**
- Check DHT22 connections and power supply
- Verify 10kΩ pull-up resistor on data line
- Ensure GPIO 4 is not used by another peripheral

**No serial output:**
- Verify baud rate is 115200
- Check USB cable supports data transfer
- Try different USB port

**Button not responding:**
- GPIO 0 is used for boot mode - don't hold during startup
- Check for 50ms debounce delay

**Tasks not running:**
- Verify sufficient heap memory
- Check stack sizes (increase if needed)
- Monitor for stack overflow errors

## Future Enhancements

- [ ] Add WiFi/MQTT integration for remote monitoring
- [ ] Implement data logging to SD card or SPIFFS
- [ ] Add interrupt-based button handling (ISR)
- [ ] Implement watchdog timer for system reliability
- [ ] Add low-power sleep modes between readings
- [ ] Support multiple DHT sensors
- [ ] Create web dashboard for real-time monitoring

## License

This project is released under the **BSD-3-Clause** license.

## Author

imosudi  
IoT Operating Systems
