#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4        // GPIO for DHT22
#define DHTTYPE DHT22   // DHT 22 (AM2302)

#define HEARTBEAT_PIN 2  // GPIO for heartbeat LED
#define FASTBEAT_PIN 15  // GPIO for fastbeat LED
#define BOOT_BUTTON 0    // ESP32-S3 BOOT button (GPIO0)

const char* MY_ID = "io24m006"; 
int captureCount = 1;

DHT dht(DHTPIN, DHTTYPE);

// Message types for queue
typedef enum {
  MSG_SENSOR,
  MSG_USERID,
  MSG_ERROR      
} MessageType_t;

// Message struct for queue
typedef struct {
  MessageType_t type;
  bool success;          // true if sensor read succeeded
  float temperature;
  float humidity;
  char errorMsg[32];     // optional error message
} SensorData_t;

QueueHandle_t sensorQueue;

// Error handler (turn built-in LED red)
void handle_error() {
  noInterrupts();
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  while (true);
}

// Task: read DHT22 every 2s
void dhtTask(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    SensorData_t data;
    data.type = MSG_SENSOR;

    // Read sensor
    data.humidity = dht.readHumidity();
    data.temperature = dht.readTemperature();

    if (!isnan(data.humidity) && !isnan(data.temperature)) {
      data.success = true;
      data.errorMsg[0] = '\0'; // no error
    } else {
      data.success = false;
      data.type = MSG_ERROR;   // mark as error
      strncpy(data.errorMsg, "Failed to read DHT22", sizeof(data.errorMsg));
      handle_error();
    }

    // Always send message to serial task
    xQueueSend(sensorQueue, &data, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(2000)); // FreeRTOS delay (non-blocking)
  }
}

// Task: only task allowed to access Serial
void serialTask(void *pvParameters) {
  (void) pvParameters;
  SensorData_t received;

  for (;;) {
    if (xQueueReceive(sensorQueue, &received, portMAX_DELAY)) {
      if (received.type == MSG_SENSOR) { 

        if (received.success) {
          Serial.printf("%2d: %.1f%%  %.1fC\n", captureCount, received.humidity, received.temperature);
                        captureCount++;
        } else {
          Serial.printf("[ERROR] %s\n", received.errorMsg);
        }
      } else if (received.type == MSG_USERID) {
        Serial.println(MY_ID);
      }
    }
  }
}

// Task: heartbeat (200 ms toggle)
void heartbeatTask(void *pvParameters) {
  (void) pvParameters;
  pinMode(HEARTBEAT_PIN, OUTPUT);
  for (;;) {
    digitalWrite(HEARTBEAT_PIN, !digitalRead(HEARTBEAT_PIN));
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Task: fastbeat (1 ms toggle, high priority)
void fastbeatTask(void *pvParameters) {
  (void) pvParameters;
  pinMode(FASTBEAT_PIN, OUTPUT);
  for (;;) {
    digitalWrite(FASTBEAT_PIN, !digitalRead(FASTBEAT_PIN));
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Task: boot button monitor
void buttonTask(void *pvParameters) {
  (void) pvParameters;
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  bool lastState = HIGH;
  for (;;) {
    bool currentState = digitalRead(BOOT_BUTTON);
    if (lastState == HIGH && currentState == LOW) {
      // Button pressed
      SensorData_t msg = {};
      msg.type = MSG_USERID;
      xQueueSend(sensorQueue, &msg, portMAX_DELAY);
    }
    lastState = currentState;
    vTaskDelay(pdMS_TO_TICKS(50)); // debounce interval
  }
}

void setup() {
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // green status
  Serial.begin(115200);
  Serial.println("ESP32-S3 FreeRTOS DHT22 Reader with Heartbeat, Fastbeat, and Boot Button");

  dht.begin();

  // Create queue
  sensorQueue = xQueueCreate(10, sizeof(SensorData_t));
  if (sensorQueue == NULL) {
    Serial.println("[FATAL] Queue creation failed!");
    handle_error();
  }

  // Create tasks (all pinned to same core)
  xTaskCreatePinnedToCore(dhtTask, "DHT22 Reader", 4096, NULL, 1, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(serialTask, "Serial Printer", 4096, NULL, 2, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(heartbeatTask, "Heartbeat", 2048, NULL, 1, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(fastbeatTask, "Fastbeat", 2048, NULL, 3, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(buttonTask, "Button Monitor", 2048, NULL, 1, NULL, APP_CPU_NUM);
}

void loop() {
  // Nothing here, FreeRTOS handles tasks
}
