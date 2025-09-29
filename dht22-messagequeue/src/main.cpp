#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4       // GPIO where the DHT22 is connected
#define DHTTYPE DHT22  // DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 DHT22 Reader (Interrupt + Timer safe)");
  dht.begin();
}

void loop() {
  delay(2000); // Sensor requires 2s delay between reads

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // Celsius by default

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT22 sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print("°C  Humidity: ");
  Serial.print(h); 
  Serial.println("%");
}
