#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"
#include "Adafruit_MPU6050.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// --- Credenciales Wi-Fi y Broker MQTT ---
const char *ssid = "OSWALDO";
const char *password = "12345678";

// IP local de tu PC (donde corre Docker EMQX)
const char *mqtt_server = "192.168.1.50"; 
const int mqtt_port = 1883;

// Tópicos MQTT para tus sensores
const char *mqtt_topic_vitals = "iot/reloj/signos";
const char *mqtt_topic_fall   = "iot/reloj/caidas";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

SemaphoreHandle_t i2cMutex;

// --- Sensores y Display ---
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define MY_BUFFER_SIZE 100
uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// Variables globales para compartir entre tareas
float latestBPM = 0;
float latestSpO2 = 0;
float latestAcc = 0;

String deviceCode;

String toBase36(uint32_t value) {
  String result = "";
  const char *digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  if (value == 0) return "0";
  while (value > 0) {
    result = digits[value % 36] + result;
    value /= 36;
  }
  return result;
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Conectando con broker MQTT...");
    String clientId = "ESP32Mini-" + deviceCode;
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" ¡Conectado a EMQX!");
    } else {
      Serial.print(" Fallo rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Reintentando en 3s...");
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
  }
}

// --- Tarea: Signos Vitales (MAX30102) ---
void taskVitalSign(void *parameter) {
  for (;;) {
    const int N = 3;
    float avgBPM = 0;
    float avgSpO2 = 0;
    int validCount = 0;

    for (int n = 0; n < N; n++) {
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MY_BUFFER_SIZE; i++) {
          while (!particleSensor.available()) {
            particleSensor.check();
          }
          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample();
        }
        xSemaphoreGive(i2cMutex);
      }

      maxim_heart_rate_and_oxygen_saturation(
          irBuffer, MY_BUFFER_SIZE, redBuffer,
          &spo2, &validSPO2, &heartRate, &validHeartRate);

      if (validHeartRate && validSPO2 && spo2 > 70 && spo2 <= 100) {
        avgBPM += heartRate;
        avgSpO2 += spo2;
        validCount++;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    if (validCount > 0) {
      latestBPM = avgBPM / validCount;
      latestSpO2 = avgSpO2 / validCount;

      // Actualizar pantalla OLED
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(10, 0);
        display.println("Signos Vitales");
        display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
        display.setTextSize(2);
        display.setCursor(10, 22);
        display.printf("BPM: %d\n", (int)round(latestBPM));
        display.setCursor(10, 44);
        display.printf("SpO2: %d%%\n", (int)round(latestSpO2));
        display.display();
        xSemaphoreGive(i2cMutex);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// --- Tarea: Detección y Aceleración (MPU6050) ---
void taskFall(void *parameter) {
  for (;;) {
    sensors_event_t a, g, temp;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      mpu.getEvent(&a, &g, &temp);
      xSemaphoreGive(i2cMutex);
    }

    latestAcc = sqrt(pow(a.acceleration.x, 2) +
                     pow(a.acceleration.y, 2) +
                     pow(a.acceleration.z, 2));

    String alerta = "";
    if (latestAcc < 2.0) {
      alerta = "Caida Libre";
    } else if (latestAcc > 18.0) {
      alerta = "Impacto";
    }

    if (alerta != "") {
      Serial.printf("ALERTA: %s (Acc: %.2f m/s^2)\n", alerta.c_str(), latestAcc);

      if (mqttClient.connected()) {
        JsonDocument doc;
        doc["deviceCode"] = deviceCode;
        doc["evento"] = alerta;
        doc["aceleracion"] = latestAcc;

        char buffer[128];
        serializeJson(doc, buffer);
        mqttClient.publish(mqtt_topic_fall, buffer);
      }

      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 15);
        display.println("ALERTA DETECTADA:");
        display.setTextSize(2);
        display.setCursor(0, 35);
        display.println(alerta);
        display.display();
        xSemaphoreGive(i2cMutex);
      }
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(8, 9, 100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("No se encontró OLED");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Conectando...");
  display.display();

  uint64_t chipid = ESP.getEfuseMac();
  deviceCode = toBase36((uint32_t)chipid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Conectado!");

  mqttClient.setServer(mqtt_server, mqtt_port);

  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 OK");
  }

  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  }

  xTaskCreate(taskVitalSign, "SignosVitales", 8192, NULL, 1, NULL);
  xTaskCreate(taskFall, "DeteccionCaida", 4096, NULL, 1, NULL);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();

    // Publicación periódica hacia la BD cada 10 segundos
    unsigned long now = millis();
    if (now - lastSend > 10000) {
      lastSend = now;

      JsonDocument doc;
      doc["deviceCode"] = deviceCode;
      doc["heartRate"] = (int)round(latestBPM);
      doc["oxygenSaturation"] = (int)round(latestSpO2);
      doc["aceleracion"] = latestAcc;

      char buffer[128];
      serializeJson(doc, buffer);
      mqttClient.publish(mqtt_topic_vitals, buffer);
      Serial.printf("Publicado a InfluxDB: %s\n", buffer);
    }
  }
  delay(10);
}