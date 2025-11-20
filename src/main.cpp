#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
// #include <HardwareSerial.h>   // UART para GPS y SIM800L
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <ArduinoJson.h>

MAX30105 particleSensor; // Sensor de pulso

// buffers para el calculo de spo2 bpm
#define MY_BUFFER_SIZE 100
uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

Adafruit_MPU6050 mpu; // Acelerómetro/Giroscopio

// variables de giro y aceleracion
float ax, ay, az;

// configuraciones para pantalla

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/*

*/
String deviceCode;

bool isVinculed;

const char *ssid = "OSWALDO";
const char *password = "12345678";
// para configurar hora
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -5 * 3600; // horario peru
const int daylightOffset_sec = 0;

void enviarSignosVitalesBackend(float bpm, float spo2);

void enviarCaidaBackend();

void verificarVinculo();

String toBase36(uint32_t value);

void taskVitalSign(void *parameter)
{
  for (;;)
  {
    // Cada 30 segundos
    vTaskDelay(30000 / portTICK_PERIOD_MS);

    // Generar valores aleatorios dentro del rango indicado
    float avgBPM = random(80, 121);   // 80 a 120
    float avgSpO2 = random(90, 99);   // 90 a 98

    Serial.print("BPM simulado: ");
    Serial.print(avgBPM);
    Serial.print(" | SpO2 simulado: ");
    Serial.print(avgSpO2);
    Serial.println("%");

    // Enviar al backend
    enviarSignosVitalesBackend(avgBPM, avgSpO2);

    // Mostrar en pantalla
    display.clearDisplay();

    // Título
    display.setTextSize(1);
    display.setCursor(10, 0);
    display.println("Signos Vitales");

    // Línea separadora
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    // Datos BPM
    display.setTextSize(2);
    display.setCursor(10, 25);
    display.print("BPM: ");
    display.println((int)avgBPM);

    // Datos SpO2
    display.setCursor(10, 50);
    display.print("SpO2: ");
    display.println((int)avgSpO2);

    display.display();
  }
}

void taskFall(void *parameter)
{
  unsigned long lastAlertTime = 0; // tiempo de la última alerta en ms

  for (;;)
  {
    // ---- generar aceleración simulada ----
    // 80% movimiento normal, 10% caída, 10% impacto
    int r = random(0, 100);
    float totalAcc;

    if (r < 80)
    {
      totalAcc = random(500, 1200) / 100.0; // 5.0 a 12.0
    }
    else if (r < 90)
    {
      totalAcc = random(50, 150) / 100.0; // 0.5 a 1.5 caída
    }
    else
    {
      totalAcc = random(1500, 2500) / 100.0; // 15.0 a 25.0 impacto
    }

    // mostrar aceleración simulada
    Serial.print("Aceleracion total simulada: ");
    Serial.print(totalAcc);
    Serial.println(" m/s^2");

    // ---- ver si ya pasó 1 minuto desde la última alerta ----
    unsigned long now = millis();
    bool puedeEnviar = (now - lastAlertTime >= 60000);

    // ---- detección de alerta ----
    if (totalAcc < 2 && puedeEnviar)
    {
      Serial.println("Posible caída libre detectada (simulada)");
      enviarCaidaBackend();
      lastAlertTime = now;

      display.clearDisplay();
      display.setCursor(0, 25);
      display.println("Caida libre detectada");
      display.display();
    }
    else if (totalAcc > 15 && puedeEnviar)
    {
      Serial.println("Impacto detectado (simulado)");
      enviarCaidaBackend();
      lastAlertTime = now;

      display.clearDisplay();
      display.setCursor(0, 25);
      display.println("Impacto detectado!");
      display.display();
    }

    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}


void setup()
{
  // iniciar puerto serial para debug
  delay(5000);
  Serial.begin(115200);

Serial.println("Escaneando redes disponibles...");
int n = WiFi.scanNetworks();
for (int i = 0; i < n; i++) {
  Serial.print(i + 1);
  Serial.print(": ");
  Serial.print(WiFi.SSID(i));
  Serial.print(" (canal ");
  Serial.print(WiFi.channel(i));
  Serial.println(")");
}

  Wire.begin(8, 9, 100000);

  


    // intentamos buscar la pantalla
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("No se encontró pantalla OLED");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();

  byte error, direccion;
  int dispositivos = 0;

  Serial.println("Buscando dispositivos I2C...\n");

  for (direccion = 1; direccion < 127; direccion++) {
    Wire.beginTransmission(direccion);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en dirección: 0x");
      display.print("Dispositivo encontrado en dirección: 0x");
      Serial.println(direccion, HEX);
      display.println(direccion, HEX);
      display.display();
      dispositivos++;
      delay(5);
    }
    else if (error == 4) {
      Serial.print("Error desconocido en dirección 0x");
      Serial.println(direccion, HEX);
    }
  }

  if (dispositivos == 0) {
    Serial.println("\nNo se encontraron dispositivos I2C.");
  } else {
    Serial.println("\nEscaneo I2C completado.");
  }

  Serial.println("Conectando al WiFi...");
  WiFi.begin(ssid, password);
  Serial.println(ssid);
  Serial.println(password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Estado WiFi: ");
    Serial.println(WiFi.status());
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // configurar ntp
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // obtenemos hora
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Error al obtener la hora");
    return;
  }

  Serial.println("Hora sincronizada correctamente:");
  Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");

  randomSeed(esp_random());

  // para chip id
  uint64_t chipid = ESP.getEfuseMac();
  deviceCode = toBase36(chipid);
  Serial.print("Codigo generado");
  Serial.print(deviceCode);

  Serial.println("Iniciando sistema reloj...");

  Serial.println("\nEscaneo I2C iniciado...");



  // configuracion del mpu--acelerometro-giroscopi


  // configuracion inicar de la pantalla
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Codigo:");
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;

  // calcular el tamaño del texto
  display.getTextBounds(deviceCode, 0, 0, &x1, &y1, &w, &h);

  // coordenadas
  int x = (SCREEN_WIDTH - w) / 2;
  int y = (SCREEN_HEIGHT - h) / 2;

  // Posiciona y escribe el texto
  display.setCursor(x, y);
  Serial.println("Codigo:");
  display.println(deviceCode);
  display.display();
  while (!isVinculed)
  {
    verificarVinculo();
    delay(4000);
  }
  display.clearDisplay();
  display.setCursor(0,0);
  Serial.println("Vinculo verificado");
  display.println("Vinculo verificado");
  display.display();
  /*
  display.clearDisplay();*/
  xTaskCreate(taskVitalSign, "SignosVitales", 8192, NULL, 1, NULL);
  xTaskCreate(taskFall, "Caida", 8192, NULL, 1, NULL);
}
void loop()
{
  /*
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en la direccion 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Error desconocido en la direccion 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No se encontraron dispositivos I2C\n");
  else
    Serial.println("Escaneo completado.\n");*/
}

void mostrarSignosVitalesPantalla(float bpm, float spo2)
{
}

void enviarSignosVitalesBackend(float bpm, float spo2)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;

    String server = "https://apucha-watch-backend-1094750444303.us-west1.run.app/vital-signs";
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String json = "{\"deviceCode\": \"" + deviceCode + "\", \"heartRate\": " + String(round(bpm)) +
                  ", \"oxygenSaturation\": " + String(round(spo2)) + "}";

    int httpResponseCode = http.POST(json);

    if (httpResponseCode > 0)
    {
      Serial.print("Código de respuesta: ");
      Serial.println(httpResponseCode);
      String respuesta = http.getString();
      Serial.println(respuesta);
    }
    else
    {
      Serial.print("Error en POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi desconectado");
  }
}

void enviarCaidaBackend()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;

    String server = "https://apucha-watch-backend-1094750444303.us-west1.run.app/fall";
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String json = "{\"deviceCode\": \"" + deviceCode + "\"}";

    int httpResponseCode = http.POST(json);

    if (httpResponseCode > 0)
    {
      Serial.print("Código de respuesta: ");
      Serial.println(httpResponseCode);
      String respuesta = http.getString();
      Serial.println(respuesta);
    }
    else
    {
      Serial.print("Error en POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi desconectado");
  }
}

void verificarVinculo()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;

    String server = "https://apucha-watch-backend-1094750444303.us-west1.run.app/devices/exist-by-code/" + deviceCode;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      Serial.print("Código de respuesta: ");
      Serial.println(httpResponseCode);
      String respuesta = http.getString();
      Serial.println(respuesta);
      // deserializamos json
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, respuesta);
      if (!error)
      {
        isVinculed = doc["exist"];
        Serial.println("Esta vinculado");
        Serial.println(isVinculed);
      }
      else
      {
        Serial.println("Error al pasear json");
        Serial.println(error.c_str());
      }
    }
    else
    {
      Serial.print("Error en POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi desconectado");
  }
}

String toBase36(uint32_t value) {
  String result = "";
  const char* digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  if (value == 0) return "0";

  while (value > 0) {
    result = digits[value % 36] + result;
    value /= 36;
  }

  return result;
}