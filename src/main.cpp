#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
#include <HardwareSerial.h>   // UART para GPS y SIM800L
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <ArduinoJson.h>

// pines de los botones
#define PIN_BOTON_1 2
#define PIN_BOTON_2 3

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

// para sim800l
HardwareSerial sim(1); // UART1

bool llamadaActiva = false;

unsigned long tiempoInicioLlamada = 0;

void enviarSignosVitalesBackend(float bpm, float spo2);

void enviarCaidaBackend();

void verificarVinculo();

String toBase36(uint32_t value);

void iniciarGPRS(String apn);
bool esperarRespuesta(String esperado, unsigned long timeout);

void enviarAT(String cmd);

void taskVitalSign(void *parameter)
{
  for (;;)
  {
    const int N = 5; // n de promedios
    float avgBPM = 0;
    float avgSpO2 = 0;
    int validCount = 0;

    for (int n = 0; n < N; n++)
    {
      // --lectura en crudo
      for (int i = 0; i < MY_BUFFER_SIZE; i++)
      {
        while (!particleSensor.available())
        {
          particleSensor.check();
        }
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
      }

      // aplicar algoritmo de SparkFun
      maxim_heart_rate_and_oxygen_saturation(
          irBuffer, MY_BUFFER_SIZE, redBuffer,
          &spo2, &validSPO2, &heartRate, &validHeartRate);

      //  si es válida, acumular
      if (validHeartRate && validSPO2 && spo2 > 70 && spo2 < 100)
      {
        avgBPM += heartRate;
        avgSpO2 += spo2;
        validCount++;
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS); // espera entre lecturas
    }

    // promedio de lecturas validas
    if (validCount > 0)
    {
      avgBPM /= validCount;
      avgSpO2 /= validCount;

      Serial.print("BPM promedio: ");
      Serial.print(avgBPM, 1);
      Serial.print(" | SpO2 promedio: ");
      Serial.print(avgSpO2, 1);
      Serial.println("%");
      // enviar al backend
      enviarSignosVitalesBackend(avgBPM, avgSpO2);
      display.clearDisplay();
      // titulo
      display.setTextSize(1);
      display.setCursor(10, 0);
      display.println("Signos Vitales");
      // linea separadora
      display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
      // datos
      display.setTextSize(2);
      display.setCursor(10, 25);
      display.print("BPM: ");
      display.println((int)round(avgBPM));

      display.setCursor(10, 50);
      display.print("SpO2: ");
      display.println((int)round(avgSpO2));
      // mandamos a imprimir
      display.display();
    }
    else
    {
      Serial.println("Lecturas no válidas, intentar de nuevo");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void taskFall(void *parameter)
{
  for (;;)
  {
    // leemos aceleracion
    sensors_event_t a,
        g, temp;
    mpu.getEvent(&a, &g, &temp);
    float totalAcc = sqrt(pow(a.acceleration.x, 2) +
                          pow(a.acceleration.y, 2) +
                          pow(a.acceleration.z, 2));
    Serial.print("Aceleracion total: ");
    delay(500);
    Serial.print(totalAcc);
    delay(500);
    Serial.print(" m/s^2 | ");

    if (totalAcc < 2)
    {
      Serial.println("Posible caída libre detectada");
      enviarCaidaBackend();
      display.clearDisplay();
      display.setCursor(10, 25);
      display.println("Posible caída libre detectada");
    }
    else if (totalAcc > 15)
    {
      Serial.println("Impacto detectado, enviando alerta...");
      enviarCaidaBackend();
      display.clearDisplay();
      display.setCursor(10, 25);
      display.println("Impacto detectado, enviando alerta...");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void taskCall(void *parameter)
{
  for (;;)
  {
    if (sim.available())
    {
      String respuesta = sim.readStringUntil('\n');
      respuesta.trim();

      if (respuesta == "RING")
      {
        Serial.println("Llamada entrante detectada...");
      }
      else if (respuesta.startsWith("+CLIP:"))
      {
        Serial.println("Número que llama: " + respuesta);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // esperamos 2 segundos
        Serial.println("Contestando automáticamente...");
        sim.println("ATA"); // contestamos
        tiempoInicioLlamada = millis();
        llamadaActiva = true;
      }
      else if (respuesta.indexOf("NO CARRIER") != -1)
      {
        Serial.println("Llamada finalizada o rechazada");
        llamadaActiva = false;
      }
      else if (respuesta.indexOf("BUSY") != -1)
      {
        Serial.println("Línea ocupada");
        llamadaActiva = false;
      }
    }

    // Si hay una llamada activa, medimos tiempo
    if (llamadaActiva)
    {
      unsigned long duracion = (millis() - tiempoInicioLlamada) / 1000;

      if (duracion > 10)
      { // colgamos despues de 10 segundos
        Serial.println("⏱️ Llamada finalizada automáticamente (10s)");
        sim.println("ATH"); // colgar
        llamadaActiva = false;
      }
    }
  }
}

void setup()
{
  // iniciar puerto serial para debug
  delay(5000);
  Serial.begin(115200);

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

  // para chip id
  uint64_t chipid = ESP.getEfuseMac();
  deviceCode = toBase36(chipid);
  Serial.print("Codigo generado");
  Serial.print(deviceCode);

  Serial.println("Iniciando sistema reloj...");
  Wire.begin(8, 9, 100000);
  Serial.println("\nEscaneo I2C iniciado...");

  // conectamos sim800l
  sim.begin(9600, SERIAL_8N1, 20, 21);
  Serial.println("Iniciando SIM800L...");
  enviarAT("AT");
  enviarAT("AT+CSQ");
  // conectamos a internet
  iniciarGPRS("entel.pe");

  // inicializamos botones
    pinMode(PIN_BOTON_1, INPUT_PULLUP);
   pinMode(PIN_BOTON_2, INPUT_PULLUP);

  // configuracion del max 30102 - pulso
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD))
  {
    Serial.println("No se encontró MAX30102");
    // display.println("MAX30102 ERROR");
  }
  else
  {
    Serial.println("MAX30102 OK");
    // display.println("MAX30102 OK");
    // display.display();
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
  }

  // configuracion del mpu--acelerometro-giroscopio
  if (!mpu.begin())
  {
    Serial.println("No se encontró MPU6050");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 OK");
  // conditional
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // intentamos buscar la pantalla
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("No se encontró pantalla OLED");
  }
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
  display.setCursor(0, 0);
  Serial.println("Vinculo verificado");
  display.println("Vinculo verificado");
  display.display();
  /*
  display.clearDisplay();*/
  xTaskCreate(taskVitalSign, "SignosVitales", 8192, NULL, 1, NULL);
  xTaskCreate(taskFall, "Caida", 8192, NULL, 1, NULL);
  xTaskCreate(taskCall, "Llamada", 4096, NULL, 1, NULL);
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

void enviarSignosVitalesBackendGrps(float bpm, float spo2)
{
  String url = "https://apucha-watch-backend-1094750444303.us-west1.run.app/vital-signs";

  // construimos el JSON
  String json = "{\"deviceCode\":\"" + deviceCode + "\",\"heartRate\":" + String(round(bpm)) +
                ",\"oxygenSaturation\":" + String(round(spo2)) + "}";

  Serial.println("Enviando signos vitales...");
  Serial.println(json);

  enviarAT("AT+HTTPTERM"); // cerramos sesion previa
  delay(1000);
  enviarAT("AT+HTTPINIT");
  enviarAT("AT+HTTPPARA=\"CID\",1");
  enviarAT("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  enviarAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");

  // Indicar que enviaremos datos (tamaño y tiempo máximo)
  sim.println("AT+HTTPDATA=" + String(json.length()) + ",10000");
  delay(100);
  if (esperarRespuesta("DOWNLOAD", 5000))
  {
    sim.print(json); // enviar el cuerpo JSON
    delay(2000);
  }

  enviarAT("AT+HTTPACTION=1"); // Post
  delay(8000);

  // leemos respuesta respuesta
  sim.println("AT+HTTPREAD");
  delay(3000);

  String respuesta = "";
  while (sim.available())
  {
    respuesta += (char)sim.read();
  }

  Serial.println("Respuesta del servidor:");
  Serial.println(respuesta);

  enviarAT("AT+HTTPTERM");
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

void enviarCaidaBackendGrps()
{
  String url = "https://apucha-watch-backend-1094750444303.us-west1.run.app/fall";
  String json = "{\"deviceCode\": \"" + deviceCode + "\"}";
  Serial.println("Enviando caida...");
  Serial.println(json);

  enviarAT("AT+HTTPTERM"); // cerramos sesion previa
  delay(1000);
  enviarAT("AT+HTTPINIT");
  enviarAT("AT+HTTPPARA=\"CID\",1");
  enviarAT("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  enviarAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  // Indicar que enviaremos datos (tamaño y tiempo máximo)
  sim.println("AT+HTTPDATA=" + String(json.length()) + ",10000");
  delay(100);
  if (esperarRespuesta("DOWNLOAD", 5000))
  {
    sim.print(json); // enviar el cuerpo JSON
    delay(2000);
  }

  enviarAT("AT+HTTPACTION=1"); // Post
  delay(8000);

  // leemos respuesta respuesta
  sim.println("AT+HTTPREAD");
  delay(3000);

  String respuesta = "";
  while (sim.available())
  {
    respuesta += (char)sim.read();
  }

  Serial.println("Respuesta del servidor:");
  Serial.println(respuesta);

  enviarAT("AT+HTTPTERM");
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

void verificarVinculoGrps()
{
  String url = "https://apucha-watch-backend-1094750444303.us-west1.run.app/devices/exist-by-code/" + deviceCode;
  Serial.println("Consultando URL:");
  Serial.println(url);

  enviarAT("AT+HTTPTERM"); // cerramos sesion previa
  delay(1000);

  enviarAT("AT+HTTPINIT");
  enviarAT("AT+HTTPPARA=\"CID\",1");

  enviarAT("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  enviarAT("AT+HTTPACTION=0"); // peticion get
  delay(6000);
  // leer respuesta

  sim.println("AT+HTTPREAD");
  delay(3000);

  String respuesta = "";
  while (sim.available())
  {
    respuesta += (char)sim.read();
  }

  Serial.println("Respuesta del servidor:");
  Serial.println(respuesta);
  // deserializamos json
  int jsonStart = respuesta.indexOf("{");
  int jsonEnd = respuesta.lastIndexOf("}");
  if (jsonStart != -1 && jsonEnd != -1)
  {
    String jsonStr = respuesta.substring(jsonStart, jsonEnd + 1);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (!error)
    {
      isVinculed = doc["exist"];
      Serial.println("Esta vinculado:");
      Serial.println(isVinculed);
    }
    else
    {
      Serial.println("Error al parsear JSON:");
      Serial.println(error.c_str());
    }
  }
  else
  {
    Serial.println("No se encontró JSON en la respuesta");
  }

  enviarAT("AT+HTTPTERM");
}

String toBase36(uint32_t value)
{
  String result = "";
  const char *digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  if (value == 0)
    return "0";

  while (value > 0)
  {
    result = digits[value % 36] + result;
    value /= 36;
  }

  return result;
}

// para conectarse a una red
void iniciarGPRS(String apn)
{
  Serial.println("Conectando a red GPRS...");
  enviarAT("AT");
  enviarAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  enviarAT("AT+SAPBR=3,1,\"APN\",\"" + apn + "\"");
  enviarAT("AT+SAPBR=1,1");
  delay(3000);
  enviarAT("AT+SAPBR=2,1");
  Serial.println("GPRS listo");
  // iniciamos para llamadas
  enviarAT("AT+CLIP=1");
  enviarAT("AT+COLP=1");
  Serial.println("GPRS listo para llamadas");
}

// para mandar comandos cmd
void enviarAT(String cmd)
{
  sim.println(cmd);
  delay(1000);
  while (sim.available())
  {
    Serial.write(sim.read());
  }
}

// para esperar respuesta de sim800l
bool esperarRespuesta(String esperado, unsigned long timeout)
{
  unsigned long start = millis();
  String respuesta = "";
  while (millis() - start < timeout)
  {
    if (sim.available())
    {
      char c = sim.read();
      respuesta += c;
      if (respuesta.indexOf(esperado) != -1)
      {
        return true;
      }
    }
  }
  return false;
}