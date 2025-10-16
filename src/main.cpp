#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
// #include <HardwareSerial.h>   // UART para GPS y SIM800L
#include "spo2_algorithm.h"
#include <WiFi.h>

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
String deviceId;

const char *ssid = "OSWALDO";      // Ejemplo: "MiRed"
const char *password = "12345678"; // Ejemplo: "12345678"

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

  Serial.println("Iniciando sistema reloj...");
  Wire.begin(8, 9, 100000);
  Serial.println("\nEscaneo I2C iniciado...");

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
  // para chip id
  uint64_t chipid = ESP.getEfuseMac();
  char idStr[13];
  sprintf(idStr, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  deviceId = String(idStr);
  Serial.println("Device ID: " + deviceId);
  display.println("ID: " + deviceId);
  display.display();
  /*
    // configurar rangos



  display.clearDisplay();*/
  delay(2000);
}
void loop()
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

    delay(500); // espera entre lecturas
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
  }
  else
  {
    Serial.println("Lecturas no válidas, intentar de nuevo");
  }

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
  /*
    // deteccion de caida por rangos
    if (totalAcc < 2)
    {
      Serial.println("Posible caída libre detectada");
    }
    else if (totalAcc > 20)
    {
      Serial.println("Impacto detectado, enviando alerta...");
    }
    delay(1000);

    display.display();*/
  Serial.println("Hola mundo");
  display.println("Hola mundo");
  display.display();
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

  delay(500);
}

void enviardatosBackend()
{
}