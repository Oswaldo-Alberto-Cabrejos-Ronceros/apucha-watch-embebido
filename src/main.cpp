#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
// #include <HardwareSerial.h>   // UART para GPS y SIM800L
#include "spo2_algorithm.h"
// configuraciones para pantalla
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105 particleSensor;     // Sensor de pulso
Adafruit_MPU6050 mpu;        // Acelerómetro/Giroscopio

// buffers para el calculo de spo2 bpm
#define MY_BUFFER_SIZE 100
uint32_t irBuffer[MY_BUFFER_SIZE];
uint32_t redBuffer[MY_BUFFER_SIZE];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;
//variables de giro y aceleracion
float ax, ay, az;
String deviceId;
void setup()
{
  // iniciar puerto serial para debug
  Serial.begin(115200);
  Serial.println("Iniciando sistema reloj...");
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

  // configuracion del max 30102 - pulso
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD))
  {
    Serial.println("No se encontró MAX30102");
    display.println("MAX30102 ERROR");
  }
  else
  {
    Serial.println("MAX30102 OK");
    display.println("MAX30102 OK");
    display.display();
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
  display.println("MPU6050 OK");
  display.display();

  // configurar rangos
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // para chip id
  uint64_t chipid = ESP.getEfuseMac();
  char idStr[13];
  sprintf(idStr, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  deviceId = String(idStr);
  Serial.println("Device ID: " + deviceId);
  display.println("ID: " + deviceId);

  delay(2000);
  display.clearDisplay();
}
void loop()
{
  // limpiamos pantalla
  display.clearDisplay();
  display.setCursor(0, 0);

  // leemos aceleracion
  sensors_event_t a,
      g, temp;
  mpu.getEvent(&a, &g, &temp);
  float totalAcc = sqrt(pow(a.acceleration.x, 2) +
                        pow(a.acceleration.y, 2) +
                        pow(a.acceleration.z, 2));
  Serial.print("Aceleracion total: ");
  Serial.print(totalAcc);
  Serial.print(" m/s^2 | ");
  display.print("Acel: ");
  display.print(totalAcc);
  // leemos datos en crudo para sp02 y bpm
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

  // algoritmo proporcionado por sparkfun
  //  Algoritmo de SparkFun
  maxim_heart_rate_and_oxygen_saturation(
      irBuffer, MY_BUFFER_SIZE, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate);

  // imprimimos para depurar
  Serial.print("BPM: ");
  Serial.print(heartRate);
  display.print("BPM: ");
  display.print(heartRate);
  Serial.print(validHeartRate ? "valido" : "no valido");
  Serial.print(" | SpO2: ");
  Serial.print(spo2);
  display.print(" | SpO2: ");
  display.print(spo2);
  Serial.println(validSPO2 ? "%" : " no valido");

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

  display.display();

  delay(500);
}
