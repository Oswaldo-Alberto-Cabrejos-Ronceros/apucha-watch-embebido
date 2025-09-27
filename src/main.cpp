#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
#include <TinyGPSPlus.h>      // GPS
#include <HardwareSerial.h>   // UART para GPS y SIM800L
#include <DFRobotDFPlayerMini.h>

// configuraciones para pantalla
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MAX30105 particleSensor;       // Sensor de pulso
Adafruit_MPU6050 mpu;          // Acelerómetro/Giroscopio
TinyGPSPlus gps;               // gps
HardwareSerial SerialGPS(1);   // UART1 para GPS
                               // UART2 para GSM
SoftwareSerial sim800(16, 17); // verificar rx tx
// Pines botones
#define BTN1 0
#define BTN2 1
// buffers para el calculo de spo2 bpm
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup()
{
  // iniciar puerto serial para debug
  Serial.begin(115200);
  Serial.println("Iniciando sistema reloj...");
  // intentamos buscar la pantalla
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("No se encontró pantalla OLED");
    for (;;)
      ; // bloquear si no se encuentra pantalla
  }
  // configuracion inicar de la pantalla
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();

  // configuracion de botones
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  // configuracion del max 30102 - pulso
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD))
  {
    Serial.println("No se encontró MAX30102");
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

  // configuracion del gps
  SerialGPS.begin(9600, SERIAL_8N1, 4, 5); // rx tx
  Serial.println("GPS inicializado");
  display.println("GPS OK");
  display.display();

  // configuracion del gps
  SerialGSM.begin(9600, SERIAL_8N1, 6, 7); // rx tx
  Serial.println("SIM800L inicializado");
  display.println("GSM OK");
  display.display();
  // configuracion del dfplayer mini
  HardwareSerial SerialDF(1); // UART 1
  DFRobotDFPlayerMini dfplayer;
  SerialDF.begin(9600, SERIAL_8N1, 8, 9); // rx tx
  delay(1000);
  if (!dfplayer.begin(SerialDF))
  {
    Serial.println("DFPlayer no detectado!");
  }
  else
  {
    Serial.println("DFPlayer listo");
    dfplayer.volume(20); // volumen inicial
  }
  delay(2000);
  display.clearDisplay();
}
void loop()
{
  // leemos aceleracion
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float totalAcc = sqrt(pow(a.acceleration.x, 2) +
                        pow(a.acceleration.y, 2) +
                        pow(a.acceleration.z, 2));

  // leemos datos en crudo para sp02 y bpm
  for (int i = 0; i < BUFFER_SIZE; i++)
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
      irBuffer, BUFFER_SIZE, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate);

  // imprimimos para depurar
  Serial.print("BPM: ");
  Serial.print(heartRate);
  Serial.print(validHeartRate ? "valido" : "no valido");
  Serial.print(" | SpO2: ");
  Serial.print(spo2);
  Serial.println(validSPO2 ? "%" : " no valido");

  // deteccion de caida por rangos
  if (totalAcc < 2)
  {
    Serial.println("Posible caída libre detectada");
  }
  else if (totalAcc > 20)
  {
    Serial.println("Impacto detectado, enviando alerta...");
    enviarCaidaBackend(heartRate, spo2);
  }
  delay(1000);
  enviarVitalSignsBackend(heartRate, spo2)
}

// funcion para enviar

void enviarVitalSignsBackend(int bpm, int spo2)
{
  sim800.println("AT+HTTPTERM");
  delay(300);
  sim800.println("AT+HTTPINIT");
  delay(300);
  sim800.println("AT+HTTPPARA=\"URL\",\"http://apuchawatch.com/api/vital-signs\"");
  delay(300);
  sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(300);
  // json
  String json = "{\"bpm\":" + String(bpm) + ",\"spo2\":" + String(spo2) + "}";
  sim800.println("AT+HTTPDATA=" + String(json.length()) + ",10000");
  delay(500);
  sim800.println(json);
  delay(500);

  sim800.println("AT+HTTPACTION=1"); // peticion POST
  delay(6000);
  sim800.println("AT+HTTPREAD");
  delay(500);

  sim800.println("AT+HTTPTERM");
}

void enviarCaidaBackend(float totalAcc)
{
  sim800.println("AT+HTTPTERM");
  delay(300);
  sim800.println("AT+HTTPINIT");
  delay(300);
  sim800.println("AT+HTTPPARA=\"URL\",\"http://apuchawatch.com/api/vital-signs\"");
  delay(300);
  sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(300);
  // json
  String json = "{\"acc\":" + String(totalAcc) + "}";
  sim800.println("AT+HTTPDATA=" + String(json.length()) + ",10000");
  delay(500);
  sim800.println(json);
  delay(500);

  sim800.println("AT+HTTPACTION=1"); // peticion POST
  delay(6000);
  sim800.println("AT+HTTPREAD");
  delay(500);

  sim800.println("AT+HTTPTERM");
}