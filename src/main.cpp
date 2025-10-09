#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "MAX30105.h"         // Para MAX30102 (usa misma librería)
#include "Adafruit_MPU6050.h" // Librería MPU6050
//#include <HardwareSerial.h>   // UART para GPS y SIM800L
#include "spo2_algorithm.h"
// configuraciones para pantalla
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


 
void setup()
{
  // iniciar puerto serial para debug
  Serial.begin(115200);
  Serial.println("Iniciando sistema reloj...");
}
void loop()
{

}
