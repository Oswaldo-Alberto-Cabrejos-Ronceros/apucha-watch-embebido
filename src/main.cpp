#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050.h>

MPU6050 mpu;

// configuraciones para pantalla

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup()
{
  // iniciar puerto serial para debug
  delay(5000);
  Serial.begin(115200);

  Wire.begin(8, 9, 100000);

  // configuracion del mpu--acelerometro-giroscopio
  mpu.initialize();

  if (!mpu.testConnection())
  {
    Serial.println("Error: no se detecta MPU6050");
    while (1)
      ;
  }
  Serial.println("Recolectando datos (ax, ay, az, gx, gy, gz)");

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
}
void loop()
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // normalizamos los valores entre -1 y 1
  float fax = ax / 16384.0;
  float fay = ay / 16384.0;
  float faz = az / 16384.0;
  float fgx = gx / 131.0;
  float fgy = gy / 131.0;
  float fgz = gz / 131.0;

  Serial.print(fax);
  Serial.print(",");
  Serial.print(fay);
  Serial.print(",");
  Serial.print(faz);
  Serial.print(",");
  Serial.print(fgx);
  Serial.print(",");
  Serial.print(fgy);
  Serial.print(",");
  Serial.println(fgz);

  delay(50);
}
