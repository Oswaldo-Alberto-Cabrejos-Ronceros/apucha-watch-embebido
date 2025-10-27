#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

MPU6050 mpu;

// configuraciones para pantalla

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char *ssid = "OSWALDO";
const char *password = "12345678";
// para configurar hora
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -5 * 3600; // horario peru
const int daylightOffset_sec = 0;

// configuracion para websocket
WebSocketsClient webSocket;
const char *websocket_server = "192.168.1.100"; // ip del backend en la red local
const uint16_t websocket_port = 8080;
const char *websocket_path = "/"; // ruta del websocket

// menejador de los mesajes de websocket

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    Serial.println("Conectado al WebSocket ");
    break;
  case WStype_DISCONNECTED:
    Serial.println("Desconectado del WebSocket");
    break;
  case WStype_TEXT:
    Serial.printf("Mensaje recibido: %s\n", payload);
    break;
  }
}

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
  // conectar wifi
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

  // configuracion inicar de la pantalla
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");

  // conexion al websocket
  webSocket.begin(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000); // reintentar cada 5 segundos
}
void loop()
{
  webSocket.loop();

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

  // enviamos datos cada medio segundo
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 500)
  {
    lastSend = millis();

    String json = "{\"ax\":" + String(fax) +
                  ",\"ay\":" + String(fay) +
                  ",\"az\":" + String(faz) +
                  ",\"gx\":" + String(fgx) +
                  ",\"gy\":" + String(fgy) +
                  ",\"gz\":" + String(fgz) + "}";
    webSocket.sendTXT(json);
    Serial.println("Datos enviados");
  }

  delay(50);
}
