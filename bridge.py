import json
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

INFLUX_URL = "http://localhost:8086"
TOKEN = "miSuperToken123456789"
ORG = "utp"
BUCKET = "iot_temperatura"

client_influx = InfluxDBClient(url=INFLUX_URL, token=TOKEN, org=ORG)
write_api = client_influx.write_api(write_options=SYNCHRONOUS)

def on_connect(client, userdata, flags, rc):
    print("Conectado a EMQX. Suscrito a los tópicos del reloj...")
    client.subscribe("iot/reloj/+")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        topic = msg.topic

        if topic == "iot/reloj/signos":
            point = (
                Point("sensores_salud")
                .tag("deviceCode", data.get("deviceCode", "ESP32"))
                .field("heartRate", float(data.get("heartRate", 0)))
                .field("oxygenSaturation", float(data.get("oxygenSaturation", 0)))
                .field("aceleracion", float(data.get("aceleracion", 0.0)))
            )
            write_api.write(bucket=BUCKET, org=ORG, record=point)
            print(f"Insertado en InfluxDB [Signos]: BPM={data.get('heartRate')}, SpO2={data.get('oxygenSaturation')}%")

        elif topic == "iot/reloj/caidas":
            point = (
                Point("alertas_caida")
                .tag("deviceCode", data.get("deviceCode", "ESP32"))
                .tag("evento", data.get("evento", "Alerta"))
                .field("aceleracion_impacto", float(data.get("aceleracion", 0.0)))
            )
            write_api.write(bucket=BUCKET, org=ORG, record=point)
            print(f"Insertado en InfluxDB [Alerta]: {data.get('evento')}")

    except Exception as e:
        print(f"Error parseando JSON: {e}")

client_mqtt = mqtt.Client()
client_mqtt.on_connect = on_connect
client_mqtt.on_message = on_message

client_mqtt.connect("localhost", 1883, 60)
client_mqtt.loop_forever()