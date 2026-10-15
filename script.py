import argparse
import json
import random
import time
from datetime import datetime

import paho.mqtt.client as mqtt

# ---------------- CONFIGURACIÓN ----------------
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SIGNOS = "iot/reloj/signos"
TOPIC_CAIDAS = "iot/reloj/caidas"
# ------------------------------------------------


def to_base36(value: int) -> str:
    """Igual que toBase36() del firmware."""
    digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    if value == 0:
        return "0"
    result = ""
    while value > 0:
        result = digits[value % 36] + result
        value //= 36
    return result


DEVICE_CODE = to_base36(0xA1B2C3D4)


class RelojSimulado:
    def __init__(self):
        self.bpm = 75.0
        self.spo2 = 97.0

    def signos(self):
        self.bpm = min(max(self.bpm + random.uniform(-3, 3), 55.0), 120.0)
        self.spo2 = min(max(self.spo2 + random.uniform(-0.7, 0.7), 91.0), 100.0)
        # En reposo la aceleración total del MPU6050 es ~9.8 m/s^2
        acc = 9.81 + random.uniform(-0.6, 0.6)
        return {
            "deviceCode": DEVICE_CODE,
            "heartRate": int(round(self.bpm)),          
            "oxygenSaturation": int(round(self.spo2)),  
            "aceleracion": round(acc, 2),               
        }

    def caida(self):
        if random.random() < 0.5:
            evento, acc = "Caida Libre", random.uniform(0.2, 1.9) 
        else:
            evento, acc = "Impacto", random.uniform(18.5, 30.0)
        return {
            "deviceCode": DEVICE_CODE,
            "evento": evento,
            "aceleracion": round(acc, 2),
        }


def conectar():
    client_id = f"ESP32Mini-{DEVICE_CODE}"
    try:
        cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError: 
        cli = mqtt.Client(client_id=client_id)

    def on_connect(client, userdata, flags, rc, properties=None):
        print(f"[MQTT] Conectado a EMQX ({MQTT_HOST}:{MQTT_PORT}) -> {rc}")

    cli.on_connect = on_connect
    cli.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    cli.loop_start()
    return cli


def publicar(cli, topic, datos):
    info = cli.publish(topic, json.dumps(datos))
    info.wait_for_publish(timeout=3)
    ok = info.is_published()
    hora = datetime.now().strftime("%H:%M:%S")
    estado = "enviado OK" if ok else "NO se pudo enviar"
    print(f"[{hora}] {topic} {json.dumps(datos)}  -> {estado}")


def main():
    ap = argparse.ArgumentParser(description="Simulador ESP32 reloj de salud")
    ap.add_argument("--intervalo", type=float, default=10.0,
                    help="segundos entre publicaciones (el firmware usa 10)")
    ap.add_argument("--prob-caida", type=float, default=0.15,
                    help="probabilidad (0-1) de simular una caída en cada ciclo")
    args = ap.parse_args()

    cli = conectar()
    reloj = RelojSimulado()
    time.sleep(1)
    print(f"Simulando ESP32 {DEVICE_CODE}... (Ctrl+C para detener)\n")

    try:
        while True:
            publicar(cli, TOPIC_SIGNOS, reloj.signos())
            if random.random() < args.prob_caida:
                publicar(cli, TOPIC_CAIDAS, reloj.caida())
            time.sleep(args.intervalo)
    except KeyboardInterrupt:
        print("\nDetenido.")
    finally:
        cli.loop_stop()
        cli.disconnect()


if __name__ == "__main__":
    main()