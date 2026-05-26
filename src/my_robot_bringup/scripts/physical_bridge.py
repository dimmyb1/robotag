import serial
import time
import json


class ArduinoBridge:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)

    def send_line(self, line: str):
        self.ser.write((line + "\n").encode("utf-8"))

    def set_motors(self, left: int, right: int):
        self.send_line(f"M,{left},{right}")

    def stop(self):
        self.send_line("STOP")

    def set_servo(self, angle: int):
        self.send_line(f"V,{angle}")

    def send_esp_packet(self, packet: dict):
        compact_json = json.dumps(packet, separators=(",", ":"))
        self.send_line(f"E,{compact_json}")

    def read_message(self):
        line = self.ser.readline().decode("utf-8", errors="replace").strip()

        if not line:
            return None

        if line.startswith("S,"):
            parts = line.split(",")
            if len(parts) != 7:
                return {"type": "bad_sensor_packet", "raw": line}

            return {
                "type": "sensors",
                "time_ms": int(parts[1]),
                "line_left": float(parts[2]),
                "line_middle": float(parts[3]),
                "line_right": float(parts[4]),
                "yaw_deg": float(parts[5]),
                "distance_cm": float(parts[6]),
                "raw": line,
            }

        if line.startswith("X,"):
            payload = line[2:]

            try:
                return {
                    "type": "esp32_packet",
                    "payload": json.loads(payload),
                    "raw_payload": payload,
                    "raw": line,
                }
            except json.JSONDecodeError:
                return {
                    "type": "esp32_packet",
                    "payload": payload,
                    "raw_payload": payload,
                    "raw": line,
                }

        if line.startswith("OK,"):
            return {"type": "ack", "raw": line}

        if line.startswith("ERR,"):
            return {"type": "error", "raw": line}

        return {"type": "unknown", "raw": line}


if __name__ == "__main__":
    arduino = ArduinoBridge("/dev/ttyUSB0", 115200)

    while True:
        msg = arduino.read_message()

        if msg is None:
            continue

        print(msg)

        if msg["type"] == "esp32_packet":
            print("Received from other robot:", msg["payload"])

        if msg["type"] == "sensors":
            print("Distance:", msg["distance_cm"], "cm")