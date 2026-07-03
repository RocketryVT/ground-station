#!/usr/bin/env python3
"""Forward secondary Pico USB altitude lines to the primary Pico.

Secondary output format:
    ALT,<alt_m>,<boot_ms>,<rssi>,<snr>

Primary accepts the same line on its USB console.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import json
import re
import select
import socket
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Install pyserial first: python3 -m pip install pyserial") from exc


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        desc = port.description or "unknown"
        hwid = port.hwid or ""
        print(f"{port.device}\t{desc}\t{hwid}")


def open_port(path: str, baud: int) -> serial.Serial:
    return serial.Serial(path, baudrate=baud, timeout=0, write_timeout=0.25)


def decode_line(raw: bytes) -> str:
    return raw.decode("utf-8", errors="replace").strip()


def write_line(port: serial.Serial, text: str) -> None:
    port.write((text.rstrip("\r\n") + "\n").encode("ascii", errors="replace"))
    port.flush()


def read_complete_lines(port: serial.Serial, buffer: bytearray) -> list[str]:
    data = port.read(port.in_waiting or 1)
    if data:
        buffer.extend(data)

    lines: list[str] = []
    while True:
        newline_positions = [pos for pos in (buffer.find(b"\n"), buffer.find(b"\r")) if pos >= 0]
        if not newline_positions:
            break
        pos = min(newline_positions)
        raw = bytes(buffer[:pos])
        del buffer[:pos + 1]
        while buffer[:1] in (b"\n", b"\r"):
            del buffer[:1]
        text = decode_line(raw)
        if text:
            lines.append(text)
    return lines


def mqtt_remaining_length(length: int) -> bytes:
    out = bytearray()
    while True:
        encoded = length % 128
        length //= 128
        if length:
            encoded |= 0x80
        out.append(encoded)
        if not length:
            return bytes(out)


def mqtt_utf8(text: str) -> bytes:
    data = text.encode("utf-8")
    return len(data).to_bytes(2, "big") + data


def mqtt_publish(host: str, port: int, topic: str, payload: str | bytes) -> None:
    client_id = f"usb-alt-bridge-{int(time.time())}"
    variable = mqtt_utf8("MQTT") + bytes([4, 2]) + (30).to_bytes(2, "big")
    packet = variable + mqtt_utf8(client_id)
    connect = bytes([0x10]) + mqtt_remaining_length(len(packet)) + packet

    topic_bytes = mqtt_utf8(topic)
    payload_bytes = payload if isinstance(payload, bytes) else payload.encode("utf-8")
    publish_packet = topic_bytes + payload_bytes
    publish = bytes([0x30]) + mqtt_remaining_length(len(publish_packet)) + publish_packet

    with socket.create_connection((host, port), timeout=0.5) as sock:
        sock.sendall(connect)
        ack = sock.recv(4)
        if len(ack) < 4 or ack[0] != 0x20 or ack[3] != 0:
            raise OSError(f"MQTT CONNACK failed: {ack!r}")
        sock.sendall(publish)
        sock.sendall(bytes([0xE0, 0x00]))


def parse_alt_line(text: str) -> tuple[float, int, float | None, float | None] | None:
    if not text.startswith("ALT,"):
        return None
    parts = text.split(",")
    if len(parts) < 2:
        return None
    alt_m = float(parts[1])
    boot_ms = int(float(parts[2])) if len(parts) > 2 and parts[2] else 0
    rssi = float(parts[3]) if len(parts) > 3 and parts[3] else None
    snr = float(parts[4]) if len(parts) > 4 and parts[4] else None
    return alt_m, boot_ms, rssi, snr


def publish_altitude_to_gui(host: str, port: int, text: str, lat: float, lon: float) -> None:
    parsed = parse_alt_line(text)
    if not parsed:
        return
    alt_m, boot_ms, rssi, snr = parsed
    payload = {
        "timestamp": int(time.time() * 1000),
        "boot_ms": boot_ms,
        "lat": lat,
        "lon": lon,
        "alt_m": alt_m,
        "alt_baro_m": alt_m,
        "alt_baro": alt_m,
        "vel_n": 0,
        "vel_e": 0,
        "vel_d": 0,
        "roll": 0,
        "pitch": 0,
        "yaw": 0,
        "state": "BARO_ONLY",
    }
    payload["rssi"] = rssi if rssi is not None else 0
    payload["snr"] = snr if snr is not None else 0
    mqtt_publish(host, port, "rocket/telemetry", json.dumps(payload, separators=(",", ":")))


def radio_status_from_log(source: str, text: str) -> dict[str, object] | None:
    match = re.match(r"\[(lora[0-9])\]\s+(.*)", text)
    if not match:
        return None

    tag, message = match.groups()
    radios = {
        ("primary", "lora0"): ("primary-915", "Primary 915", "SX1276", 915.0),
        ("primary", "lora1"): ("primary-433", "Primary 433", "RF69", 424.5),
        ("secondary", "lora1"): ("secondary-915", "Secondary 915", "SX1276", 915.0),
        ("secondary", "lora2"): ("secondary-433", "Secondary 433", "RF69", 424.5),
    }
    if (source, tag) not in radios:
        return None

    radio_id, label, radio, freq_mhz = radios[(source, tag)]
    status: dict[str, object] = {
        "id": radio_id,
        "label": label,
        "board": source,
        "radio": radio,
        "freq_mhz": freq_mhz,
        "message": message,
    }

    lower = message.lower()
    if "init failed" in lower:
        status["state"] = "init_failed"
    elif "bad frame" in lower:
        status["state"] = "bad_frame"
    elif "ready" in lower or "diag:" in lower:
        status["state"] = "ready"
    elif "rx" in lower or "sigma" in lower:
        status["state"] = "rx"
    else:
        return None

    if "gps" in lower or "nav" in lower or "lat=" in lower:
        status["has_gps"] = True
    if "baro" in lower or "nav" in lower:
        status["has_baro"] = True
    if "gps" in lower or "alt_gps" in lower:
        status["has_gps_alt"] = True

    patterns = {
        "lat": r"lat=([-+]?\d+(?:\.\d+)?)",
        "lon": r"lon=([-+]?\d+(?:\.\d+)?)",
        "alt_baro_m": r"(?:baro|alt)=([-+]?\d+(?:\.\d+)?)\s*m",
        "alt_gps_m": r"alt_gps=([-+]?\d+(?:\.\d+)?)\s*m",
        "rssi": r"RSSI[ =]([-+]?\d+(?:\.\d+)?)",
        "snr": r"SNR[ =]([-+]?\d+(?:\.\d+)?)",
        "len": r"rx\s+(\d+)\s+B",
    }
    for key, pattern in patterns.items():
        found = re.search(pattern, message)
        if not found:
            continue
        value = float(found.group(1))
        status[key] = int(value) if key == "len" else value

    if status.get("has_gps") and not status.get("has_baro") and "alt_baro_m" in status:
        status["alt_gps_m"] = status.pop("alt_baro_m")

    return status


def publish_radio_status_to_gui(host: str, port: int, source: str, text: str) -> bool:
    status = radio_status_from_log(source, text)
    if not status:
        return False
    mqtt_publish(host, port, "gs/radio/status", json.dumps(status, separators=(",", ":")))
    return True


def parse_mqttb64_line(text: str) -> tuple[str, bytes] | None:
    if not text.startswith("MQTTB64,"):
        return None
    parts = text.split(",", 3)
    if len(parts) != 4:
        return None
    _, topic, length_text, encoded = parts
    expected_len = int(length_text)
    payload = base64.b64decode(encoded, validate=True)
    if len(payload) != expected_len:
        raise ValueError(f"MQTTB64 length mismatch for {topic}: got {len(payload)}, expected {expected_len}")
    return topic, payload


def is_primary_noise(text: str) -> bool:
    noisy_prefixes = (
        "[imu] WHO_AM_I read failed",
        "[imu] init retry",
        "[mag] LIS3MDL init failed",
        "[mag] init retry",
    )
    return any(text.startswith(prefix) for prefix in noisy_prefixes)


def bridge(
    secondary: str,
    primary: str,
    baud: int,
    mqtt_host: str,
    mqtt_port: int,
    mqtt_enabled: bool,
    show_primary_noise: bool,
    lat: float,
    lon: float,
) -> int:
    with open_port(secondary, baud) as sec, open_port(primary, baud) as pri:
        sec_buffer = bytearray()
        pri_buffer = bytearray()
        mqttb64_count = 0
        print(f"Forwarding ALT lines: {secondary} -> {primary} @ {baud}")
        if mqtt_enabled:
            print(f"Publishing GUI telemetry to MQTT {mqtt_host}:{mqtt_port}")
        else:
            print("MQTT GUI publishing disabled.")
        print("Primary console is attached here too.")
        print("Type primary commands like: status, base 880 1000, arm, auto")
        print("Bridge commands: /quit")
        print("Press Ctrl-C to stop.")
        while True:
            readable, _, _ = select.select([sec, pri, sys.stdin], [], [], 0.1)

            if sec in readable:
                for text in read_complete_lines(sec, sec_buffer):
                    print(f"secondary: {text}")
                    if mqtt_enabled:
                        try:
                            publish_radio_status_to_gui(mqtt_host, mqtt_port, "secondary", text)
                        except OSError as error:
                            print(f"gui: MQTT radio status failed: {error}")
                    if text.startswith("ALT,"):
                        write_line(pri, text)
                        print(f"forwarded: {text}")
                        if mqtt_enabled:
                            try:
                                publish_altitude_to_gui(mqtt_host, mqtt_port, text, lat, lon)
                                print("gui: published rocket/telemetry")
                            except OSError as error:
                                print(f"gui: MQTT publish failed: {error}")

            if pri in readable:
                for text in read_complete_lines(pri, pri_buffer):
                    mqttb64 = None
                    try:
                        mqttb64 = parse_mqttb64_line(text)
                    except (ValueError, binascii.Error) as error:
                        print(f"primary: bad MQTTB64 frame: {error}")

                    if mqttb64 is not None:
                        topic, payload = mqttb64
                        if mqtt_enabled:
                            try:
                                mqtt_publish(mqtt_host, mqtt_port, topic, payload)
                                mqttb64_count += 1
                                if show_primary_noise or (mqttb64_count % 50) == 1:
                                    print(f"gui: published {topic} ({len(payload)} B)")
                            except OSError as error:
                                print(f"gui: MQTT publish failed on {topic}: {error}")
                        continue

                    if show_primary_noise or not is_primary_noise(text):
                        print(f"primary: {text}")
                    if mqtt_enabled:
                        try:
                            publish_radio_status_to_gui(mqtt_host, mqtt_port, "primary", text)
                        except OSError as error:
                            print(f"gui: MQTT radio status failed: {error}")

            if sys.stdin in readable:
                line = sys.stdin.readline()
                if line == "":
                    continue
                text = line.strip()
                if text == "/quit":
                    return 0
                if text:
                    write_line(pri, text)
                    if text.startswith("ALT,") and mqtt_enabled:
                        try:
                            publish_altitude_to_gui(mqtt_host, mqtt_port, text, lat, lon)
                            print("gui: published rocket/telemetry")
                        except OSError as error:
                            print(f"gui: MQTT publish failed: {error}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--secondary", help="secondary Pico serial port")
    parser.add_argument("--primary", help="primary Pico serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mqtt-host", default="localhost")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--no-mqtt", action="store_true", help="do not publish relayed altitude to GUI MQTT")
    parser.add_argument("--show-primary-noise", action="store_true", help="show repeated primary IMU init retry logs")
    parser.add_argument("--lat", type=float, default=0.0, help="GUI placeholder latitude for baro-only telemetry")
    parser.add_argument("--lon", type=float, default=0.0, help="GUI placeholder longitude for baro-only telemetry")
    args = parser.parse_args(argv)

    if args.list:
        list_serial_ports()
        return 0

    if not args.secondary or not args.primary:
        parser.error("--secondary and --primary are required unless --list is used")

    return bridge(
        args.secondary,
        args.primary,
        args.baud,
        args.mqtt_host,
        args.mqtt_port,
        not args.no_mqtt,
        args.show_primary_noise,
        args.lat,
        args.lon,
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
