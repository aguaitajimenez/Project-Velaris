import socket
import json
import time
import numpy as np
import csv
from datetime import datetime
import os
from collections import deque

# Server configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 8000
TIMEOUT_SECONDS = 10
MAX_RSSI_HISTORY = 8

latest_gps = None  # Optional


# CSV setup
csv_filename = "udp_log.csv"
file_exists = os.path.isfile(csv_filename)
csv_file = open(csv_filename, mode='a', newline='')
csv_writer = csv.writer(csv_file)

if not file_exists:
    csv_writer.writerow(["timestamp", "id", "rssi", "avg_rssi", "distance", "temperature", "bpm",
                         "accel_x", "accel_y", "accel_z",
                         "wrist_battV", "wrist_battP",
                         "my_battV", "my_battP"])

# UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"✅ UDP Server listening on port {UDP_PORT}...")

# Beacon data
rssi_history = {}        # { beacon_id: deque([rssi1, rssi2, ...]) }
last_seen_time = {}      # { beacon_id: timestamp }

# Subscribers
subscribers = set()

# Alert state with hysteresis
alert_state = 0  # 0 = safe, 1 = alert

# Latest sensor snapshot
latest_data = {
    "type": None,
    "temperature": None,
    "bpm": None,
    "accel_x": None,
    "accel_y": None,
    "accel_z": None,
    "wrist_battV": None,
    "wrist_battP": None,
    "my_battV": None,
    "my_battP": None,
    # NEW: GPS fields
    "gps_fix": None,
    "gps_lat": None,
    "gps_lat_dir": None,
    "gps_lon": None,
    "gps_lon_dir": None,
    "gps_alt": None,
    "gps_sats": None
}


while True:
    data, addr = sock.recvfrom(1024)
    message = data.decode()

    try:
        parsed = json.loads(message)

        # Handle subscribe/unsubscribe
        if isinstance(parsed, dict) and "command" in parsed:
            print("Received command from", addr, data)
            command = parsed["command"]
            if command == "subscribe":
                subscribers.add(addr)
                print(f"✅ {addr} subscribed.")
            elif command == "unsubscribe":
                print(f"❌ {addr} unsubscribed.")
                subscribers.discard(addr)
            continue

        timestamp = datetime.now().isoformat()
        id_ = parsed.get("id")

        # 🌐 Determine packet type
        if "tmp" in parsed and "rs" in parsed:
            print("📦 Received BLE Packet")
            print(f"   ID: {id_}")
            print(f"   RSSI: {parsed['rs']}")
            print(f"   Temperature: {parsed['tmp']}")
            print(f"   BPM: {parsed.get('bpm')}")
            acc = parsed.get("acc", {})
            print(f"   Acceleration: x={acc.get('x')}, y={acc.get('y')}, z={acc.get('z')}")
            print(f"   Wrist Battery → V: {parsed.get('wbv')}, P: {parsed.get('wbp')}")
            print(f"   Beacon Battery → V: {parsed.get('mbv')}, P: {parsed.get('mbp')}")

            # RSSI history and CSV logging
            rssi = parsed.get("rs")
            if id_ is not None and rssi is not None:
                if id_ not in rssi_history:
                    rssi_history[id_] = deque(maxlen=MAX_RSSI_HISTORY)
                rssi_history[id_].append(rssi)
                last_seen_time[id_] = time.time()

                avg_rssi = sum(rssi_history[id_]) / len(rssi_history[id_])
                distance = 10 ** ((-8 - avg_rssi - 20 * np.log10(2400) + 28) / 28)

                print(f"📡 Beacon '{id_}' Avg RSSI = {avg_rssi:.2f} dBm → Distance ≈ {distance:.2f} m")

                csv_writer.writerow([timestamp, id_, rssi, avg_rssi, distance, parsed.get("tmp"), parsed.get("bpm"),
                                     acc.get("x"), acc.get("y"), acc.get("z"),
                                     parsed.get("wbv"), parsed.get("wbp"),
                                     parsed.get("mbv"), parsed.get("mbp")])

                # Update latest data
                latest_data.update({
                    "type": "ble",
                    "temperature": parsed.get("tmp"),
                    "bpm": parsed.get("bpm"),
                    "accel_x": acc.get("x"),
                    "accel_y": acc.get("y"),
                    "accel_z": acc.get("z"),
                    "wrist_battV": parsed.get("wbv"),
                    "wrist_battP": parsed.get("wbp"),
                    "my_battV": parsed.get("mbv"),
                    "my_battP": parsed.get("mbp")
                })

        elif "g" in parsed:
            print("📦 Received LoRa Packet")
            gps = parsed["g"]
            print(f"   ID: {id_}")
            print(f"   Wrist Battery → V: {parsed.get('wbv')}, P: {parsed.get('wbp')}")
            print(f"   Beacon Battery → V: {parsed.get('mbv')}, P: {parsed.get('mbp')}")
            print(f"   GPS Fix: {gps.get('f')}")
            print(f"   Latitude: {gps.get('la')} {gps.get('ld')}")
            print(f"   Longitude: {gps.get('lo')} {gps.get('lod')}")
            print(f"   Altitude: {gps.get('a')}")
            print(f"   Satellites: {gps.get('s')}")

            # Update latest_data with GPS and battery info
            latest_data.update({
                "type": "lora",
                "my_battV": float(parsed.get("mbv", 0)),
                "my_battP": float(parsed.get("mbp", 0)),
                "wrist_battV": float(parsed.get("wbv", 0)),
                "wrist_battP": float(parsed.get("wbp", 0)),
                "gps_fix": gps.get("f"),
                "gps_lat": gps.get("la"),
                "gps_lat_dir": gps.get("ld"),
                "gps_lon": gps.get("lo"),
                "gps_lon_dir": gps.get("lod"),
                "gps_alt": gps.get("a"),
                "gps_sats": gps.get("s")
            })



            # Save latest_gps if needed elsewhere
            latest_gps = gps


        else:
            print("⚠️ Unknown packet format.")

        # Hysteresis and beacon tracking
        # Hysteresis and beacon tracking
        now = time.time()
        closest_id = None
        closest_rssi = None
        closest_distance = None

        if rssi_history:
            avg_rssi_dict = {bid: sum(dq) / len(dq) for bid, dq in rssi_history.items()}
            closest_id = max(avg_rssi_dict, key=avg_rssi_dict.get)
            closest_rssi = avg_rssi_dict[closest_id]
            closest_distance = 10 ** ((-8 - closest_rssi - 20 * np.log10(2400) + 28) / 28)

            if alert_state == 0 and closest_distance > 6:
                alert_state = 1
            elif alert_state == 1 and closest_distance < 4:
                alert_state = 0

            print(f"   📶 Closest Beacon: {closest_id}")
            print(f"   RSSI: {closest_rssi:.2f} dBm")
            print(f"   Estimated Distance: {closest_distance:.2f} meters")
            print(f"   🚨 Alert: {alert_state}")
        else:
            print("   📶 No active beacons.")

        # Send to subscribers (even if no RSSI data)
        if subscribers:
            try:
                if latest_data["type"] == "ble":
                    response_payload = {
                        "type": latest_data["type"],
                        "temperature": latest_data["temperature"],
                        "bpm": latest_data["bpm"],
                        "acceleration": {
                            "x": latest_data["accel_x"],
                            "y": latest_data["accel_y"],
                            "z": latest_data["accel_z"]
                        },
                        "wrist_batt": {
                            "V": latest_data["wrist_battV"],
                            "P": latest_data["wrist_battP"]
                        },
                        "beacon_batt": {
                            "V": latest_data["my_battV"],
                            "P": latest_data["my_battP"]
                        },
                        "closest_beacon": closest_id,
                        "avg_rssi": closest_rssi,
                        "estimated_distance": closest_distance,
                        "alert": alert_state
                    }
                elif latest_data["type"] == "lora":
                    response_payload = {
                        "type": latest_data["type"],
                        "gps": {
                            "fix": latest_data["gps_fix"],
                            "lat": latest_data["gps_lat"],
                            "lat_dir": latest_data["gps_lat_dir"],
                            "lon": latest_data["gps_lon"],
                            "lon_dir": latest_data["gps_lon_dir"],
                            "alt": latest_data["gps_alt"],
                            "sats": latest_data["gps_sats"]
                        },
                        "wrist_batt": {
                            "V": latest_data["wrist_battV"],
                            "P": latest_data["wrist_battP"]
                        },
                        "beacon_batt": {
                            "V": latest_data["my_battV"],
                            "P": latest_data["my_battP"]
                        },
                        "closest_beacon": closest_id,
                        "avg_rssi": closest_rssi,
                        "estimated_distance": closest_distance,
                        "alert": alert_state
                    }
                else:
                    response_payload = {"error": "unknown data type"}

                response_json = json.dumps(response_payload).encode()
                # print("Sending to subscribers:", response_payload)

                for sub_addr in list(subscribers):
                    sock.sendto(response_json, sub_addr)

            except Exception as e:
                print(f"⚠️ Failed to send update to subscribers: {e}")


        # Remove stale beacons
        expired_ids = [bid for bid, t in last_seen_time.items() if now - t > TIMEOUT_SECONDS]
        for bid in expired_ids:
            print(f"⏳ Beacon '{bid}' removed due to timeout.")
            rssi_history.pop(bid, None)
            last_seen_time.pop(bid, None)

    except json.JSONDecodeError:
        print("⚠️ Received non-JSON data.")
    except Exception as e:
        print(f"⚠️ Error: {e}")
