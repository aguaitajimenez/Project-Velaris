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
    "temperature": None,
    "bpm": None,
    "accel_x": None,
    "accel_y": None,
    "accel_z": None,
    "wrist_battV": None,
    "wrist_battP": None,
    "my_battV": None,
    "my_battP": None
}

while True:
    data, addr = sock.recvfrom(1024)
    message = data.decode()

    try:
        parsed = json.loads(message)

        # Handle subscribe/unsubscribe
        if isinstance(parsed, dict) and "command" in parsed:
            command = parsed["command"]
            if command == "subscribe":
                subscribers.add(addr)
                print(f"✅ {addr} subscribed.")
            elif command == "unsubscribe":
                subscribers.discard(addr)
                print(f"❌ {addr} unsubscribed.")
            continue

        # Extract base values
        timestamp = datetime.now().isoformat()
        id_ = parsed.get("id")
        rssi = parsed.get("rssi")
        temperature = parsed.get("temperature")
        bpm = parsed.get("bpm")

        accel = parsed.get("acceleration", {})
        accel_x = accel.get("x")
        accel_y = accel.get("y")
        accel_z = accel.get("z")

        wrist_batt = parsed.get("wrist_batt", {})
        wrist_battV = wrist_batt.get("wrist_battV")
        wrist_battP = wrist_batt.get("wrist_battP")

        my_batt = parsed.get("my_batt", {})
        my_battV = my_batt.get("my_battV")
        my_battP = my_batt.get("my_battP")

        # Maintain history of RSSI
        if id_ is not None and rssi is not None:
            if id_ not in rssi_history:
                rssi_history[id_] = deque(maxlen=MAX_RSSI_HISTORY)
            rssi_history[id_].append(rssi)
            last_seen_time[id_] = time.time()

            # Compute average RSSI and distance
            avg_rssi = sum(rssi_history[id_]) / len(rssi_history[id_])
            distance = 10 ** ((-8 - avg_rssi - 20 * np.log10(2400) + 28) / 28)

            print(f"📡 Beacon '{id_}' Avg RSSI = {avg_rssi:.2f} dBm → Distance ≈ {distance:.2f} m")

            # Log to CSV
            csv_writer.writerow([timestamp, id_, rssi, avg_rssi, distance, temperature, bpm,
                                 accel_x, accel_y, accel_z,
                                 wrist_battV, wrist_battP,
                                 my_battV, my_battP])

        # Update latest sensor info
        latest_data.update({
            "temperature": temperature,
            "bpm": bpm,
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "wrist_battV": wrist_battV,
            "wrist_battP": wrist_battP,
            "my_battV": my_battV,
            "my_battP": my_battP
        })

        # Print latest sensor info
        print("\n🧾 Latest Sensor Status")
        print(f"   Temperature: {latest_data['temperature']} °C")
        print(f"   BPM: {latest_data['bpm']}")
        print(f"   Acceleration → x: {latest_data['accel_x']}, y: {latest_data['accel_y']}, z: {latest_data['accel_z']}")
        print(f"   Wrist Battery → V: {latest_data['wrist_battV']}, P: {latest_data['wrist_battP']}")
        print(f"   Beacon Battery → V: {latest_data['my_battV']}, P: {latest_data['my_battP']}")

        # Determine closest beacon
        closest_id = None
        closest_rssi = None
        closest_distance = None

        if rssi_history:
            avg_rssi_dict = {bid: sum(dq) / len(dq) for bid, dq in rssi_history.items()}
            closest_id = max(avg_rssi_dict, key=avg_rssi_dict.get)
            closest_rssi = avg_rssi_dict[closest_id]
            closest_distance = 10 ** ((-8 - closest_rssi - 20 * np.log10(2400) + 28) / 28)

            # ✅ Hysteresis logic for alert state
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

        # Send update to subscribers
        if subscribers:
            try:
                response_payload = {
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

                response_json = json.dumps(response_payload).encode()
                for sub_addr in list(subscribers):
                    sock.sendto(response_json, sub_addr)

            except Exception as e:
                print(f"⚠️ Failed to send update to subscribers: {e}")

    except json.JSONDecodeError:
        print("⚠️ Received non-JSON data.")
        continue
    except Exception as e:
        print(f"⚠️ Error: {e}")
        continue

    # Remove stale beacons
    now = time.time()
    expired_ids = [bid for bid, t in last_seen_time.items() if now - t > TIMEOUT_SECONDS]
    for bid in expired_ids:
        print(f"⏳ Beacon '{bid}' removed due to timeout.")
        rssi_history.pop(bid, None)
        last_seen_time.pop(bid, None)
