import socket
import json
import time 
import numpy as np

# Server configuration
UDP_IP = "0.0.0.0"       # Listen on all available network interfaces
UDP_PORT = 8000          # Port to listen on
MEAS_TIME = 1            # Maximum seconds to measure and choose nearest beacon
ID_IDX = 0               # Index of the column corresponding to the ID
RSSI_IDX = 1             # Index of the column corresponding to the RSSI




# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP Server listening on port {UDP_PORT}...")

# Start receiving data

rssi_measurements = {}
rssi_num = {}
rssi_average = {}
start = time.time()

while True:
    
    data, addr = sock.recvfrom(1024)  # Buffer size = 1024 bytes
    message = data.decode()
    # print(f"\nReceived data from {addr}:{message}")
    try:
       
        # Attempt to parse JSON
        parsed = json.loads(message)

        # # Pretty-print the contents
        # print(f"  ID: {parsed.get('id')}")
        # # print(f"  Timestamp: {parsed.get('timestamp')}")
        # print(f"  RSSI: {parsed.get('rssi')} dBm")
        # print(f"  Temperature: {parsed.get('temperature')} °C")
        # print(f"  BPM: {parsed.get('bpm')}")

        # accel = parsed.get("acceleration", {})
        # print(f"  Acceleration:")
        # print(f"    x: {accel.get('x')}")
        # print(f"    y: {accel.get('y')}")
        # print(f"    z: {accel.get('z')}")

        # wrist_batt = parsed.get("wrist_batt", {})
        # print(f"   Battery in wristband:")
        # print(f"    V: {wrist_batt.get('wrist_battV')}")
        # print(f"    P: {wrist_batt.get('wrist_battP')}")

        # wrist_batt = parsed.get("my_batt", {})
        # print(f"   Battery in beacon:")
        # print(f"    V: {wrist_batt.get('my_battV')}")
        # print(f"    P: {wrist_batt.get('my_battP')}")

    except json.JSONDecodeError:
        print("⚠️ Received non-JSON data.")
    except Exception as e:
        print(f"⚠️ Error: {e}")

    if parsed.get('id') in rssi_measurements:
        rssi_measurements[parsed.get('id')] = rssi_measurements[parsed.get('id')] + parsed.get('rssi')
        rssi_num[parsed.get('id')] = rssi_num[parsed.get('id')] + 1
    else:
        rssi_measurements[parsed.get('id')] = parsed.get('rssi')
        rssi_num[parsed.get('id')] = 1


    if time.time() - start > MEAS_TIME:

        if not rssi_measurements:
            print("The dictionary is empty.")
            break

        # Create a new dictionary with values divided by MAX_DATA
        for id, sum_rssi in rssi_measurements.items():
            rssi_average[id] = sum_rssi / rssi_num[id]

        # Find the maximum average value of RSSI
        max_rssi = -2000
        max_rssi_id = 0
        for id, rssi_avg in rssi_average.items():
            if rssi_avg > max_rssi:
                max_rssi = rssi_avg
                max_rssi_id = id
        distance = 10**((-8-max_rssi - 20*np.log10(2400) + 28)/28)
        print(f"The nearest beacon is {max_rssi_id}")
        print(f"The RSSI is {max_rssi}")
        print(f"The distance to this beacon is {distance}")

        rssi_measurements = {}
        rssi_num = {}
        rssi_average = {}
        start = time.time()



    















        # Create socket
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind((UDP_IP, UDP_PORT))

# print(f"UDP server listening on port {UDP_PORT}...\n")

# while True:
#     data, addr = sock.recvfrom(1024)
#     message = data.decode().strip()
#     print(f"Received from {addr}: {message}")

    # Parse the comma-separated data
    # try:
    #     fields = message.split(",")
    #     if len(fields) != 8:
    #         raise ValueError("Expected 8 data fields")

    #     # Assign values
    #     id = fields[0]
    #     timestamp = fields[1]
    #     rssi = int(fields[2])
    #     temp = float(fields[3])
    #     ax = float(fields[4])
    #     ay = float(fields[5])
    #     az = float(fields[6])
    #     bpm = int(fields[7])

    #     # Display parsed data
    #     print("\n📊 Parsed Data:")
    #     print(f"  ID         : {id}")
    #     print(f"  Timestamp  : {timestamp}")
    #     print(f"  RSSI       : {rssi} dBm")
    #     print(f"  Temp       : {temp} °C")
    #     print(f"  BPM        : {bpm}")
    #     print(f"  Accel (x,y,z): {ax}, {ay}, {az}")

    # except Exception as e:
    #     print(f"⚠️ Error parsing data: {e}")