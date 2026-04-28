from dora import Node
import serial
import time

import numpy as np

from rustypot import Scs0009PyController

gantry = 0

def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def send_command(cmd):
    print(f"Sending: {cmd.strip()}")
    gantry.write(cmd.encode())

    timeout = time.time() + 2
    while time.time() < timeout:
        if gantry.in_waiting:
            response = gantry.readline().decode().strip()
            print(f"Arduino said: {response}")
            if "ok" in response or "ALARM" in response:
                return response
    return "TIMEOUT"
# 1. Initialize Gantry (USB-A port)
# Verify the COM port in Device Manager; it will be different from the hand (COM8)
gantry = serial.Serial('COM5', 115200, timeout=1) 
#time.sleep(2)
#send_command("$X\n")     # Unlock the startup alarm
#send_command("$H\n")
#time.sleep(90)
#send_command("G10 P0 L20 X0\n")
#send_command("G10 P0 L20 Y0\n")
#send_command("G90\n")    # Set to Absolute Mode (for tracking)
#send_command("G21\n")    # Ensure we are in Millimeters

# 1. Initialize and Home
send_command("$X\n") # Unlock startup alarm
print("Starting Homing Cycle...")
gantry.write(b"$H\n") # Start homing

# Instead of time.sleep(90), wait for the physical 'ok' from the homing finish
homing_complete = False
while not homing_complete:
    if gantry.in_waiting:
        line = gantry.readline().decode().strip()
        print(f"Homing status: {line}")
        if "ok" in line:
            homing_complete = True
    time.sleep(0.1)

# Post-Homing Setup
send_command("G90\n") # Absolute Mode
send_command("G21\n") # Millimeters

# --- Main Dora Loop ---
history_x, history_y = [], []
BUFFER_SIZE = 5
MOVE_THRESHOLD = 4.0  # mm (Adjust this: higher = steadier, lower = more sensitive)
SEND_INTERVAL = 0.04   # seconds (20Hz)
FEEDRATE = 8000        # mm/min adjust for gantry speed
SYNC_EVERY = 10
commands_sent = 0
last_target_x = 0
last_target_y = 0
last_send_time = 0


def send_init(cmd):
      gantry.write(cmd.encode())
      time.sleep(0.1)

send_init("G90\n") 
send_init("$110=5000\n") # Max speed X
send_init("$111=5000\n") # Max speed Y
send_init("$120=400\n")  # Acceleration X (High = Snappy)
send_init("$121=400\n")  # Acceleration Y (High = Snappy)

commands_sent = 0 
last_target_x = 0
last_target_y = 0
last_send_time = 0

print("Gantry ready. Starting loop...")


# --- Main Dora Loop ---
node = Node()
for event in node:
    if event["type"] == "INPUT" and event["id"] == "wrist_pos":
        try:
            # Settings for window offset
            scaley = 1
            scalex = 1
            offsetx = 0
            offsety = 0
            # 1. Get Hand Pos
            val = event["value"].to_pylist()
            wrist = val[0] if isinstance(val[0], list) else val
            
            # 2. Map (330mm workspace)
            tx = (1 - max(0.02, min(0.98, wrist[0]))) * 330 * scalex
            ty = (1 - max(0.1, min(0.9, wrist[1]))) * 330 * scaley

            tx = tx + offsetx
            ty = ty + offsety

            tx = constrain(tx,0,330)
            ty = constrain(ty,0,330)

            # 3. Micro-Smoothing
            history_x.append(tx); history_y.append(ty)
            if len(history_x) > BUFFER_SIZE:
                history_x.pop(0); history_y.pop(0)

            sx = sum(history_x) / len(history_x)
            sy = sum(history_y) / len(history_y)

            # 4. The Jogging Logic
            now = time.time()
            if now - last_send_time > SEND_INTERVAL:
                dx = abs(sx - last_target_x)
                dy = abs(sy - last_target_y)

                if dx > MOVE_THRESHOLD or dy > MOVE_THRESHOLD:
                    # THE SECRET SAUCE: 
                    # $J= (Jogging) is better than G0/G1 for real-time.
                    # It tells GRBL: "Go here now, and if a new $J comes, move there instead."
                    jog_cmd = f"$J=G21G90X{sx:.2f}Y{sy:.2f}F{FEEDRATE}\n"
                    gantry.write(jog_cmd.encode())
                    
                    last_target_x, last_target_y = sx, sy
                
                last_send_time = now

            # 5. Continuous Buffer Flush
            # If we don't read the 'ok' responses, the Arduino's output buffer fills up
            # and slows down the whole chip.
            if gantry.in_waiting:
                gantry.read_all()

        except Exception:
            continue