import serial
import time

s = serial.Serial('COM5', 115200, timeout=1)
time.sleep(3) # Wait for reboot

def send_command(cmd):
    print(f"Sending: {cmd.strip()}")
    s.write(cmd.encode())
    response = s.readline().decode().strip() # Wait for Arduino to answer
    print(f"Arduino said: {response}")
    time.sleep(0.3)

send_command("$X\n")     # Unlock the startup alarm
send_command("$H\n")
time.sleep(15)
send_command("G10 P0 L20 X0\n")
send_command("G10 P0 L20 Y0\n")
send_command("G90\n")    # Set to Absolute Mode (for tracking)
send_command("G21\n")    # Ensure we are in Millimeters
send_command("G0 X10 Y10\n")
send_command("G0 X50 Y50\n")
time.sleep(2)
send_command("G0 X1 Y1\n")