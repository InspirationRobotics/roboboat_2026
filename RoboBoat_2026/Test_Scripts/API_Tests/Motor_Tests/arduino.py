import serial
import time

# === CONFIG ===
PORT = 'COM7'   # <-- Change to your Arduino port (e.g. /dev/ttyACM0 on Linux)
BAUD = 115200
DELAY = 0.5     # delay between commands (seconds)

# Base PWM values
NEUTRAL = 1500
DELTA = 80      # amount to increase/decrease around neutral

# Connect to Arduino
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # wait for Arduino reset

print(f"Connected to {PORT} at {BAUD} baud.\n")

def send_pwm(fp, fs, ap, asb):
    """Send PWM command string to Arduino."""
    cmd = f"{fp},{fs},{ap},{asb}\n"
    ser.write(cmd.encode('utf-8'))
    print(f"Sent: {cmd.strip()}")

try:
    # --- TEST SEQUENCE ---
    print("Starting low-speed test... (Ctrl+C to stop)\n")

    while True:
        # Step 1: All neutral
        send_pwm(NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL)
        time.sleep(1)

        # Step 2: Slight forward
        send_pwm(NEUTRAL + DELTA, NEUTRAL + DELTA, NEUTRAL + DELTA, NEUTRAL + DELTA)
        time.sleep(2)

        # Step 3: Slight reverse
        send_pwm(NEUTRAL - DELTA, NEUTRAL - DELTA, NEUTRAL - DELTA, NEUTRAL - DELTA)
        time.sleep(2)

        # Step 4: Small turn test (port slower, starboard faster)
        send_pwm(NEUTRAL - DELTA, NEUTRAL + DELTA, NEUTRAL - DELTA, NEUTRAL + DELTA)
        time.sleep(2)

        # Step 5: Back to neutral
        send_pwm(NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL)
        time.sleep(2)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    send_pwm(NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL)
    ser.close()
    print("Serial connection closed.")
