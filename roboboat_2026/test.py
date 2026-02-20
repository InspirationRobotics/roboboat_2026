#!/usr/bin/env python3
import serial
import threading

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    lock = threading.Lock()

    print("Connected to Teensy.")
    print("Type: m1,m2,m3,m4,pump")
    print("Example: 0.0,0.0,0.0,0.0,0")
    print("Type 'q' to quit.\n")

    try:
        while True:
            line = input(">> ").strip()

            if line.lower() in ("q", "quit", "exit"):
                break

            # basic validation
            parts = line.split(",")
            if len(parts) != 5:
                print("❌ Expected 5 values: m1,m2,m3,m4,pump")
                continue

            try:
                values = [float(p) for p in parts[:4]] + [int(parts[4])]
            except ValueError:
                print("❌ Invalid number format")
                continue

            msg = ",".join(str(v) for v in values) + "\n"

            with lock:
                ser.write(msg.encode("utf-8"))

            print(f"Sent: {msg.strip()}")

    except KeyboardInterrupt:
        print("\nCtrl-C received")

    finally:
        print("Stopping thrusters and closing port...")
        ser.write(b"0,0,0,0,0\n")
        ser.close()

if __name__ == "__main__":
    main()

