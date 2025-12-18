import serial
import time

class ArdiunoCompound:
    def __init__(self, port, baudrate=9600):
        """
        Initializes the serial connection to the Mini Maestro Servo Controller.

        Args:
            - port (str): The COM port (e.g., "COM3" for Windows or "/dev/ttyUSB0" for Linux/Mac).
            - baudrate (int): Communication speed (default is 9600).
        """
        self.serial_conn = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Allow time for the connection to establish

    def send_command(self, msg):
        """
        Sends a string or byte message to the serial device.
        Automatically encodes string messages to bytes.
        """
        if isinstance(msg, str):
            msg = msg.encode()  # Convert to bytes if it's a string
        self.serial_conn.write(msg)

    def set_pwm(self, channel, target):
        """
        Sends a command to set the PWM signal for a servo.

        Args:
            - channel (int): The servo channel (0-5 for Mini Maestro 6).
            - target (int): PWM value (in microseconds, typically 500-2500).
        """
        target = target * 4  # Convert to Maestro format
        lsb = target & 0x7F  # Lower 7 bits
        msb = (target >> 7) & 0x7F  # Upper 7 bits
        command = bytes([0x84, channel, lsb, msb])  # Compact binary command
        self.serial_conn.write(command)

    def close(self):
        """Closes the serial connection."""
        if self.serial_conn.is_open:
            self.serial_conn.close()


# === Example usage ===
if __name__ == "__main__":
    # Change port based on your system (e.g., "COM3" on Windows, "/dev/ttyUSB0" on Linux/Mac)
    ardiuno_compound = ArdiunoCompound(port="/dev/ttyACM3")

    # Send simple character command
    ardiuno_compound.send_command("g")  # Will now send as bytes: b'g' 'a' for reload
    time.sleep(2)

    # Close connection when done
    ardiuno_compound.close()