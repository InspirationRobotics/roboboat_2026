"""
File with code taken from motor_core.py in order to test motor_core.py.
This serves absolutely no purpose at present.
"""
# NOTE: Simply put in a rudimentary process function.
import serial
import time

class TestControl():
    def __init__(self, port = "/dev/tty/ACM0"):
        arduino = serial.Serial(port = port, baudrate = 9600)
        time.sleep(1)

    def process_input(self, data):
        if data == "Exit" or data == "Quit":
            return False
        
        elif data == "Continue":
            return True

    def handle_input(self):
        print("Welcome to the Motor Control Test Terminal.")
        print("Available commands:")
        # NOTE: Will think about this and come back.
        print("[FILLER TODO]")

        while True:
            data = input(">>> ")
            if not self.process_input(data):
                break
            print("Exiting...")

if __name__ == "main":
    control = TestControl()
    control.handle_input()