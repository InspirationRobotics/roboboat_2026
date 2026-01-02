#!/usr/bin/env python3

from unittest.mock import patch, MagicMock
from deviceHelper import findFromId

# Example config (same as roboboat.json snippet)
config = {
    "teensy": {"port": "/devices/platform/3610000.xhci/usb1/1-2/1-2.4/1-2.4:1.0/tty/ttyACM0", "rate": 115200},
    "ball_launcher": {"port": "/devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2.2/1-2.2.2:1.0/tty/ttyACM1", "rate": 9600},
    "gps": {"port": "/devices/platform/3610000.xhci/usb1/1-2/1-2.1/1-2.1:1.0/ttyUSB0/tty/ttyUSB0", "rate": 115200},
    "oakd_lr": {"id": "18443010012C48F500"},
    "web_came": {"port": "/devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2.3/1-2.2.3:1.0/video4linux/video0"},
}

def make_popen_mock(output: str):
    """Return a fake os.popen object whose .read() returns output."""
    def _mock_popen(cmd):
        m = MagicMock()
        m.read.return_value = output
        return m
    return _mock_popen

def main():
    # Fake usbLink.sh output: devices with different, volatile numbers
    usb_output = """\
/dev/ttyACM5 - /devices/platform/3610000.xhci/usb1/1-2/1-2.4/1-2.4:1.0/tty/ttyACM9
/dev/ttyACM3 - /devices/platform/3610000.xhci/usb1/1-2/1-2.2/1-2.2.2/1-2.2.2:1.0/tty/ttyACM1
/dev/ttyUSB2 - /devices/platform/3610000.xhci/usb1/1-2/1-2.1/1-2.1:1.0/ttyUSB0/tty/ttyUSB0
"""

    with patch("os.popen", make_popen_mock(usb_output)):
        teensy_port = findFromId([config["teensy"]["port"]])
        ball_port   = findFromId([config["ball_launcher"]["port"]])
        gps_port    = findFromId([config["gps"]["port"]])

    print("Teensy mapped to:      ", teensy_port)   # expect: /dev/ttyACM5
    print("Ball launcher mapped to:", ball_port)    # expect: /dev/ttyACM3
    print("GPS mapped to:         ", gps_port)      # expect: /dev/ttyUSB2)

if __name__ == "__main__":
    main()
