#!/usr/bin/env python3
"""RS-485 link transmit test (Pi side).

Sends a message over the Pi UART through the RS-485 transceiver, on a loop.
It only transmits, it never reads. Use it to confirm the Pi to brain link
carries bytes before building anything on top of it.

Run it:
    python3 pi_transmit.py
    python3 pi_transmit.py "some other message"

Missing packages are pip-installed on first run.

Wiring, from the schematic:
    GPIO12 / pin 32  TXD5  -> transceiver D   (RSTX)
    GPIO13 / pin 33  RXD5  -> transceiver R   (RSRX, unused here)
    GPIO6  / pin 31        -> transceiver DE and /RE  (EN), 10k pulldown to GND

DE and /RE are tied to one EN line, so the link idles in receive mode. This
script drives EN high to enable the driver and transmit.

GPIO12/13 is UART5. Enable it once: add
    dtoverlay=uart5
to /boot/firmware/config.txt and reboot. Then check the device name with
    ls -l /dev/ttyAMA*
and set SERIAL_PORT below to match.
"""

import subprocess
import sys
import time

# ----- edit these -----
MESSAGE = "Hello World"
SERIAL_PORT = "/dev/ttyAMA5"  # UART5 (GPIO12/13) after dtoverlay=uart5
BAUD = 115200
EN_GPIO = 6                   # drives transceiver DE + /RE high for transmit
INTERVAL_S = 1.0
# -----------------------


def ensure(module, package=None):
    """Import a module, pip-installing it first if it is missing."""
    package = package or module
    try:
        return __import__(module)
    except ImportError:
        print("installing " + package + " ...")
        attempts = (
            [sys.executable, "-m", "pip", "install", package],
            [sys.executable, "-m", "pip", "install", "--break-system-packages", package],
        )
        for args in attempts:
            if subprocess.run(args).returncode == 0:
                break
        return __import__(module)


serial = ensure("serial", "pyserial")
ensure("gpiozero")
from gpiozero import OutputDevice


def main():
    message = sys.argv[1] if len(sys.argv) > 1 else MESSAGE

    en = OutputDevice(EN_GPIO)
    en.on()  # driver enabled, transmit mode

    port = serial.Serial(SERIAL_PORT, BAUD, timeout=0)
    payload = (message + "\n").encode()

    print("transmitting " + repr(message) + " on " + SERIAL_PORT
          + " at " + str(BAUD) + " baud. ctrl-c to stop.")
    sent = 0
    try:
        while True:
            port.write(payload)
            port.flush()
            sent += 1
            print("sent #" + str(sent) + ": " + message)
            time.sleep(INTERVAL_S)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        port.close()
        en.off()


if __name__ == "__main__":
    main()
