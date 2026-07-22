# rs485_link

Confirms the Pi to brain RS-485 link carries bytes. The Pi transmits a message
on a loop, the brain reads it and shows it on screen. Nothing here is the real
protocol, it is only a wire test.

## Wiring

From the transceiver schematic (SN65HVD11):

| Pi | Transceiver | Note |
|----|-------------|------|
| GPIO12 / pin 32 (TXD5) | D, pin 4 | Pi transmits |
| GPIO13 / pin 33 (RXD5) | R, pin 1 | unused in this test |
| GPIO6 / pin 31 | DE + /RE, pins 3 and 2 | EN, 10k pulldown to GND |

The transceiver A and B pins go into a V5 smart port.

DE and /RE share one EN line, so the link idles in receive mode. The Pi script
drives EN high to transmit.

## Pi side

GPIO12/13 is UART5. Enable it once:

1. Add `dtoverlay=uart5` to `/boot/firmware/config.txt` and reboot.
2. Find the device: `ls -l /dev/ttyAMA*`, set `SERIAL_PORT` in the script.

Then:

    python3 pi_transmit.py
    python3 pi_transmit.py "any message"

Missing packages install themselves on first run. Default message is
"Hello World".

## Brain side

Copy `brain_main.cpp` into a PROS project as `src/main.cpp`. Set `kSerialPort`
to the smart port the RS-485 pair is plugged into. Build and run.

## Success

The brain screen shows "RS485 link up", a message counter that climbs once per
second, and the last received text. If the counter climbs, the link works.
