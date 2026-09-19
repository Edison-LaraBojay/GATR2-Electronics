# Raspberry Pi access

Use a headless Raspberry Pi OS installation with SSH enabled. Choose the hostname,
account, network settings, and credentials during imaging. The examples below use
`gatr2@gatr2.local`; replace them with the account and hostname on your Pi.

## Connect

```text
ssh gatr2@gatr2.local
```

If local hostname resolution fails, use the address shown by the router or by
`hostname -I` on the Pi. Ethernet or a directly connected keyboard/display can
help establish the initial connection.

To enable SSH on an existing installation, use `sudo raspi-config` and its SSH
interface option. For key-based login, create an SSH key on the viewing computer
if needed and install its public key on the Pi:

```text
ssh-keygen -t ed25519
ssh-copy-id gatr2@gatr2.local
```

## Provision the runtime

Update packages, then clone this repository using its GitHub clone URL. The
[Navigatr README](../pi/navigatr/README.md) has build, test, and run commands;
[Pi camera setup](../pi/navigatr/docs/pi_camera_setup.md) lists camera packages and
capture configuration. The default runnable profile is a synthetic demo. Physical
robot profiles use the measurements in the
[calibration inventory](../pi/navigatr/docs/calibration_inventory.md).

## HAT UART

The HAT's Brain interface uses UART5 on GPIO12/13. Enable the corresponding overlay
in `/boot/firmware/config.txt`:

```text
dtoverlay=uart5
```

Reboot after editing boot configuration and inspect the resulting `/dev/ttyAMA*`
devices. The profiles use `/dev/ttyAMA5` for this link; confirm the device on the
actual installation and update the XML or bench script if it differs. The Pico
link is a separate UART and must match its 115200-baud firmware configuration.

The [RS-485 bench test](../bench/rs485_link/README.md) documents transmit bring-up.
The runtime's configured DriverEnable remains high while the link is open;
bidirectional command traffic needs transmit/receive turnaround work.

## View the running Pi

From the viewing computer:

```text
ssh -N -L 8765:127.0.0.1:8765 gatr2@gatr2.local
```

Run Navigatr on the Pi with inspection enabled, then open
`http://127.0.0.1:8765/` on the viewing computer. The
[inspection guide](../pi/navigatr/docs/inspection.md) describes the field view,
robot pose, camera overlays, diagnostics, and service configuration.
