# Raspberry Pi setup and automatic startup

This setup runs naviGATR with a Brain-profiled configuration whenever the Pi
boots. Internet is needed to install build packages once. Afterwards
localization and the 3D viewer work offline; Ethernet is only needed to view or
manage the Pi. Power the Pi separately from the Ethernet cable.

The robot is described on the Brain, in
[brain/robot/gatr2_robot.h](../brain/robot/gatr2_robot.h), and sent to the Pi as
a robot profile each time the Brain connects; the Pi config names only its own
devices. The current bench: Pico encoder ports 0 (forward) and 1 (sideways),
the VEX IMU on Brain Smart Port **1**, and the Brain-Pi link over **USB**. The
Pi needs the config that matches the Brain's link:

| Brain link (`kUseUsb` in `gatr2_robot.h`) | Pi config |
|---|---|
| USB (`true`, the bench) | `config/override/brain_profile_usb.xml` |
| RS-485 on a Smart Port (`false`, `kLinkPort`) | `config/override/brain_profile_rs485.xml` |

That is the only Brain setting the Pi must match; nothing detects the
transport automatically. Build and upload the
[Brain locaGATR app](brain_setup.md#2-build-and-upload); the Pi service
does not start a program on the Brain. See
[Brain robot profiles](../pi/naviGATR/docs/brain_profile.md) for what the Pi does
with the profile and [USB bench](../pi/naviGATR/docs/usb_localization_bench.md)
for the bench procedure.

## 1. Get the Pi online once, over Ethernet

If you already enabled the direct `robot-direct` profile, first use the
[return-to-router steps](#6-return-to-router-ethernet-when-you-need-internet).

Use Raspberry Pi OS with NetworkManager (Bookworm or later). For a fresh SD card,
set a username, password, hostname, and **enable SSH** in Raspberry Pi Imager.
Wi-Fi configuration is unnecessary. On an existing installation, enable SSH from
its local terminal:

```sh
sudo systemctl enable --now ssh
```

Connect the Pi and Windows computer to an internet-connected router by Ethernet.
Leave Windows Ethernet IP and DNS on **Automatic (DHCP)** for this step. Find the
Pi's address in the router's client list, or run `hostname -I` on the Pi. From
Windows PowerShell, replacing the example username and address:

```powershell
ssh YOUR_USER@PI_ROUTER_IP
```

The configured hostname, such as `gatr2.local`, may also work. These are ordinary
[Raspberry Pi SSH connections](https://www.raspberrypi.com/documentation/computers/remote-access.html).
All shell commands below run on the Pi unless labeled Windows.

Install the tools needed for this camera-free build:

```sh
sudo apt update
sudo apt install -y build-essential cmake git
```

## 2. Transfer the current code and build

**Use the current working files for this bench test.** `git pull` cannot retrieve
changes that have not been committed and pushed. To copy the current Pi program
and shared protocol from Windows, run these in the repository's top directory:

```powershell
tar.exe -czf "$env:TEMP\navigatr-source.tar.gz" --exclude=build --exclude=build-* --exclude=.pio pi/naviGATR common
scp "$env:TEMP\navigatr-source.tar.gz" YOUR_USER@PI_ROUTER_IP:/tmp/navigatr-source.tar.gz
```

Then on the Pi:

```sh
mkdir -p ~/GATR2-Electronics
tar -xzf /tmp/navigatr-source.tar.gz -C ~/GATR2-Electronics
cd ~/GATR2-Electronics/pi/naviGATR
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release \
  -DNAVIGATR_BUILD_TESTS=OFF -DNAVIGATR_WITH_LIBCAMERA=OFF
cmake --build build-bench -j2
```

This replaces files with the same names, so preserve Pi-only calibration edits
before repeating the transfer. Once the changes are published, a normal clone of
`https://github.com/Edison-LaraBojay/GATR2-Electronics.git` also works. The viewer
is built into the executable and requires no browser internet/CDN.

## 3. Enable the HAT serial connections

Run `sudo raspi-config`, select its serial-port interface settings, disable the
**serial login shell**, and enable the **serial port hardware**. The Pico link
and Brain link must not be used by a serial console.

The HAT's Brain interface uses UART5 on GPIO12/13. Ensure
`/boot/firmware/config.txt` includes these lines in an applicable section:

```text
enable_uart=1
dtoverlay=uart5
```

Reboot with `sudo reboot`, reconnect by SSH, and inspect the UART devices:

```sh
ls -l /dev/serial* /dev/ttyAMA*
```

Both Brain-profile configs use `/dev/ttyAMA0` for the Pico at 115200 baud;
`brain_profile_rs485.xml` also uses `/dev/ttyAMA5` for the Brain (UART5 is only
needed for RS-485; USB finds the Brain's USB port itself). Confirm these paths
on the Pi. On Pi 4, the primary header UART can be
`/dev/ttyS0` rather than `/dev/ttyAMA0` depending on Bluetooth/UART overlays;
`/dev/serial0` identifies the primary UART when that alias is available. Update
the XML to match the hardware instead of assuming the suffixes. The official
[UART documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)
describes the primary UART aliases and overlays.

The RS-485 Brain link also uses a driver-enable pin. Its XML `gpio` value is a
**sysfs GPIO number**, which may differ from the BCM number: on a kernel whose
GPIO chip starts at 512, BCM GPIO6 is **518**, not 6. Check
`/sys/class/gpio/gpiochip*/base` and the chip labels. See
[serial resource configuration](navigatr_resources.md#linux_serial_link) and the
[RS-485 bench test](../bench/rs485_link/README.md). Software device names and GPIO
numbering must match this Pi's configuration.

## 4. Enable automatic startup

Stop any manually running naviGATR first (Ctrl+C in its terminal). The installer
does not stop unrelated processes. From the Pi's program directory, install the
service as your normal Pi account:

```sh
cd ~/GATR2-Electronics/pi/naviGATR
sudo bash tools/install_service.sh --user "$USER"
sudo systemctl status navigatr --no-pager
sudo journalctl -u navigatr -n 50 --no-pager
```

The installer defaults to `build-bench/navigatr` and
`config/override/brain_profile_usb.xml`. It enables and starts
`navigatr.service`, which runs without an SSH session or network connection and
restarts after a process exit. Logs go to the journal. It uses absolute paths to
this checkout; rerun the installer if you move it.

The same with the config named explicitly, and the RS-485 variant:

```sh
sudo bash tools/install_service.sh --user "$USER" --config config/override/brain_profile_usb.xml
sudo bash tools/install_service.sh --user "$USER" --binary build-bench/navigatr \
  --config config/override/brain_profile_rs485.xml
```

Changing the robot's geometry, encoders, IMU source or footprint is a Brain
upload; the service keeps running and applies the new profile when the Brain
connects. Changing between USB and RS-485, or editing the Pi XML, needs the
installer (or `sudo systemctl restart navigatr` after an XML edit).

Add `--dry-run` to inspect the unit without installing it. The unit grants the
existing Raspberry Pi OS `dialout` and `gpio` groups to the service; installation
reports an error if either required group is missing. A green
service status alone does **not** prove either link is working: check the journal
for UART/GPIO permission or device faults, then confirm changing encoder/IMU
readings and pose in the Brain app and viewer. The Pico UART (and the RS-485
UART) are `required`: startup fails and the service retries if they are
unavailable. The Brain USB port is optional and retried once per second.

The journal shows the lifecycle as `event: ...` lines: the Brain session,
`profile <id> applied (...)` or `refused: <reason>`, calibration results, Pico
restarts, and `sensor lost: ...: place again` or `motion lost: ...: place
again` (a model had to discard motion it measured).

Useful commands:

```sh
sudo systemctl restart navigatr             # reload changed XML / rebuilt binary
sudo journalctl -u navigatr -f              # live logs; Ctrl+C leaves it running
sudo systemctl stop navigatr                # stop until reboot or manual start
sudo systemctl disable --now navigatr       # stop and disable automatic startup
sudo systemctl enable --now navigatr        # enable again
```

Stop the service before running the executable manually so two processes do not
compete for the UARTs or viewer port. For source updates: stop, transfer/rebuild,
then start it again. Reboot once to confirm startup and the viewer return without
manually launching naviGATR.

The Brain app places the robot at its start pose once, when the link, profile
and sensors are ready. If it gave up waiting (the Pi booted too slowly), or the
pose was lost later, put the robot at the start pose and press controller
**A**. That places it without restarting the Pi service.

## 5. Set up direct Windows-to-Pi Ethernet

Do this after online installation. Use a subnet that does not overlap another
active connection; these examples use `192.168.50.0/24`.

First create the saved direct profile on the Pi **without activating it yet**:

```sh
nmcli device status
nmcli connection show
sudo nmcli connection add type ethernet ifname eth0 con-name robot-direct \
  ipv4.method manual ipv4.addresses 192.168.50.2/24 \
  ipv4.never-default yes ipv6.method disabled \
  connection.autoconnect no connection.autoconnect-priority 100
```

Replace `eth0` if the Ethernet interface has a different name. If `robot-direct`
already exists, use `nmcli connection modify robot-direct` with the settings
instead of adding a duplicate. Keep the original DHCP/router profile and record
its name for switching back. NetworkManager chooses higher
[autoconnect priorities](https://www.networkmanager.dev/docs/api/latest/settings-connection.html)
when bringing up a connection; it cannot infer whether the cable goes to your
computer or router.

When ready to switch, run on the Pi:

```sh
sudo nmcli connection modify robot-direct connection.autoconnect yes
sudo nmcli connection up robot-direct
```

**The current router SSH connection will drop.** Move the cable so the Pi connects
directly to the computer. In Windows, open **Settings > Network & internet >
Ethernet > IP assignment > Edit**, choose **Manual**, enable IPv4, and set:

| Setting | Windows | Pi (already saved) |
|---|---|---|
| Address | `192.168.50.1` | `192.168.50.2` |
| Subnet mask / prefix | `255.255.255.0` / `24` | `/24` |
| Gateway and DNS | Leave blank | None |

Windows saying "No internet" is expected. This direct connection does not share
the computer's internet access. From Windows:

```powershell
ssh -L 8765:127.0.0.1:8765 YOUR_USER@192.168.50.2
```

Open **http://127.0.0.1:8765/** on Windows. Keep this SSH window open while viewing.
To forward the viewer without an interactive terminal, add `-N`. Closing the
viewer or disconnecting Ethernet does not stop the Pi service. The
[inspection guide](../pi/naviGATR/docs/inspection.md) explains the field, pose,
Brain link readiness and diagnostics. The Brain-profile configs have no live
camera or AprilTag updates.

## 6. Return to router Ethernet when you need internet

Before unplugging the direct cable, run on the Pi:

```sh
sudo nmcli connection modify robot-direct connection.autoconnect no
sudo reboot
```

During reboot, connect the Pi to the router. Restore Windows Ethernet IP and DNS
to **Automatic (DHCP)** if connecting the computer to the router too. The Pi's
retained DHCP profile can then autoconnect; find its router address and SSH there.
If that original profile has autoconnect disabled, enable it by its recorded name
before rebooting. A local keyboard/display can also run
`nmcli connection up "YOUR_DHCP_PROFILE"` to recover access.

To return to the direct cable, repeat the two activation commands in step 5 and
restore Windows's static address. Automatic naviGATR startup is independent of
the Ethernet profile. Before disconnecting Pi power, shut down with
`sudo poweroff` when practical.
