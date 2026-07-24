# Raspberry Pi access

How to reach the Pi over SSH and pull the code onto it. The Pi is just a host
for the code in this repo, it holds nothing that is not already here.

This board uses the same value for user and hostname:

- user: `gatr2`
- hostname: `gatr2`, so `gatr2.local`
- password: in `secrets.local` at the repo root, which is git-ignored so it
  stays out of the public history. Set up a key below and you never type it.

## Provisioning after a reflash

Do these once on a fresh card, in order. The sections below have the detail.

1. Image the card with Raspberry Pi Imager. In the settings gear, set hostname
   `gatr2`, user `gatr2`, the password, enable SSH, and add the WiFi networks.
2. Boot, then SSH in (see Find the Pi).
3. Update the system:

       sudo apt update && sudo apt full-upgrade -y

4. Enable UART5 for the brain link (GPIO12/13):

       echo "dtoverlay=uart5" | sudo tee -a /boot/firmware/config.txt
       sudo reboot

   After the reboot, confirm the device exists:

       ls -l /dev/ttyAMA*

   On the Pi 4 this is `/dev/ttyAMA5` (uart5 maps to ttyAMA5). If the name
   differs on your board, set `SERIAL_PORT` in the bring-up script to match.
5. Clone the repo (see Get the code).
6. Run a bring-up test to confirm the hardware (see Run a bring-up test).

Add more here as hardware lands, for example the camera overlay.

## Enable SSH

Easiest at imaging time: in Raspberry Pi Imager, open the settings gear and
enable SSH, set the user, and set the hostname before writing the card.

On a board that is already running:

    sudo raspi-config    # Interface Options -> SSH -> enable

## Find the Pi

On the same network:

    ping gatr2.local
    ssh gatr2@gatr2.local        # user here, password in secrets.local

If the hostname will not resolve:

- Confirm the Pi is booted. The green activity LED should be blinking, not
  sitting solid or dark.
- Run an Ethernet cable straight from the Pi to your laptop, then try the same
  `ssh gatr2@gatr2.local` again.
- Last resort, plug in a monitor and keyboard and work on it directly.

To get the IP another way, from your router, or on the Pi itself:

    hostname -I

## Networks

The Pi has WiFi stored for the home network and the `<name>` hotspot. If
neither is in range, use the Ethernet or monitor fallback above.

## First connection

    ssh gatr2@gatr2.local

Accept the host key on first connect. Log in with the password once, then set
up a key so you do not need it again.

## Key login, recommended

From your laptop, once:

    ssh-keygen -t ed25519        # if you do not already have a key
    ssh-copy-id gatr2@gatr2.local

Now `ssh gatr2@gatr2.local` logs in with no password. Optionally add a shortcut
to `~/.ssh/config` on your laptop:

    Host gatr2
        HostName gatr2.local
        User gatr2

Then it is just:

    ssh gatr2

## Get the code onto the Pi

    git clone https://github.com/<owner>/GATR2-Electronics.git
    cd GATR2-Electronics

Pull later updates with:

    git pull

## Run a bring-up test

    cd bringup/rs485_link
    python3 pi_transmit.py

See that test's README for wiring and the one-time UART setup.
