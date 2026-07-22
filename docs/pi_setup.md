# Raspberry Pi access

How to reach the Pi over SSH and pull the code onto it. The Pi is just a host
for the code in this repo, it holds nothing that is not already here.

This board uses the same value for user and hostname:

- user: `gatr2`
- hostname: `gatr2`, so `gatr2.local`
- password: in `secrets.local` at the repo root, which is git-ignored so it
  stays out of the public history. Set up a key below and you never type it.

## Enable SSH

Easiest at imaging time: in Raspberry Pi Imager, open the settings gear and
enable SSH, set the user, and set the hostname before writing the card.

On a board that is already running:

    sudo raspi-config    # Interface Options -> SSH -> enable

## Find the Pi

On the same network:

    ping gatr2.local

If `.local` does not resolve, get the IP from your router, or on the Pi:

    hostname -I

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
