# Bench tools

Standalone utilities for hardware bring-up and diagnosing physical links.
These checks run on the connected hardware and are separate from the production
firmware and host CI tests.

- [RS-485 link test](rs485_link/README.md): sends text from the Pi and displays it
  on the V5 Brain to check byte transfer. It does not exercise the framed
  command/pose protocol or reverse-direction traffic.
