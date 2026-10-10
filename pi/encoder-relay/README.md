# encoder-relay

Pi relay for the encoder localization demo. It forwards Pico encoder records
from the UART to the V5 Brain USB user port. It does no localization.
It drains available UART input and forwards the newest complete record. Each
pass reads up to 32 KiB before servicing the Brain, so continuous input cannot
starve output. A partly sent line is finished first.
Build, service and usage: [brain/localization-demo/README.md](../../brain/localization-demo/README.md).
