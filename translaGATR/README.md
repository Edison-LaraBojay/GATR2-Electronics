# translaGATR

Legacy wire formats and codecs from the former Pico/Pi/Brain system. This is
the byte-packet library, not a hardware driver or path planner:

- `frames.h`: message types, packet fields and units.
- `frame_codec.*`: bounded framing, encoding, decoding and checksums.
- `link_documents.*`: robot profiles, field maps and chunked documents.
- `tests/`: host tests for those formats.

It remains because the retained communiGATR client uses it. Active aggreGATR
acquisition and investiGATR/actuGATR do not need this old protocol. The schema
is specific to the retired Pi architecture; it is not the future MCU-to-Brain
contract. Generic framing may be reused during redesign.

See the [legacy specification](../docs/interfaces.md) and
[deprecated system](../deprecated/README.md).
