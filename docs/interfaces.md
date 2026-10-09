# Communication interfaces

The existing protocol is retained for communiGATR compatibility while its
replacement is designed. It describes the retired Pico/Pi/Brain architecture,
not an implemented new MCU link.

- [translaGATR](../translaGATR/README.md) defines the existing packets/codecs.
- [frames.h](../translaGATR/frames.h) defines message fields and units.
- [frame_codec.h](../translaGATR/frame_codec.h) defines framing/serialization.
- [link_documents.h](../translaGATR/link_documents.h) defines chunked documents.
- [Historical wire specification](../deprecated/docs/interfaces.md) preserves
  the detailed protocol and previous deployment assumptions.

Active planning/control interfaces are independent of that protocol:

- `investigatr::StateSource`: robot pose, age and validity/frame identity.
- `investigatr::FieldSource`: separately supplied Brain-owned field geometry.
- `investigatr::PlanRequest`: start/goal poses, motion model, and optional field.

The intended MCU estimates pose without storing the field. New wire messages,
transport/recovery behavior and firmware are future work.
