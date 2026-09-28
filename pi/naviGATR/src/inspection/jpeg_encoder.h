// jpeg_encoder.h
// Grayscale JPEG previews from Y8 frames, for browser clients. Encoding
// runs on the inspection service thread, never on a worker; the source
// pixels are an immutable frame copy, so nothing here touches capture
// buffers. A preview wider than max_width is box-downscaled by an integer
// factor first, and the preview dimensions are reported so overlays can be
// scaled from full-resolution pixel coordinates exactly.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace navigatr
{

struct JpegPreview {
    std::vector<uint8_t> bytes;
    int                  width_px  = 0;   // preview dimensions
    int                  height_px = 0;
    int                  scale     = 1;   // source pixels per preview pixel
    double               encode_ms = 0.0;
};

// quality 1..100. max_width_px <= 0 means no downscale. False with err on
// bad input or encoder failure.
bool encodeY8Jpeg(const uint8_t* y8, int width_px, int height_px, int max_width_px,
                  int quality, JpegPreview& out, std::string& err);

} // namespace navigatr
