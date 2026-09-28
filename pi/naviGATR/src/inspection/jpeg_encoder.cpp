// jpeg_encoder.cpp
// Y8 to grayscale JPEG through the vendored stb_image_write, whose
// implementation lives in this translation unit only. The downscale is a
// plain box average by an integer factor so a preview pixel maps back to
// a whole number of source pixels and overlays scale exactly.

#include "inspection/jpeg_encoder.h"

#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include <stb_image_write.h>

namespace navigatr
{

namespace
{

void appendBytes(void* context, void* data, int size) {
    auto* out = static_cast<std::vector<uint8_t>*>(context);
    const auto* p = static_cast<const uint8_t*>(data);
    out->insert(out->end(), p, p + size);
}

// Average every factor x factor block; trailing rows and columns that do
// not fill a block are dropped so the scale stays exact.
void boxDownscale(const uint8_t* src, int width, int height, int factor,
                  std::vector<uint8_t>& dst, int& out_w, int& out_h) {
    out_w = width / factor;
    out_h = height / factor;
    dst.resize(static_cast<std::size_t>(out_w) * out_h);
    const unsigned area = static_cast<unsigned>(factor) * factor;
    for (int y = 0; y < out_h; ++y) {
        for (int x = 0; x < out_w; ++x) {
            unsigned sum = 0;
            for (int dy = 0; dy < factor; ++dy) {
                const uint8_t* row = src + static_cast<std::size_t>(y * factor + dy) * width +
                                     static_cast<std::size_t>(x) * factor;
                for (int dx = 0; dx < factor; ++dx) {
                    sum += row[dx];
                }
            }
            dst[static_cast<std::size_t>(y) * out_w + x] =
                static_cast<uint8_t>((sum + area / 2) / area);
        }
    }
}

} // namespace

bool encodeY8Jpeg(const uint8_t* y8, int width_px, int height_px, int max_width_px,
                  int quality, JpegPreview& out, std::string& err) {
    if (y8 == nullptr || width_px <= 0 || height_px <= 0) {
        err = "jpeg: empty frame";
        return false;
    }
    if (quality < 1 || quality > 100) {
        err = "jpeg: quality must be 1..100";
        return false;
    }
    const auto t0 = std::chrono::steady_clock::now();

    int factor = 1;
    if (max_width_px > 0 && width_px > max_width_px) {
        factor = (width_px + max_width_px - 1) / max_width_px;
    }
    std::vector<uint8_t> scaled;
    const uint8_t*       pixels = y8;
    int                  w      = width_px;
    int                  h      = height_px;
    if (factor > 1) {
        boxDownscale(y8, width_px, height_px, factor, scaled, w, h);
        if (w <= 0 || h <= 0) {
            err = "jpeg: frame too small for the requested width";
            return false;
        }
        pixels = scaled.data();
    }

    out.bytes.clear();
    out.bytes.reserve(static_cast<std::size_t>(w) * h / 4);
    if (stbi_write_jpg_to_func(appendBytes, &out.bytes, w, h, 1, pixels, quality) == 0) {
        err = "jpeg: encoder failed";
        out.bytes.clear();
        return false;
    }
    out.width_px  = w;
    out.height_px = h;
    out.scale     = factor;
    out.encode_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return true;
}

} // namespace navigatr
