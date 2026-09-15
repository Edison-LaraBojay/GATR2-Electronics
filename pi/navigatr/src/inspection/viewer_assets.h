// viewer_assets.h
// The browser inspector compiled into the binary: viewer/ plus the pinned
// three.js files under vendor/. Generated at build time by
// cmake/embed_assets.cmake so the Pi serves the viewer with no filesystem
// dependency, no CDN and no development server.

#pragma once
#include <cstddef>

namespace navigatr
{

struct EmbeddedAsset {
    const char*          path;           // served path, e.g. "index.html", "vendor/three.module.min.js"
    const char*          content_type;
    const unsigned char* data;
    std::size_t          size;
};

// Table of every embedded file; count entries.
const EmbeddedAsset* embeddedViewerAssets(std::size_t& count);

} // namespace navigatr
