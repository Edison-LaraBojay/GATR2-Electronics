// process.h
// Process-level OS pieces: Ctrl+C, the executable's own path, raw bytes from stdin.

#pragma once
#include <cstddef>
#include <cstdint>
#include <filesystem>

namespace spectagatr::receiver::platform
{

using Path = std::filesystem::path;

// Ctrl+C (and SIGTERM, console close) set a flag the main loop polls. POSIX also ignores SIGPIPE.
void installInterruptHandler();
bool interruptRequested();

// This executable, absolute; empty when the OS will not say.
Path executablePath();

// stdin as raw bytes: no newline translation, no Ctrl+Z end on Windows. Call once first.
void useBinaryStandardInput();

// Blocks until some bytes are in: the count, 0 at end of input, -1 on an error.
long readStandardInput(std::uint8_t* destination, std::size_t capacity);

} // namespace spectagatr::receiver::platform
