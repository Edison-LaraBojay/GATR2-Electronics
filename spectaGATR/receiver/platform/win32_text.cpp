// win32_text.cpp

#if defined(_WIN32)

#include "receiver/platform/win32_text.h"

#include <array>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace spectagatr::receiver::platform
{

std::wstring toWide(std::string_view text) {
    if (text.empty()) {
        return {};
    }
    const int size = static_cast<int>(text.size());
    const int wide = MultiByteToWideChar(CP_UTF8, 0, text.data(), size, nullptr, 0);
    if (wide <= 0) {
        return {};
    }
    std::wstring out(static_cast<std::size_t>(wide), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.data(), size, out.data(), wide);
    return out;
}

std::string toUtf8(std::wstring_view text) {
    if (text.empty()) {
        return {};
    }
    const int size = static_cast<int>(text.size());
    const int narrow =
        WideCharToMultiByte(CP_UTF8, 0, text.data(), size, nullptr, 0, nullptr, nullptr);
    if (narrow <= 0) {
        return {};
    }
    std::string out(static_cast<std::size_t>(narrow), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.data(), size, out.data(), narrow, nullptr, nullptr);
    return out;
}

std::string win32ErrorText(unsigned long code) {
    std::array<wchar_t, 256> buffer{};
    const DWORD              length =
        FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, code, 0,
                       buffer.data(), static_cast<DWORD>(buffer.size()), nullptr);
    std::string text = toUtf8(std::wstring_view(buffer.data(), length));
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) {
        text.pop_back();
    }
    if (text.empty()) {
        text = "error";
    }
    return text + " (" + std::to_string(code) + ")";
}

} // namespace spectagatr::receiver::platform

#endif
