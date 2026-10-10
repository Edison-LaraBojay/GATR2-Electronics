// win32_text.h
// UTF-8 and UTF-16 for Windows calls, and Windows error text. Windows only.

#pragma once
#if defined(_WIN32)

#include <string>
#include <string_view>

namespace spectagatr::receiver::platform
{

std::wstring toWide(std::string_view text);
std::string  toUtf8(std::wstring_view text);

// "Access is denied. (5)"
std::string win32ErrorText(unsigned long code);

} // namespace spectagatr::receiver::platform

#endif
