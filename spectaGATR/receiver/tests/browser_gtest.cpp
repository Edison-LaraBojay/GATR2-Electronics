// browser_gtest.cpp
// What may be handed to the OS opener, and the Linux launch rule. Never opens a browser.

#include "receiver/platform/browser.h"

#include <gtest/gtest.h>
#include <string>

namespace spectagatr::receiver::platform
{
namespace
{

TEST(Browser, OnlyPlainHttpUrls) {
    EXPECT_TRUE(browserUrlAllowed("http://127.0.0.1:8765/"));
    EXPECT_TRUE(browserUrlAllowed("https://pi.local:8765/view?src=brain"));
    EXPECT_FALSE(browserUrlAllowed("file:///C:/Windows/System32/calc.exe"));
    EXPECT_FALSE(browserUrlAllowed("C:\\Windows\\System32\\calc.exe"));
    EXPECT_FALSE(browserUrlAllowed("http://127.0.0.1/ --flag"));
    EXPECT_FALSE(browserUrlAllowed("http://127.0.0.1/\"x"));
    EXPECT_FALSE(browserUrlAllowed("http://127.0.0.1/'x"));
    EXPECT_FALSE(browserUrlAllowed("http://127.0.0.1/`x`"));
    EXPECT_FALSE(browserUrlAllowed("http://127.0.0.1/\nx"));
    EXPECT_FALSE(browserUrlAllowed(""));
}

TEST(Browser, RefusedUrlNeverLaunches) {
    std::string err;
    EXPECT_FALSE(openBrowser("javascript:alert(1)", err));
    EXPECT_EQ(err, "not an http URL: javascript:alert(1)");
}

TEST(Browser, LinuxNeedsADesktopAndNoRoot) {
    std::string    err;
    DesktopSession session;
    EXPECT_FALSE(linuxLaunchAllowed(session, err));
    EXPECT_NE(err.find("no desktop session"), std::string::npos);

    session.display = true;
    EXPECT_TRUE(linuxLaunchAllowed(session, err));

    session.root = true;
    EXPECT_FALSE(linuxLaunchAllowed(session, err));
    EXPECT_EQ(err, "not opening a browser as root");
}

} // namespace
} // namespace spectagatr::receiver::platform
