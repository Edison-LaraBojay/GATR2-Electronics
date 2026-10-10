// static_site_gtest.cpp
// Static files from a temp web/ and fields/: MIME types, index.html, /api/fields, 404, and
// refused traversal.

#include "receiver/server/static_site.h"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>

namespace spectagatr::receiver::server
{
namespace
{

namespace fs = std::filesystem;

class Site : public ::testing::Test {
  protected:
    fs::path   root;
    SiteConfig site;

    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        root             = fs::temp_directory_path() / ("spectagatr-site-" + std::to_string(stamp));
        fs::create_directories(root / "web" / "js");
        fs::create_directories(root / "fields");
        write(root / "web" / "index.html", "<html>index</html>");
        write(root / "web" / "js" / "app.js", "export const a = 1;");
        write(root / "web" / "js" / "lib.mjs", "export {};");
        write(root / "web" / "style.css", "body{}");
        write(root / "web" / "logo.png", "PNG");
        write(root / "web" / "icon.svg", "<svg/>");
        write(root / "web" / "data.json", "{}");
        write(root / "web" / ".hidden", "no");
        write(root / "fields" / "override.json", "{\"name\":\"override\"}");
        write(root / "fields" / "empty.json", "{\"name\":\"empty\"}");
        write(root / "fields" / "README.md", "not a field");
        write(root / "secret.txt", "outside");
        site.web_dir    = root / "web";
        site.fields_dir = root / "fields";
    }

    void TearDown() override {
        std::error_code error;
        fs::remove_all(root, error);
    }

    static void write(const fs::path& file, const std::string& text) {
        std::ofstream(file, std::ios::binary) << text;
    }

    HttpResponse get(const std::string& path) const {
        HttpRequest request;
        request.method = "GET";
        request.path   = path;
        return serveSite(site, request);
    }
};

TEST_F(Site, RootServesIndexHtml) {
    const HttpResponse response = get("/");
    EXPECT_EQ(response.status, 200);
    EXPECT_EQ(response.body, "<html>index</html>");
    EXPECT_EQ(response.content_type, "text/html; charset=utf-8");
    EXPECT_EQ(response.headers.at("Cache-Control"), "no-store");
}

TEST_F(Site, MimeTypesByExtension) {
    EXPECT_EQ(get("/js/app.js").content_type, "text/javascript; charset=utf-8");
    EXPECT_EQ(get("/js/lib.mjs").content_type, "text/javascript; charset=utf-8");
    EXPECT_EQ(get("/style.css").content_type, "text/css; charset=utf-8");
    EXPECT_EQ(get("/logo.png").content_type, "image/png");
    EXPECT_EQ(get("/icon.svg").content_type, "image/svg+xml");
    EXPECT_EQ(get("/data.json").content_type, "application/json; charset=utf-8");
    EXPECT_EQ(get("/js/app.js").body, "export const a = 1;");
    EXPECT_EQ(mimeTypeOf("X.HTML"), "text/html; charset=utf-8");
    EXPECT_EQ(mimeTypeOf("a.bin"), "application/octet-stream");
}

TEST_F(Site, MissingIs404) {
    EXPECT_EQ(get("/nope.js").status, 404);
    EXPECT_EQ(get("/js/").status, 404); // no index.html there
    EXPECT_EQ(get("/fields/nope.json").status, 404);
}

TEST_F(Site, FieldsAreServedAndListed) {
    const HttpResponse field = get("/fields/override.json");
    EXPECT_EQ(field.status, 200);
    EXPECT_EQ(field.body, "{\"name\":\"override\"}");
    EXPECT_EQ(field.content_type, "application/json; charset=utf-8");
    const HttpResponse listing = get("/api/fields");
    EXPECT_EQ(listing.status, 200);
    EXPECT_EQ(listing.body, "{\"fields\":[\"empty\",\"override\"],\"default\":\"override\"}");
    EXPECT_EQ(get("/fields/README.md").status, 403); // only .json
    EXPECT_EQ(listFields(root / "absent").size(), 0u);
}

TEST_F(Site, TraversalIsRefused) {
    for (const char* path :
         {"/../secret.txt", "/js/../../secret.txt", "/..", "/js/..", "/fields/../secret.txt",
          "/fields/../../secret.txt", "/..\\secret.txt", "/js\\..\\..\\secret.txt",
          "/C:/Windows/win.ini", "/.hidden", "//etc/passwd", "/js//app.js", "/fields/sub/x.json"}) {
        const HttpResponse response = get(path);
        EXPECT_EQ(response.status, 403) << path;
        EXPECT_EQ(response.body.find("outside"), std::string::npos) << path;
    }
    EXPECT_EQ(get(std::string("/index.html\0.js", 15)).status, 403);
}

TEST(SafeRelativePath, KeepsPlainPaths) {
    const std::optional<fs::path> path = safeRelativePath("/js/app.js");
    ASSERT_TRUE(path);
    EXPECT_EQ(*path, fs::path("js") / "app.js");
    EXPECT_FALSE(safeRelativePath("relative.js"));
    EXPECT_FALSE(safeRelativePath(""));
}

} // namespace
} // namespace spectagatr::receiver::server
