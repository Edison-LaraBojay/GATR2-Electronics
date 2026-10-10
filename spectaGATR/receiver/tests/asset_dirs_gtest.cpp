// asset_dirs_gtest.cpp
// Where web/ and fields/ are found: flag, next to the program, install layout, source tree.

#include "receiver/app/asset_dirs.h"

#include <gtest/gtest.h>

#include <chrono>

namespace spectagatr::receiver::app
{
namespace
{

namespace fs = std::filesystem;

class Dirs : public ::testing::Test {
  protected:
    fs::path root;

    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        root             = fs::temp_directory_path() / ("spectagatr-dirs-" + std::to_string(stamp));
        fs::create_directories(root / "source" / "web");
        fs::create_directories(root / "source" / "fields");
        fs::create_directories(root / "prefix" / "bin");
    }

    void TearDown() override {
        std::error_code error;
        fs::remove_all(root, error);
    }

    AssetSearch search() const {
        AssetSearch s;
        s.executable_dir = root / "prefix" / "bin";
        s.source_web     = root / "source" / "web";
        s.source_fields  = root / "source" / "fields";
        return s;
    }
};

TEST_F(Dirs, SourceTreeWhenNothingElse) {
    const AssetDirs dirs = findAssetDirs(search());
    EXPECT_EQ(dirs.web, root / "source" / "web");
    EXPECT_EQ(dirs.fields, root / "source" / "fields");
}

TEST_F(Dirs, InstallLayoutBeatsTheSourceTree) {
    fs::create_directories(root / "prefix" / "share" / "spectagatr" / "web");
    fs::create_directories(root / "prefix" / "share" / "spectagatr" / "fields");
    const AssetDirs dirs = findAssetDirs(search());
    EXPECT_EQ(dirs.web, root / "prefix" / "share" / "spectagatr" / "web");
    EXPECT_EQ(dirs.fields, root / "prefix" / "share" / "spectagatr" / "fields");
}

TEST_F(Dirs, NextToTheProgramBeatsTheInstallLayout) {
    fs::create_directories(root / "prefix" / "share" / "spectagatr" / "web");
    fs::create_directories(root / "prefix" / "bin" / "web");
    EXPECT_EQ(findAssetDirs(search()).web, root / "prefix" / "bin" / "web");
}

TEST_F(Dirs, FlagsWinEvenWhenMissing) {
    AssetSearch s        = search();
    s.web_flag           = (root / "mine").string();
    s.fields_flag        = "f";
    const AssetDirs dirs = findAssetDirs(s);
    EXPECT_EQ(dirs.web, root / "mine");
    EXPECT_EQ(dirs.fields, fs::path("f"));
}

} // namespace
} // namespace spectagatr::receiver::app
