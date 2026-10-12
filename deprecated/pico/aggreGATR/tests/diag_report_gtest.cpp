// diag_report_gtest.cpp

#include <gtest/gtest.h>

#include <vector>

#include "diag_report.h"
#include "frame_codec.h"
#include "frames.h"

using namespace pilink;

TEST(DiagReport, ReadPinsSetsKnownForEveryListedPin) {
    const DiagPin pins[] = {
        {translagatr::kPicoPinImuInt, 22},
        {translagatr::kPicoPinEnc0A, 0},
        {translagatr::kPicoPinPiRx, 17},
    };
    std::vector<uint8_t> asked;
    uint16_t             levels = 0xFFFF, known = 0xFFFF;
    readPins(pins, 3,
             [&](uint8_t gpio) {
                 asked.push_back(gpio);
                 return gpio == 17 || gpio == 22;
             },
             levels, known);
    EXPECT_EQ(asked, (std::vector<uint8_t>{22, 0, 17}));
    EXPECT_EQ(known, translagatr::kPicoPinImuInt | translagatr::kPicoPinEnc0A |
                         translagatr::kPicoPinPiRx);
    EXPECT_EQ(levels, translagatr::kPicoPinImuInt | translagatr::kPicoPinPiRx);
}

TEST(DiagReport, NoPinsNothingKnown) {
    uint16_t levels = 1, known = 1;
    readPins(static_cast<const DiagPin*>(nullptr), 0, [](uint8_t) { return true; }, levels, known);
    EXPECT_EQ(levels, 0);
    EXPECT_EQ(known, 0);
}

TEST(DiagReport, FieldsPassThrough) {
    DiagInputs in;
    in.boot_id              = 0xBEEF;
    in.seq                  = 9;
    in.firmware             = translagatr::kPicoFirmwareAsm330;
    in.pins                 = 0x0401;
    in.pins_known           = 0x07FF;
    in.imu_present          = true;
    in.imu.rx               = 100;
    in.imu.bad              = 2;
    in.imu.resets           = 1;
    in.imu.error            = translagatr::kPicoImuReasonStream;
    in.imu.reports_ok       = 100;
    in.imu.reports_rejected = 0;
    in.imu.have_report      = true;
    in.imu.report_age_ms    = 7;
    in.link_rx_bad          = 3;
    in.ticks_skipped        = 4;

    const translagatr::PicoDiag d = makeDiag(in);
    EXPECT_EQ(d.version, translagatr::kPicoLinkVersion);
    EXPECT_EQ(d.boot_id, 0xBEEF);
    EXPECT_EQ(d.seq, 9);
    EXPECT_EQ(d.firmware, translagatr::kPicoFirmwareAsm330);
    EXPECT_EQ(d.pins, 0x0401);
    EXPECT_EQ(d.pins_known, 0x07FF);
    EXPECT_EQ(d.imu_rx, 100);
    EXPECT_EQ(d.imu_bad, 2);
    EXPECT_EQ(d.imu_resets, 1);
    EXPECT_EQ(d.imu_error, translagatr::kPicoImuReasonStream);
    EXPECT_EQ(d.reports_ok, 100);
    EXPECT_EQ(d.reports_rejected, 0);
    EXPECT_EQ(d.report_age_ms, 7);
    EXPECT_EQ(d.link_rx_bad, 3);
    EXPECT_EQ(d.ticks_skipped, 4);
    EXPECT_EQ(d.flags, translagatr::kPicoDiagImuPresent);
}

TEST(DiagReport, CountersWrapAndTheErrorSaturates) {
    DiagInputs in;
    in.imu.rx               = 0x12345678u;
    in.imu.bad              = 0x10000u;
    in.imu.resets           = 0x1FFu;
    in.imu.reports_ok       = 0xFFFFFu;
    in.imu.reports_rejected = 0x10001u;
    in.link_rx_bad          = 0x20003u;
    in.ticks_skipped        = 0xFFFFFFFFu;
    in.imu.error            = -1000;
    translagatr::PicoDiag d = makeDiag(in);
    EXPECT_EQ(d.imu_rx, 0x5678);
    EXPECT_EQ(d.imu_bad, 0);
    EXPECT_EQ(d.imu_resets, 0xFF);
    EXPECT_EQ(d.reports_ok, 0xFFFF);
    EXPECT_EQ(d.reports_rejected, 1);
    EXPECT_EQ(d.link_rx_bad, 3);
    EXPECT_EQ(d.ticks_skipped, 0xFFFF);
    EXPECT_EQ(d.imu_error, -128);
    EXPECT_EQ(d.flags, 0);

    in.imu.error = 1000;
    EXPECT_EQ(makeDiag(in).imu_error, 127);
    in.imu.error = -6;   // SH2_ERR_IO
    EXPECT_EQ(makeDiag(in).imu_error, -6);
}

TEST(DiagReport, ReportAgeNoneAndSaturation) {
    DiagInputs in;
    in.imu.have_report   = false;
    in.imu.report_age_ms = 5;
    EXPECT_EQ(makeDiag(in).report_age_ms, 0xFFFF);   // none, whatever the age says
    in.imu.have_report   = true;
    in.imu.report_age_ms = 0;
    EXPECT_EQ(makeDiag(in).report_age_ms, 0);
    in.imu.report_age_ms = 0xFFFE;
    EXPECT_EQ(makeDiag(in).report_age_ms, 0xFFFE);
    in.imu.report_age_ms = 0xFFFF;
    EXPECT_EQ(makeDiag(in).report_age_ms, 0xFFFE);   // never mistaken for none
    in.imu.report_age_ms = 0xFFFFFFFFu;
    EXPECT_EQ(makeDiag(in).report_age_ms, 0xFFFE);
}

TEST(DiagReport, EncodedFrameIsWhatTheCodecDecodes) {
    DiagInputs in;
    in.boot_id         = 0x1234;
    in.seq             = 200;
    in.firmware        = translagatr::kPicoFirmwareBno08x;
    in.pins            = translagatr::kPicoPinImuRst | translagatr::kPicoPinImuWake;
    in.pins_known      = 0x07FF;
    in.imu_present     = true;
    in.imu.error       = -3;
    in.imu.have_report = true;
    in.imu.report_age_ms = 12;
    uint8_t        buf[32];
    const uint16_t n = translagatr::encodePicoDiag(makeDiag(in), buf, sizeof(buf));
    ASSERT_EQ(n, 32);
    translagatr::PicoDiag back;
    ASSERT_TRUE(translagatr::decodePicoDiag(buf, n, back));
    EXPECT_EQ(back.boot_id, 0x1234);
    EXPECT_EQ(back.seq, 200);
    EXPECT_EQ(back.pins, in.pins);
    EXPECT_EQ(back.imu_error, -3);
    EXPECT_EQ(back.report_age_ms, 12);
}

TEST(DiagReport, CommandReaderStatsCountRejectedFrames) {
    // link_rx_bad comes from the Pi command FrameReader: a corrupted command
    // is one check error, noise is only dropped bytes.
    translagatr::PicoCommand c;
    c.op             = translagatr::kPicoOpDiagnostics;
    c.request_id     = 5;
    c.target_boot_id = 6;
    c.diag_hz        = 1;
    uint8_t        frame[translagatr::kMaxFrameLen];
    const uint16_t n = translagatr::encodePicoCommand(c, frame, sizeof(frame));
    ASSERT_EQ(n, 13);

    translagatr::FrameReader reader;
    int                      frames = 0;
    const auto push = [&](const uint8_t* bytes, uint16_t len) {
        for (uint16_t i = 0; i < len; ++i) {
            if (reader.push(bytes[i])) {
                do {
                    ++frames;
                } while (reader.next());
            }
        }
    };
    const uint8_t noise[] = {0x00, 0x13, 0x55};
    push(noise, sizeof(noise));
    uint8_t bad[13];
    for (int i = 0; i < 13; ++i) {
        bad[i] = frame[i];
    }
    bad[8] ^= 0x10;
    push(bad, 13);
    push(frame, n);
    EXPECT_EQ(frames, 1);
    EXPECT_EQ(reader.stats().check_errors + reader.stats().length_errors, 1u);
}
