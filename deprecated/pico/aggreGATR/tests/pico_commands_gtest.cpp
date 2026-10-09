// pico_commands_gtest.cpp

#include <gtest/gtest.h>

#include <vector>

#include "frame_codec.h"
#include "frames.h"
#include "pico_commands.h"

using namespace pilink;

namespace
{

constexpr uint16_t kBoot = 0x3C5A;

using Bytes = std::vector<uint8_t>;

Bytes commandFrame(uint8_t op, uint16_t id, uint8_t body = 0, uint16_t target = kBoot) {
    translagatr::PicoCommand c;
    c.op             = op;
    c.request_id     = id;
    c.target_boot_id = target;
    c.imu_enabled    = body;
    c.imu_port       = body;
    c.diag_hz        = body;
    Bytes          out(translagatr::kMaxFrameLen);
    const uint16_t n = translagatr::encodePicoCommand(c, out.data(), static_cast<uint16_t>(out.size()));
    out.resize(n);
    return out;
}

// A command frame around an arbitrary payload, with a valid CRC.
Bytes rawFrame(const Bytes& payload) {
    Bytes out = {translagatr::kSync0, translagatr::kSync1, translagatr::kFramePicoCommand,
                 static_cast<uint8_t>(payload.size())};
    out.insert(out.end(), payload.begin(), payload.end());
    const uint16_t crc = translagatr::crc16(out.data() + 2, static_cast<uint16_t>(payload.size() + 2));
    out.push_back(static_cast<uint8_t>(crc));
    out.push_back(static_cast<uint8_t>(crc >> 8));
    return out;
}

// The firmware side: applies effects and reports IMU state like main.cpp.
struct Pico {
    Commands commands;
    bool     imu_enabled = true;
    uint8_t  imu_state   = translagatr::kPicoImuReady;
    uint8_t  acq_epoch   = 0;
    int      imu_reinits = 0;
    int      restarts    = 0;
    int      answered    = 0;
    uint8_t  diag_hz     = 0;
    int      diag_sets   = 0;

    Outcome send(const Bytes& frame) {
        const Outcome out =
            commands.receive(frame.data(), static_cast<uint16_t>(frame.size()), kBoot, imu_enabled);
        switch (out.effect) {
        case Effect::None:
            break;
        case Effect::EnableImu:
            imu_enabled = true;
            imu_state   = translagatr::kPicoImuInitializing;
            break;
        case Effect::DisableImu:
            imu_enabled = false;
            imu_state   = translagatr::kPicoImuDisabled;
            break;
        case Effect::ReinitImu:
            ++imu_reinits;
            imu_state = translagatr::kPicoImuInitializing;
            break;
        case Effect::RestartAcquisition:
            ++restarts;
            ++acq_epoch;
            break;
        case Effect::SetDiagnostics:
            diag_hz = out.diag_hz;
            ++diag_sets;
            break;
        }
        if (out.answered) {
            ++answered;
        }
        commands.imuProgress(imu_state);
        return out;
    }

    translagatr::PicoStatus status() const {
        translagatr::PicoStatus s;
        commands.fill(s);
        return s;
    }
};

void expectLast(const Pico& p, uint16_t id, uint8_t op, uint8_t status, uint8_t detail) {
    const translagatr::PicoStatus s = p.status();
    EXPECT_EQ(s.last_request_id, id);
    EXPECT_EQ(s.last_op, op);
    EXPECT_EQ(s.last_status, status);
    EXPECT_EQ(s.last_detail, detail);
}

} // namespace

TEST(PicoCommands, NoCommandYet) {
    Pico p;
    expectLast(p, 0, 0, translagatr::kPicoCommandNone, translagatr::kPicoDetailNone);
}

TEST(PicoCommands, RestartAcquisitionRunsOnceAcrossLostAcknowledgements) {
    Pico        p;
    const Bytes f = commandFrame(translagatr::kPicoOpRestartAcquisition, 41);
    EXPECT_EQ(p.send(f).effect, Effect::RestartAcquisition);
    expectLast(p, 41, translagatr::kPicoOpRestartAcquisition, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    // The status carrying the completion was lost; the Pi resends.
    for (int i = 0; i < 5; ++i) {
        const Outcome out = p.send(f);
        EXPECT_TRUE(out.answered);
        EXPECT_EQ(out.effect, Effect::None);
    }
    EXPECT_EQ(p.restarts, 1);
    EXPECT_EQ(p.acq_epoch, 1);
    EXPECT_EQ(p.commands.duplicates(), 5u);
    expectLast(p, 41, translagatr::kPicoOpRestartAcquisition, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    // A new request id is a new restart.
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 42)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.acq_epoch, 2);
}

TEST(PicoCommands, ConfigureChangesOnlyWhatDiffers) {
    Pico p;
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpConfigure, 1, 1)).effect, Effect::None);
    expectLast(p, 1, translagatr::kPicoOpConfigure, translagatr::kPicoCommandCompleted, translagatr::kPicoDetailNone);

    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpConfigure, 2, 0)).effect, Effect::DisableImu);
    EXPECT_FALSE(p.imu_enabled);
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpConfigure, 3, 0)).effect, Effect::None);
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpConfigure, 4, 1)).effect, Effect::EnableImu);
    EXPECT_TRUE(p.imu_enabled);

    // A resend of the disable after the enable changes nothing.
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpConfigure, 2, 0)).effect, Effect::None);
    EXPECT_TRUE(p.imu_enabled);
}

TEST(PicoCommands, ConfigureRejectsAnUnknownSetting) {
    Pico        p;
    const Bytes f = commandFrame(translagatr::kPicoOpConfigure, 5, 2);
    EXPECT_EQ(p.send(f).effect, Effect::None);
    expectLast(p, 5, translagatr::kPicoOpConfigure, translagatr::kPicoCommandFailed, translagatr::kPicoDetailBadBody);
    EXPECT_TRUE(p.imu_enabled);
}

TEST(PicoCommands, ReinitRunsUntilTheImuIsReady) {
    Pico        p;
    const Bytes f = commandFrame(translagatr::kPicoOpReinitImu, 10, 0);
    EXPECT_EQ(p.send(f).effect, Effect::ReinitImu);
    expectLast(p, 10, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandRunning, translagatr::kPicoDetailNone);

    EXPECT_FALSE(p.commands.imuProgress(translagatr::kPicoImuInitializing));
    EXPECT_FALSE(p.commands.imuProgress(translagatr::kPicoImuRetrying));
    EXPECT_FALSE(p.commands.imuProgress(translagatr::kPicoImuAligning));
    // A resend while running reports running and does not restart the IMU.
    EXPECT_EQ(p.send(f).effect, Effect::None);
    expectLast(p, 10, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandRunning, translagatr::kPicoDetailNone);

    p.imu_state = translagatr::kPicoImuReady;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    EXPECT_FALSE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 10, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    EXPECT_EQ(p.send(f).effect, Effect::None);
    EXPECT_EQ(p.imu_reinits, 1);
}

TEST(PicoCommands, ReinitFailsWhenTheImuGivesUp) {
    Pico p;
    p.send(commandFrame(translagatr::kPicoOpReinitImu, 11, 0));
    p.imu_state = translagatr::kPicoImuFailed;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 11, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailImuAbsent);
}

TEST(PicoCommands, ReinitFailsWhenDisabledMeanwhile) {
    Pico        p;
    const Bytes reinit = commandFrame(translagatr::kPicoOpReinitImu, 12, 0);
    p.send(reinit);
    p.send(commandFrame(translagatr::kPicoOpConfigure, 13, 0));
    expectLast(p, 13, translagatr::kPicoOpConfigure, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    // The older request is reported when the Pi resends it.
    EXPECT_EQ(p.send(reinit).effect, Effect::None);
    expectLast(p, 12, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailImuDisabled);
}

TEST(PicoCommands, ReinitRefusedWhileDisabledOrForAnotherPort) {
    Pico p;
    p.send(commandFrame(translagatr::kPicoOpReinitImu, 20, 1));
    expectLast(p, 20, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailNoSuchPort);

    p.send(commandFrame(translagatr::kPicoOpConfigure, 21, 0));
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpReinitImu, 22, 0)).effect, Effect::None);
    expectLast(p, 22, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailImuDisabled);
    EXPECT_EQ(p.imu_reinits, 0);
}

TEST(PicoCommands, TwoRunningReinitsSettleTogether) {
    Pico        p;
    const Bytes first = commandFrame(translagatr::kPicoOpReinitImu, 30, 0);
    p.send(first);
    p.send(commandFrame(translagatr::kPicoOpReinitImu, 31, 0));
    EXPECT_EQ(p.imu_reinits, 2);
    p.imu_state = translagatr::kPicoImuReady;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 31, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);
    p.send(first);
    expectLast(p, 30, translagatr::kPicoOpReinitImu, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);
}

TEST(PicoCommands, WrongTargetFailsAndIsNotRecorded) {
    Pico          p;
    const Bytes   stale = commandFrame(translagatr::kPicoOpRestartAcquisition, 50, 0, kBoot ^ 0x0101);
    const Outcome out   = p.send(stale);
    EXPECT_TRUE(out.answered);
    EXPECT_EQ(out.effect, Effect::None);
    expectLast(p, 50, translagatr::kPicoOpRestartAcquisition, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailWrongTarget);

    // Target 0 never matches: boot ids are nonzero.
    p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 51, 0, 0));
    expectLast(p, 51, translagatr::kPicoOpRestartAcquisition, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailWrongTarget);

    // The same id retargeted to this boot runs.
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 50)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.restarts, 1);
}

TEST(PicoCommands, UnknownOpFails) {
    Pico p;
    EXPECT_EQ(p.send(commandFrame(9, 60)).effect, Effect::None);
    expectLast(p, 60, 9, translagatr::kPicoCommandFailed, translagatr::kPicoDetailUnknownOp);

    // Unknown ops may carry a body.
    const Bytes with_body =
        rawFrame({translagatr::kPicoLinkVersion, 200, 61, 0, static_cast<uint8_t>(kBoot),
                  static_cast<uint8_t>(kBoot >> 8), 7, 7});
    EXPECT_TRUE(p.send(with_body).answered);
    expectLast(p, 61, 200, translagatr::kPicoCommandFailed, translagatr::kPicoDetailUnknownOp);
}

TEST(PicoCommands, BodyLengthWrongForTheOpFails) {
    Pico p;
    // CONFIGURE without its body byte.
    const Bytes short_configure =
        rawFrame({translagatr::kPicoLinkVersion, translagatr::kPicoOpConfigure, 70, 0,
                  static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8)});
    EXPECT_EQ(p.send(short_configure).effect, Effect::None);
    expectLast(p, 70, translagatr::kPicoOpConfigure, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailBadBody);

    // RESTART_ACQUISITION with a stray byte never restarts.
    const Bytes long_restart =
        rawFrame({translagatr::kPicoLinkVersion, translagatr::kPicoOpRestartAcquisition, 71, 0,
                  static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8), 0});
    EXPECT_EQ(p.send(long_restart).effect, Effect::None);
    expectLast(p, 71, translagatr::kPicoOpRestartAcquisition, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailBadBody);
    EXPECT_EQ(p.restarts, 0);
}

TEST(PicoCommands, OtherVersionsZeroIdsAndCorruptFramesAreIgnored) {
    Pico  p;
    Bytes other_version = rawFrame({static_cast<uint8_t>(translagatr::kPicoLinkVersion + 1),
                                    translagatr::kPicoOpRestartAcquisition, 80, 0,
                                    static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8)});
    EXPECT_FALSE(p.send(other_version).answered);

    EXPECT_FALSE(p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 0)).answered);

    Bytes corrupt = commandFrame(translagatr::kPicoOpRestartAcquisition, 81);
    corrupt[5] ^= 0x40;
    EXPECT_FALSE(p.send(corrupt).answered);

    EXPECT_EQ(p.restarts, 0);
    EXPECT_EQ(p.commands.ignored(), 3u);
    expectLast(p, 0, 0, translagatr::kPicoCommandNone, translagatr::kPicoDetailNone);
}

TEST(PicoCommands, SameIdWithAnotherOpIsANewCommand) {
    Pico p;
    p.send(commandFrame(translagatr::kPicoOpConfigure, 90, 1));
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 90)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpRestartAcquisition, 90)).effect, Effect::None);
    EXPECT_EQ(p.restarts, 1);
}

TEST(PicoCommands, DuplicatesAreRememberedForTheLastFourCommands) {
    Pico        p;
    const Bytes oldest = commandFrame(translagatr::kPicoOpRestartAcquisition, 100);
    p.send(oldest);
    for (uint16_t id = 101; id <= 103; ++id) {
        p.send(commandFrame(translagatr::kPicoOpConfigure, id, 1));
    }
    EXPECT_EQ(p.send(oldest).effect, Effect::None);

    p.send(commandFrame(translagatr::kPicoOpConfigure, 104, 1));
    EXPECT_EQ(p.send(oldest).effect, Effect::RestartAcquisition);
    EXPECT_EQ(p.restarts, 2);
}

TEST(PicoCommands, StatusFrameCarriesTheLastCommand) {
    Pico p;
    p.send(commandFrame(translagatr::kPicoOpReinitImu, 0xBEEF, 0));

    translagatr::PicoStatus st = p.status();
    st.boot_id           = kBoot;
    st.imu_state         = translagatr::kPicoImuInitializing;
    uint8_t        buf[translagatr::kMaxFrameLen];
    const uint16_t n = translagatr::encodePicoStatus(st, buf, sizeof(buf));
    ASSERT_EQ(n, translagatr::kPicoStatusLen + translagatr::kLinkEnvelopeLen);

    translagatr::PicoStatus back;
    ASSERT_TRUE(translagatr::decodePicoStatus(buf, n, back));
    EXPECT_EQ(back.boot_id, kBoot);
    EXPECT_EQ(back.last_request_id, 0xBEEF);
    EXPECT_EQ(back.last_op, translagatr::kPicoOpReinitImu);
    EXPECT_EQ(back.last_status, translagatr::kPicoCommandRunning);
}

TEST(PicoCommands, FrameReaderDeliversCommandsFromAByteStream) {
    Pico               p;
    translagatr::FrameReader reader;
    Bytes              stream = {0x00, 0xAA, 0x13};
    const Bytes        a      = commandFrame(translagatr::kPicoOpConfigure, 110, 0);
    const Bytes        b      = commandFrame(translagatr::kPicoOpRestartAcquisition, 111);
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());

    int frames = 0;
    for (uint8_t byte : stream) {
        if (!reader.push(byte)) {
            continue;
        }
        do {
            ASSERT_EQ(reader.frameType(), translagatr::kFramePicoCommand);
            p.send(Bytes(reader.frame(), reader.frame() + reader.frameLen()));
            ++frames;
        } while (reader.next());
    }
    EXPECT_EQ(frames, 2);
    EXPECT_FALSE(p.imu_enabled);
    EXPECT_EQ(p.restarts, 1);
}

TEST(PicoCommands, DiagnosticsSetsTheRate) {
    Pico          p;
    const Outcome on = p.send(commandFrame(translagatr::kPicoOpDiagnostics, 120, 1));
    EXPECT_TRUE(on.answered);
    EXPECT_EQ(on.effect, Effect::SetDiagnostics);
    EXPECT_EQ(on.diag_hz, 1);
    EXPECT_EQ(p.diag_hz, 1);
    expectLast(p, 120, translagatr::kPicoOpDiagnostics, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpDiagnostics, 121, kMaxDiagHz)).diag_hz,
              kMaxDiagHz);
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpDiagnostics, 122, 0)).effect,
              Effect::SetDiagnostics);
    EXPECT_EQ(p.diag_hz, 0);
    EXPECT_EQ(p.diag_sets, 3);
}

TEST(PicoCommands, DiagnosticsRatesAboveTheMaximumAreBadBody) {
    for (int hz = kMaxDiagHz + 1; hz < 256; ++hz) {
        Pico          p;
        const Outcome out =
            p.send(commandFrame(translagatr::kPicoOpDiagnostics, 130, static_cast<uint8_t>(hz)));
        EXPECT_TRUE(out.answered) << hz;
        EXPECT_EQ(out.effect, Effect::None) << hz;
        EXPECT_EQ(p.diag_sets, 0) << hz;
        expectLast(p, 130, translagatr::kPicoOpDiagnostics, translagatr::kPicoCommandFailed,
                   translagatr::kPicoDetailBadBody);
    }
}

TEST(PicoCommands, DiagnosticsResendIsADuplicate) {
    Pico        p;
    const Bytes on = commandFrame(translagatr::kPicoOpDiagnostics, 140, 2);
    p.send(on);
    // A lost status frame: the Pi resends the same request.
    const Outcome again = p.send(on);
    EXPECT_TRUE(again.answered);
    EXPECT_EQ(again.effect, Effect::None);
    EXPECT_EQ(p.diag_sets, 1);
    EXPECT_EQ(p.commands.duplicates(), 1u);
    expectLast(p, 140, translagatr::kPicoOpDiagnostics, translagatr::kPicoCommandCompleted,
               translagatr::kPicoDetailNone);

    // The same id with another rate is not a duplicate: it runs.
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpDiagnostics, 140, 3)).effect,
              Effect::SetDiagnostics);
    EXPECT_EQ(p.diag_hz, 3);
}

TEST(PicoCommands, DiagnosticsForAnotherBootFails) {
    Pico p;
    EXPECT_EQ(p.send(commandFrame(translagatr::kPicoOpDiagnostics, 150, 1, kBoot ^ 1)).effect,
              Effect::None);
    expectLast(p, 150, translagatr::kPicoOpDiagnostics, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailWrongTarget);
    EXPECT_EQ(p.diag_sets, 0);
}

TEST(PicoCommands, DiagnosticsWithoutItsBodyByteIsBadBody) {
    Pico        p;
    const Bytes header_only =
        rawFrame({translagatr::kPicoLinkVersion, translagatr::kPicoOpDiagnostics, 160, 0,
                  static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8)});
    EXPECT_EQ(p.send(header_only).effect, Effect::None);
    expectLast(p, 160, translagatr::kPicoOpDiagnostics, translagatr::kPicoCommandFailed,
               translagatr::kPicoDetailBadBody);
}
