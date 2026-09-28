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
    gatr2::PicoCommand c;
    c.op             = op;
    c.request_id     = id;
    c.target_boot_id = target;
    c.imu_enabled    = body;
    c.imu_port       = body;
    Bytes          out(gatr2::kMaxFrameLen);
    const uint16_t n = gatr2::encodePicoCommand(c, out.data(), static_cast<uint16_t>(out.size()));
    out.resize(n);
    return out;
}

// A command frame around an arbitrary payload, with a valid CRC.
Bytes rawFrame(const Bytes& payload) {
    Bytes out = {gatr2::kSync0, gatr2::kSync1, gatr2::kFramePicoCommand,
                 static_cast<uint8_t>(payload.size())};
    out.insert(out.end(), payload.begin(), payload.end());
    const uint16_t crc = gatr2::crc16(out.data() + 2, static_cast<uint16_t>(payload.size() + 2));
    out.push_back(static_cast<uint8_t>(crc));
    out.push_back(static_cast<uint8_t>(crc >> 8));
    return out;
}

// The firmware side: applies effects and reports IMU state like main.cpp.
struct Pico {
    Commands commands;
    bool     imu_enabled = true;
    uint8_t  imu_state   = gatr2::kPicoImuReady;
    uint8_t  acq_epoch   = 0;
    int      imu_reinits = 0;
    int      restarts    = 0;
    int      answered    = 0;

    Outcome send(const Bytes& frame) {
        const Outcome out =
            commands.receive(frame.data(), static_cast<uint16_t>(frame.size()), kBoot, imu_enabled);
        switch (out.effect) {
        case Effect::None:
            break;
        case Effect::EnableImu:
            imu_enabled = true;
            imu_state   = gatr2::kPicoImuInitializing;
            break;
        case Effect::DisableImu:
            imu_enabled = false;
            imu_state   = gatr2::kPicoImuDisabled;
            break;
        case Effect::ReinitImu:
            ++imu_reinits;
            imu_state = gatr2::kPicoImuInitializing;
            break;
        case Effect::RestartAcquisition:
            ++restarts;
            ++acq_epoch;
            break;
        }
        if (out.answered) {
            ++answered;
        }
        commands.imuProgress(imu_state);
        return out;
    }

    gatr2::PicoStatus status() const {
        gatr2::PicoStatus s;
        commands.fill(s);
        return s;
    }
};

void expectLast(const Pico& p, uint16_t id, uint8_t op, uint8_t status, uint8_t detail) {
    const gatr2::PicoStatus s = p.status();
    EXPECT_EQ(s.last_request_id, id);
    EXPECT_EQ(s.last_op, op);
    EXPECT_EQ(s.last_status, status);
    EXPECT_EQ(s.last_detail, detail);
}

} // namespace

TEST(PicoCommands, NoCommandYet) {
    Pico p;
    expectLast(p, 0, 0, gatr2::kPicoCommandNone, gatr2::kPicoDetailNone);
}

TEST(PicoCommands, RestartAcquisitionRunsOnceAcrossLostAcknowledgements) {
    Pico        p;
    const Bytes f = commandFrame(gatr2::kPicoOpRestartAcquisition, 41);
    EXPECT_EQ(p.send(f).effect, Effect::RestartAcquisition);
    expectLast(p, 41, gatr2::kPicoOpRestartAcquisition, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);

    // The status carrying the completion was lost; the Pi resends.
    for (int i = 0; i < 5; ++i) {
        const Outcome out = p.send(f);
        EXPECT_TRUE(out.answered);
        EXPECT_EQ(out.effect, Effect::None);
    }
    EXPECT_EQ(p.restarts, 1);
    EXPECT_EQ(p.acq_epoch, 1);
    EXPECT_EQ(p.commands.duplicates(), 5u);
    expectLast(p, 41, gatr2::kPicoOpRestartAcquisition, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);

    // A new request id is a new restart.
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 42)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.acq_epoch, 2);
}

TEST(PicoCommands, ConfigureChangesOnlyWhatDiffers) {
    Pico p;
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpConfigure, 1, 1)).effect, Effect::None);
    expectLast(p, 1, gatr2::kPicoOpConfigure, gatr2::kPicoCommandCompleted, gatr2::kPicoDetailNone);

    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpConfigure, 2, 0)).effect, Effect::DisableImu);
    EXPECT_FALSE(p.imu_enabled);
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpConfigure, 3, 0)).effect, Effect::None);
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpConfigure, 4, 1)).effect, Effect::EnableImu);
    EXPECT_TRUE(p.imu_enabled);

    // A resend of the disable after the enable changes nothing.
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpConfigure, 2, 0)).effect, Effect::None);
    EXPECT_TRUE(p.imu_enabled);
}

TEST(PicoCommands, ConfigureRejectsAnUnknownSetting) {
    Pico        p;
    const Bytes f = commandFrame(gatr2::kPicoOpConfigure, 5, 2);
    EXPECT_EQ(p.send(f).effect, Effect::None);
    expectLast(p, 5, gatr2::kPicoOpConfigure, gatr2::kPicoCommandFailed, gatr2::kPicoDetailBadBody);
    EXPECT_TRUE(p.imu_enabled);
}

TEST(PicoCommands, ReinitRunsUntilTheImuIsReady) {
    Pico        p;
    const Bytes f = commandFrame(gatr2::kPicoOpReinitImu, 10, 0);
    EXPECT_EQ(p.send(f).effect, Effect::ReinitImu);
    expectLast(p, 10, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandRunning, gatr2::kPicoDetailNone);

    EXPECT_FALSE(p.commands.imuProgress(gatr2::kPicoImuInitializing));
    EXPECT_FALSE(p.commands.imuProgress(gatr2::kPicoImuRetrying));
    EXPECT_FALSE(p.commands.imuProgress(gatr2::kPicoImuAligning));
    // A resend while running reports running and does not restart the IMU.
    EXPECT_EQ(p.send(f).effect, Effect::None);
    expectLast(p, 10, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandRunning, gatr2::kPicoDetailNone);

    p.imu_state = gatr2::kPicoImuReady;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    EXPECT_FALSE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 10, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);

    EXPECT_EQ(p.send(f).effect, Effect::None);
    EXPECT_EQ(p.imu_reinits, 1);
}

TEST(PicoCommands, ReinitFailsWhenTheImuGivesUp) {
    Pico p;
    p.send(commandFrame(gatr2::kPicoOpReinitImu, 11, 0));
    p.imu_state = gatr2::kPicoImuFailed;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 11, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailImuAbsent);
}

TEST(PicoCommands, ReinitFailsWhenDisabledMeanwhile) {
    Pico        p;
    const Bytes reinit = commandFrame(gatr2::kPicoOpReinitImu, 12, 0);
    p.send(reinit);
    p.send(commandFrame(gatr2::kPicoOpConfigure, 13, 0));
    expectLast(p, 13, gatr2::kPicoOpConfigure, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);

    // The older request is reported when the Pi resends it.
    EXPECT_EQ(p.send(reinit).effect, Effect::None);
    expectLast(p, 12, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailImuDisabled);
}

TEST(PicoCommands, ReinitRefusedWhileDisabledOrForAnotherPort) {
    Pico p;
    p.send(commandFrame(gatr2::kPicoOpReinitImu, 20, 1));
    expectLast(p, 20, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailNoSuchPort);

    p.send(commandFrame(gatr2::kPicoOpConfigure, 21, 0));
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpReinitImu, 22, 0)).effect, Effect::None);
    expectLast(p, 22, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailImuDisabled);
    EXPECT_EQ(p.imu_reinits, 0);
}

TEST(PicoCommands, TwoRunningReinitsSettleTogether) {
    Pico        p;
    const Bytes first = commandFrame(gatr2::kPicoOpReinitImu, 30, 0);
    p.send(first);
    p.send(commandFrame(gatr2::kPicoOpReinitImu, 31, 0));
    EXPECT_EQ(p.imu_reinits, 2);
    p.imu_state = gatr2::kPicoImuReady;
    EXPECT_TRUE(p.commands.imuProgress(p.imu_state));
    expectLast(p, 31, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);
    p.send(first);
    expectLast(p, 30, gatr2::kPicoOpReinitImu, gatr2::kPicoCommandCompleted,
               gatr2::kPicoDetailNone);
}

TEST(PicoCommands, WrongTargetFailsAndIsNotRecorded) {
    Pico          p;
    const Bytes   stale = commandFrame(gatr2::kPicoOpRestartAcquisition, 50, 0, kBoot ^ 0x0101);
    const Outcome out   = p.send(stale);
    EXPECT_TRUE(out.answered);
    EXPECT_EQ(out.effect, Effect::None);
    expectLast(p, 50, gatr2::kPicoOpRestartAcquisition, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailWrongTarget);

    // Target 0 never matches: boot ids are nonzero.
    p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 51, 0, 0));
    expectLast(p, 51, gatr2::kPicoOpRestartAcquisition, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailWrongTarget);

    // The same id retargeted to this boot runs.
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 50)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.restarts, 1);
}

TEST(PicoCommands, UnknownOpFails) {
    Pico p;
    EXPECT_EQ(p.send(commandFrame(9, 60)).effect, Effect::None);
    expectLast(p, 60, 9, gatr2::kPicoCommandFailed, gatr2::kPicoDetailUnknownOp);

    // Unknown ops may carry a body.
    const Bytes with_body =
        rawFrame({gatr2::kPicoLinkVersion, 200, 61, 0, static_cast<uint8_t>(kBoot),
                  static_cast<uint8_t>(kBoot >> 8), 7, 7});
    EXPECT_TRUE(p.send(with_body).answered);
    expectLast(p, 61, 200, gatr2::kPicoCommandFailed, gatr2::kPicoDetailUnknownOp);
}

TEST(PicoCommands, BodyLengthWrongForTheOpFails) {
    Pico p;
    // CONFIGURE without its body byte.
    const Bytes short_configure =
        rawFrame({gatr2::kPicoLinkVersion, gatr2::kPicoOpConfigure, 70, 0,
                  static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8)});
    EXPECT_EQ(p.send(short_configure).effect, Effect::None);
    expectLast(p, 70, gatr2::kPicoOpConfigure, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailBadBody);

    // RESTART_ACQUISITION with a stray byte never restarts.
    const Bytes long_restart =
        rawFrame({gatr2::kPicoLinkVersion, gatr2::kPicoOpRestartAcquisition, 71, 0,
                  static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8), 0});
    EXPECT_EQ(p.send(long_restart).effect, Effect::None);
    expectLast(p, 71, gatr2::kPicoOpRestartAcquisition, gatr2::kPicoCommandFailed,
               gatr2::kPicoDetailBadBody);
    EXPECT_EQ(p.restarts, 0);
}

TEST(PicoCommands, OtherVersionsZeroIdsAndCorruptFramesAreIgnored) {
    Pico  p;
    Bytes other_version = rawFrame({static_cast<uint8_t>(gatr2::kPicoLinkVersion + 1),
                                    gatr2::kPicoOpRestartAcquisition, 80, 0,
                                    static_cast<uint8_t>(kBoot), static_cast<uint8_t>(kBoot >> 8)});
    EXPECT_FALSE(p.send(other_version).answered);

    EXPECT_FALSE(p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 0)).answered);

    Bytes corrupt = commandFrame(gatr2::kPicoOpRestartAcquisition, 81);
    corrupt[5] ^= 0x40;
    EXPECT_FALSE(p.send(corrupt).answered);

    EXPECT_EQ(p.restarts, 0);
    EXPECT_EQ(p.commands.ignored(), 3u);
    expectLast(p, 0, 0, gatr2::kPicoCommandNone, gatr2::kPicoDetailNone);
}

TEST(PicoCommands, SameIdWithAnotherOpIsANewCommand) {
    Pico p;
    p.send(commandFrame(gatr2::kPicoOpConfigure, 90, 1));
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 90)).effect,
              Effect::RestartAcquisition);
    EXPECT_EQ(p.send(commandFrame(gatr2::kPicoOpRestartAcquisition, 90)).effect, Effect::None);
    EXPECT_EQ(p.restarts, 1);
}

TEST(PicoCommands, DuplicatesAreRememberedForTheLastFourCommands) {
    Pico        p;
    const Bytes oldest = commandFrame(gatr2::kPicoOpRestartAcquisition, 100);
    p.send(oldest);
    for (uint16_t id = 101; id <= 103; ++id) {
        p.send(commandFrame(gatr2::kPicoOpConfigure, id, 1));
    }
    EXPECT_EQ(p.send(oldest).effect, Effect::None);

    p.send(commandFrame(gatr2::kPicoOpConfigure, 104, 1));
    EXPECT_EQ(p.send(oldest).effect, Effect::RestartAcquisition);
    EXPECT_EQ(p.restarts, 2);
}

TEST(PicoCommands, StatusFrameCarriesTheLastCommand) {
    Pico p;
    p.send(commandFrame(gatr2::kPicoOpReinitImu, 0xBEEF, 0));

    gatr2::PicoStatus st = p.status();
    st.boot_id           = kBoot;
    st.imu_state         = gatr2::kPicoImuInitializing;
    uint8_t        buf[gatr2::kMaxFrameLen];
    const uint16_t n = gatr2::encodePicoStatus(st, buf, sizeof(buf));
    ASSERT_EQ(n, gatr2::kPicoStatusLen + gatr2::kLinkEnvelopeLen);

    gatr2::PicoStatus back;
    ASSERT_TRUE(gatr2::decodePicoStatus(buf, n, back));
    EXPECT_EQ(back.boot_id, kBoot);
    EXPECT_EQ(back.last_request_id, 0xBEEF);
    EXPECT_EQ(back.last_op, gatr2::kPicoOpReinitImu);
    EXPECT_EQ(back.last_status, gatr2::kPicoCommandRunning);
}

TEST(PicoCommands, FrameReaderDeliversCommandsFromAByteStream) {
    Pico               p;
    gatr2::FrameReader reader;
    Bytes              stream = {0x00, 0xAA, 0x13};
    const Bytes        a      = commandFrame(gatr2::kPicoOpConfigure, 110, 0);
    const Bytes        b      = commandFrame(gatr2::kPicoOpRestartAcquisition, 111);
    stream.insert(stream.end(), a.begin(), a.end());
    stream.insert(stream.end(), b.begin(), b.end());

    int frames = 0;
    for (uint8_t byte : stream) {
        if (!reader.push(byte)) {
            continue;
        }
        do {
            ASSERT_EQ(reader.frameType(), gatr2::kFramePicoCommand);
            p.send(Bytes(reader.frame(), reader.frame() + reader.frameLen()));
            ++frames;
        } while (reader.next());
    }
    EXPECT_EQ(frames, 2);
    EXPECT_FALSE(p.imu_enabled);
    EXPECT_EQ(p.restarts, 1);
}
