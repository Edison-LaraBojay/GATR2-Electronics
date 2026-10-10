// cdc2.h
// The V5 CDC packets the controller source sends (SYSTEM_VERSION, USER_READ, RADIO_STATUS) and the
// replies it reads. Integers little endian, CRC-16/XMODEM big endian. Layouts from pros-cli
// v5_device.py and vex-cdc.
//   simple request    C9 36 B8 47 <cmd>
//   simple reply      AA 55 <cmd> <len u8> <payload>
//   extended request  C9 36 B8 47 56 <ecmd> <size VarU16> <payload> <crc16 BE>
//   extended reply    AA 55 56 <size VarU16> <ecmd> <ack> <payload> <crc16 BE>
// No request that writes to the Brain or switches the radio exists here.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

constexpr Byte kCommandSystemVersion = 0xA4;
constexpr Byte kCommandExtended      = 0x56; // USER_CDC
constexpr Byte kExtendedUserRead     = 0x27;
constexpr Byte kExtendedRadioStatus  = 0x26;
constexpr Byte kAck                  = 0x76;
constexpr Byte kUserChannel          = 1;    // stdio, channel 1
constexpr Byte kUserReadLength       = 0x40; // what pros-cli asks for
constexpr Byte kProductBrain         = 0x10;
constexpr Byte kProductController    = 0x11;
constexpr Byte kLinkedByCable        = 0x01;
constexpr Byte kLinkedByRadio        = 0x02;

constexpr std::size_t kMaxReplyBytes = 512; // longer sizes are garbage

uint16_t crc16Xmodem(const Byte* data, std::size_t size);

Bytes systemVersionRequest();
Bytes userReadRequest(Byte channel, Byte length);
Bytes radioStatusRequest();

struct CdcReply {
    Byte  command = 0;
    Bytes payload; // simple: len bytes; extended: ecmd, ack, data, no crc
};
using CdcReplies = std::vector<CdcReply>;

struct CdcScan {
    CdcReplies replies;
    uint32_t   bad_replies = 0; // extended with a bad crc or size
    uint32_t   stray_bytes = 0; // before a reply header
};

// Byte stream to replies; resyncs on AA 55.
class CdcReplyScanner {
  public:
    CdcScan push(const Byte* data, std::size_t size);
    void    reset();

  private:
    std::optional<CdcReply> next(CdcScan& scan);

    Bytes buffer_;
};

enum class LinkKind : uint8_t { kNone, kCable, kRadio };

struct SystemVersion {
    Byte product = 0;
    Byte flags   = 0;
};

std::optional<SystemVersion> parseSystemVersion(const CdcReply& reply);

// kNone unless a controller with a Brain link.
LinkKind controllerLink(const SystemVersion& version);

enum class UserReadError : uint8_t { kNone, kNotUserRead, kNack, kShort };

struct UserRead {
    UserReadError error = UserReadError::kNone;
    Byte          ack   = 0;
    Bytes         data; // Brain channel-1 bytes, may be empty
};

UserRead parseUserRead(const CdcReply& reply);

// Payload after ACK: device u8, quality u16, strength i16, channel u8, timeslot u8 (vexide).
std::optional<RadioStatus> parseRadioStatus(const CdcReply& reply);

const char* linkKindName(LinkKind kind);
const char* userReadErrorName(UserReadError error);

} // namespace spectagatr::receiver::sources
