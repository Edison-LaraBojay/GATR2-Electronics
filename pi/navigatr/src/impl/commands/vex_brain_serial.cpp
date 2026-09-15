// vex_brain_serial.cpp

#include "impl/commands/vex_brain_serial.h"

#include "math/angles.h"
#include "resources/resource_store.h"

namespace navigatr
{

std::unique_ptr<Commands> VexBrainSerialCommands::create(const ConfigNode& node,
                                                         SlotInitializationContext& context,
                                                         std::string& err) {
    const ResourceId link_id{node.child("Serial").attr("resource_id")};
    if (link_id.empty()) {
        err = node.path() + ": needs <Serial resource_id=.../>";
        return nullptr;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    auto        link = context.resources->require<SerialLink>(link_id, inner);
    if (link == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    auto commands             = std::make_unique<VexBrainSerialCommands>();
    commands->link_           = std::move(link);
    commands->diagnostics_id_ = link_id.value;
    return commands;
}

void VexBrainSerialCommands::reset() {
    reader_.reset();
    have_seq_ = false;
}

CommandsOutput VexBrainSerialCommands::run(const CommandsInput& in) {
    CommandsOutput out;
    out.command = in.previous;

    LinkStats* stats =
        in.diagnostics != nullptr ? &in.diagnostics->links[diagnostics_id_] : nullptr;

    uint8_t buf[128];
    for (;;) {
        const SerialReadResult read =
            link_->readAvailable(MutableByteSpan{buf, sizeof(buf)});
        if (read.closed) {
            out.status = FunctionStatus::kNoData;   // dead link; state persists
            return out;
        }
        if (read.bytes == 0) {
            break;
        }
        if (stats != nullptr) {
            stats->bytes += static_cast<uint32_t>(read.bytes);
        }
        for (std::size_t i = 0; i < read.bytes; ++i) {
            if (!reader_.push(buf[i])) {
                continue;
            }
            if (reader_.frameType() != gatr2::kFrameCommand) {
                continue;
            }
            gatr2::CommandFrame c{};
            if (!gatr2::decodeCommandFrame(reader_.frame(), reader_.frameLen(), c)) {
                if (stats != nullptr) {
                    ++stats->decode_errors;
                }
                continue;
            }
            if (stats != nullptr) {
                ++stats->packets;
            }
            if (have_seq_ && c.seq == last_seq_) {
                continue;   // resend of the packet already applied
            }
            have_seq_ = true;
            last_seq_ = c.seq;

            switch (c.command) {
            case gatr2::kCmdInitPose:
                out.command.init_pose.x_m         = c.x_mm / 1000.0;
                out.command.init_pose.y_m         = c.y_mm / 1000.0;
                out.command.init_pose.heading_rad = cdegToRad(c.heading_cdeg);
                out.command.mode                  = c.mode;
                out.command.init_sequence += 1;
                break;
            case gatr2::kCmdSelectObject:
                out.command.object_requested =
                    (c.flags & gatr2::kCmdFlagObjectRequested) != 0;
                out.command.object_wire_id = c.object_id;
                out.command.object_sequence += 1;
                break;
            case gatr2::kCmdSetStream:
                out.command.stream_on = (c.flags & gatr2::kCmdFlagStreamOn) != 0;
                break;
            default:
                break;   // newer brain, unknown command, ignore
            }
        }
    }
    return out;
}

} // namespace navigatr
