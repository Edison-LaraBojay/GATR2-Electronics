// vex_brain_serial.h
// Decodes VEX brain command packets from a serial resource and folds them
// into the command state. A resent packet with the same wire seq is applied
// once; a new init command bumps init_sequence so localization applies it
// edge triggered. All brain wire knowledge (packet layout, fixed point mm
// and centidegrees) lives here and in common/.
//
//   <CommandCollection type="commands/vex_brain_serial">
//       <Serial resource_id="brain_uart"/>
//   </CommandCollection>

#pragma once
#include <memory>
#include <string>

#include "common/frame_codec.h"
#include "contracts/commands.h"
#include "resources/serial_link.h"

namespace navigatr
{

class VexBrainSerialCommands : public Commands
{
public:
    static std::unique_ptr<Commands> create(const ConfigNode& node,
                                            SlotInitializationContext& context,
                                            std::string& err);

    CommandsOutput run(const CommandsInput& in) override;

    void reset() override;

private:
    std::shared_ptr<SerialLink> link_;
    std::string                 diagnostics_id_;

    gatr2::FrameReader reader_;
    bool               have_seq_ = false;
    uint8_t            last_seq_ = 0;
};

} // namespace navigatr
