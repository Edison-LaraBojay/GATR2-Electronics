// diag_report.cpp

#include "diag_report.h"

namespace pilink
{

translagatr::PicoDiag makeDiag(const DiagInputs& in) {
    translagatr::PicoDiag d;
    d.boot_id          = in.boot_id;
    d.seq              = in.seq;
    d.firmware         = in.firmware;
    d.pins             = in.pins;
    d.pins_known       = in.pins_known;
    d.imu_rx           = static_cast<uint16_t>(in.imu.rx);
    d.imu_bad          = static_cast<uint16_t>(in.imu.bad);
    d.imu_resets       = static_cast<uint8_t>(in.imu.resets);
    d.imu_error        = static_cast<int8_t>(in.imu.error < -128  ? -128
                                             : in.imu.error > 127 ? 127
                                                                  : in.imu.error);
    d.reports_ok       = static_cast<uint16_t>(in.imu.reports_ok);
    d.reports_rejected = static_cast<uint16_t>(in.imu.reports_rejected);
    d.report_age_ms    = !in.imu.have_report          ? 0xFFFFu
                         : in.imu.report_age_ms > 0xFFFEu ? 0xFFFEu
                                                          : static_cast<uint16_t>(in.imu.report_age_ms);
    d.link_rx_bad      = static_cast<uint16_t>(in.link_rx_bad);
    d.ticks_skipped    = static_cast<uint16_t>(in.ticks_skipped);
    d.flags            = in.imu_present ? translagatr::kPicoDiagImuPresent : 0;
    return d;
}

} // namespace pilink
