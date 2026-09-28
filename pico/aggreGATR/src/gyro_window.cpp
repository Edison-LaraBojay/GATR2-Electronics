// gyro_window.cpp

#include "gyro_window.h"

#include <cmath>

namespace bno08x
{

namespace
{

constexpr double kMdpsPerLsb = 180000.0 / (512.0 * 3.14159265358979323846);

} // namespace

int32_t q9MeanToMdps(double sum_q9, uint32_t n) {
    return static_cast<int32_t>(std::lround(sum_q9 * kMdpsPerLsb / n));
}

void GyroWindow::add(uint32_t t_us, float rate_q9) {
    if (!std::isfinite(rate_q9) || std::fabs(rate_q9) > 65536.0f) {
        return;
    }
    if (has_cut_ && atOrBefore(t_us, cut_us_)) {
        ++late_;
        return;
    }
    if (n_ == kCapacity) {
        ++overflowed_;
        return;
    }
    buf_[n_++] = {t_us, rate_q9};
}

bool GyroWindow::take(uint32_t cut_us, int32_t& mdps) {
    double  sum  = 0;
    uint8_t used = 0;
    uint8_t kept = 0;
    for (uint8_t i = 0; i < n_; ++i) {
        if (atOrBefore(buf_[i].t_us, cut_us)) {
            sum += buf_[i].rate_q9;
            ++used;
        } else {
            buf_[kept++] = buf_[i];
        }
    }
    n_       = kept;
    cut_us_  = cut_us;
    has_cut_ = true;

    if (used == 0) {
        return false;
    }
    mdps = q9MeanToMdps(sum, used);
    return true;
}

void GyroWindow::clear() { n_ = 0; }

} // namespace bno08x
