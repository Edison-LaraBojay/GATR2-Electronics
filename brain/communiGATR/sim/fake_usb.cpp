// fake_usb.cpp

#include "sim/fake_usb.h"

#include <algorithm>

namespace communigatr
{

int FakeUsb::Port::read(uint8_t* buf, int max) { return usb_.brainRead(buf, max); }

bool FakeUsb::Port::write(const uint8_t* data, int len) { return usb_.brainWrite(data, len); }

FakeUsb::FakeUsb(FakePi& pi, const FakeUsbConfig& config)
    : pi_(pi), config_(config), port_(*this) {}

void FakeUsb::advanceTo(Seconds now) {
    now_ = std::max(now_, now);
    while (!to_pi_.empty() && to_pi_.begin()->first <= now_) {
        const Seconds     at    = to_pi_.begin()->first;
        const std::string chars = to_pi_.begin()->second;
        to_pi_.erase(to_pi_.begin());
        deliverToPi(chars, at);
    }
    while (!to_brain_.empty() && to_brain_.begin()->first <= now_) {
        const std::string chars = to_brain_.begin()->second;
        to_brain_.erase(to_brain_.begin());
        for (char c : chars) {
            if (brain_decoder_.push(c)) {
                brain_rx_.insert(brain_rx_.end(), brain_decoder_.bytes(),
                                 brain_decoder_.bytes() + brain_decoder_.length());
            }
        }
    }
}

void FakeUsb::setPlugged(bool plugged) {
    if (plugged == plugged_) {
        return;
    }
    plugged_ = plugged;
    to_pi_.clear();
    to_brain_.clear();
    if (plugged) {
        // pros_usb_link reopens and flushes; its first drain has no reply.
        reopened_ = true;
        pi_decoder_.reset();
    }
}

void FakeUsb::textToPi(const std::string& text) {
    if (plugged_) {
        to_pi_.emplace(now_ + config_.to_pi, text);
    }
}

void FakeUsb::textToBrain(const std::string& text) {
    if (plugged_) {
        to_brain_.emplace(now_ + config_.to_brain, text);
    }
}

int FakeUsb::brainRead(uint8_t* buf, int max) {
    const int n = std::min<int>(max, static_cast<int>(brain_rx_.size()));
    std::copy(brain_rx_.begin(), brain_rx_.begin() + n, buf);
    brain_rx_.erase(brain_rx_.begin(), brain_rx_.begin() + n);
    return n;
}

bool FakeUsb::brainWrite(const uint8_t* data, int len) {
    char              line[kUsbLineMax];
    const std::size_t n = encodeUsbLine(data, static_cast<std::size_t>(len), line, sizeof(line));
    if (n == 0) {
        return false;
    }
    if (plugged_) {
        to_pi_.emplace(now_ + config_.to_pi, std::string(line, n));
    }
    return true;
}

void FakeUsb::deliverToPi(const std::string& chars, Seconds at) {
    for (char c : chars) {
        if (!pi_decoder_.push(c)) {
            continue;
        }
        ++lines_to_pi_;
        const auto replies = pi_.receive(pi_decoder_.bytes(), pi_decoder_.length());
        for (const std::vector<uint8_t>& reply : replies) {
            if (reopened_) {
                reopened_ = false;
                ++unanswered_;
                continue;
            }
            char              line[kUsbLineMax];
            const std::size_t n = encodeUsbLine(reply.data(), reply.size(), line, sizeof(line));
            to_brain_.emplace(at + config_.turnaround + config_.to_brain, std::string(line, n));
        }
    }
}

} // namespace communigatr
