// RS-485 link receive test (brain side).
//
// Reads whatever pi_transmit.py sends and shows it on the brain screen.
// Copy this in as your PROS project's src/main.cpp, set kSerialPort to the
// smart port the RS-485 pair is plugged into, build, and run.
//
// The A/B pair from the transceiver goes into a V5 smart port. This reads it
// with generic serial at the same baud the Pi transmits.

#include "main.h"
#include "pros/llemu.hpp"
#include "pros/serial.hpp"

#include <string>

namespace {
constexpr int kSerialPort = 1;  // smart port the RS-485 pair plugs into
constexpr int kBaud       = 115200;
} // namespace

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "RS485 link test: waiting");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    pros::Serial serial(kSerialPort, kBaud);
    std::string  line;
    int          count = 0;

    while (true) {
        while (serial.get_read_avail() > 0) {
            std::int32_t b = serial.read_byte();
            if (b < 0) {
                break;
            }
            if (b == '\n') {
                ++count;
                pros::lcd::print(0, "RS485 link up");
                pros::lcd::print(1, "messages: %d", count);
                pros::lcd::print(2, "last: %s", line.c_str());
                line.clear();
            } else if (b != '\r') {
                line.push_back(static_cast<char>(b));
            }
        }
        pros::delay(10);
    }
}
