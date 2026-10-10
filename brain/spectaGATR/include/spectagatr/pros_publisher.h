// pros_publisher.h
// spectaGATR publisher for a PROS program. One background task writes each
// pose line to stdout with one write() call, so each line is one PROS COBS
// packet (raw bytes when another library has turned COBS off). It never
// reads stdin, never calls serctl or fdctl, never opens a /ser stream.
// Source in pros/, PROS builds only.
//
//   spectagatr::ProsPublisher viz;          // global
//   void initialize() { viz.start(); }
//   viz.updatePose(x, y, heading);          // any task, never blocks
//   viz.updatePose(x, y, spectagatr::Attitude{roll, pitch, yaw});
//
// Only the background task ever waits on stdout: PROS blocks a write while its
// 2047 B output buffer is full, and then every other stdout writer too.

#pragma once
#include <atomic>

#include "spectagatr/publisher.h"

namespace spectagatr
{

class ProsPublisher {
  public:
    explicit ProsPublisher(Hertz rate = kDefaultRateHz);

    ProsPublisher(const ProsPublisher&)            = delete;
    ProsPublisher& operator=(const ProsPublisher&) = delete;

    // From initialize() or later, not from a global constructor. The first call
    // picks the session id and creates the task (priority TASK_PRIORITY_DEFAULT
    // - 1); later calls resume output. False when the task could not be created.
    bool start();
    // Stops output; the task stays parked until the next start().
    void stop();
    bool running() const;

    // Stamped with pros::millis(). Any task, never blocks.
    void updatePose(Meters x, Meters y, Radians heading);
    void updatePose(Meters x, Meters y, const Attitude& attitude);
    void resetTrail();

    void           setRate(Hertz rate); // 1..50 Hz, default 10
    Hertz          rate() const;
    PublisherStats stats() const;
    SessionId      session() const; // 0 before the first start()

  private:
    static void run(void* self);

    Publisher         publisher_;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> task_created_{false};
};

} // namespace spectagatr
