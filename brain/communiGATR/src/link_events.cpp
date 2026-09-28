// link_events.cpp

#include "communigatr/link_events.h"

namespace communigatr
{

LinkSnapshot snapshotOf(const Client& client, Seconds now) {
    LinkSnapshot s;
    s.connected   = client.connected(now);
    s.session     = client.session();
    s.pi_instance = client.piInstance();
    s.profile     = client.profile().state;
    const StateSample& sample = client.state();
    s.state_valid             = sample.valid;
    if (sample.valid) {
        s.localized       = (sample.state.robot_flags & gatr2::kRobotLocalized) != 0;
        s.health          = sample.state.health;
        s.calibration     = sample.state.calibration;
        s.odometry_epoch  = sample.state.odometry_epoch;
        s.anchor_revision = sample.state.anchor_revision;
    }
    return s;
}

const char* toString(LinkEvent event) {
    switch (event) {
    case LinkEvent::kLinkLost: return "link lost";
    case LinkEvent::kLinkRestored: return "link restored";
    case LinkEvent::kNewSession: return "new session";
    case LinkEvent::kPiRestarted: return "Pi restarted";
    case LinkEvent::kProfileApplied: return "profile applied";
    case LinkEvent::kProfileRejected: return "profile rejected";
    case LinkEvent::kPlaced: return "placed";
    case LinkEvent::kPlacementLost: return "placement lost";
    case LinkEvent::kOdometryReset: return "odometry reset";
    case LinkEvent::kPicoLost: return "Pico link lost";
    case LinkEvent::kPicoRestored: return "Pico link restored";
    case LinkEvent::kImuFailed: return "Pico IMU failed";
    case LinkEvent::kImuReady: return "Pico IMU ready";
    case LinkEvent::kCalibrating: return "calibrating";
    case LinkEvent::kCalibrated: return "calibrated";
    case LinkEvent::kCalibrationFailed: return "calibration failed";
    }
    return "?";
}

const LinkEventRecord& LinkEvents::at(std::size_t i) const {
    return ring_[(next_ + kCapacity - 1 - i) % kCapacity];
}

void LinkEvents::add(LinkEvent event, Seconds now, Seconds duration) {
    ring_[next_] = LinkEventRecord{now, event, duration};
    next_        = (next_ + 1) % kCapacity;
    if (count_ < kCapacity) {
        ++count_;
    }
    ++total_;
}

namespace
{

bool calibrating(uint8_t c) {
    return c == gatr2::kCalibrationRunning || c == gatr2::kCalibrationWaitingStill ||
           c == gatr2::kCalibrationWaitingData;
}

} // namespace

void LinkEvents::update(const LinkSnapshot& s, Seconds now) {
    if (connected_ && !s.connected) {
        lost_    = true;
        lost_at_ = now;
        add(LinkEvent::kLinkLost, now);
    } else if (!connected_ && s.connected && lost_) {
        lost_ = false;
        add(LinkEvent::kLinkRestored, now, now - lost_at_);
    }
    connected_ = s.connected;

    bool restarted = false;
    if (s.pi_instance != 0) {
        restarted    = pi_instance_ != 0 && s.pi_instance != pi_instance_;
        pi_instance_ = s.pi_instance;
    }
    if (restarted) {
        add(LinkEvent::kPiRestarted, now);
    }
    if (s.session != 0) {
        if (!restarted && session_ != 0 && s.session != session_) {
            add(LinkEvent::kNewSession, now);
        }
        session_ = s.session;
    }

    if (s.profile != profile_) {
        if (s.profile == ProfileSync::kApplied) {
            add(LinkEvent::kProfileApplied, now);
        } else if (s.profile == ProfileSync::kRejected || s.profile == ProfileSync::kInvalid) {
            add(LinkEvent::kProfileRejected, now);
        }
        profile_ = s.profile;
    }

    if (!s.state_valid) {
        return;
    }
    if (have_state_) {
        compareStates(state_, s, now);
    } else if (calibrating(s.calibration)) {
        add(LinkEvent::kCalibrating, now);
    }
    have_state_ = true;
    state_      = s;
}

// A Pi restart implies an odometry reset; only epochs of one Pi compare.
void LinkEvents::compareStates(const LinkSnapshot& p, const LinkSnapshot& s, Seconds now) {
    if (!p.localized && s.localized) {
        add(LinkEvent::kPlaced, now);
    } else if (p.localized && !s.localized) {
        add(LinkEvent::kPlacementLost, now);
    }
    if (s.pi_instance == p.pi_instance && s.odometry_epoch != p.odometry_epoch) {
        add(LinkEvent::kOdometryReset, now);
    }

    const bool pico_was = (p.health & gatr2::kHealthPicoLink) != 0;
    const bool pico_is  = (s.health & gatr2::kHealthPicoLink) != 0;
    if (pico_was && !pico_is) {
        add(LinkEvent::kPicoLost, now);
    } else if (!pico_was && pico_is) {
        add(LinkEvent::kPicoRestored, now);
    }
    const bool failed_was = (p.health & gatr2::kHealthImuFailed) != 0;
    const bool failed_is  = (s.health & gatr2::kHealthImuFailed) != 0;
    if (!failed_was && failed_is) {
        add(LinkEvent::kImuFailed, now);
    } else if (failed_was && !failed_is) {
        add(LinkEvent::kImuReady, now);
    }

    if (s.calibration != p.calibration) {
        if (calibrating(s.calibration) && !calibrating(p.calibration)) {
            add(LinkEvent::kCalibrating, now);
        } else if (s.calibration == gatr2::kCalibrationDone) {
            add(LinkEvent::kCalibrated, now);
        } else if (s.calibration == gatr2::kCalibrationFailed) {
            add(LinkEvent::kCalibrationFailed, now);
        }
    }
}

} // namespace communigatr
