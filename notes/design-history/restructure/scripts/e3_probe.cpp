// e3_probe.cpp
// Step 3 numeric probe against the shipped helpers in
// pipeline/world_estimation/field_state.h (not local copies).
// Old and new policy-none bodies are the ae457ee and step-3 texts of
// ConfiguredTargetResolution::estimateTargetPose, minus the member lookups.
//
// g++ -std=c++17 -O0 -I<wt-navi>/pi/naviGATR/src e3_probe.cpp -o e3_probe.exe

#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <random>

#include "math/angles.h"
#include "math/se3.h"
#include "math/transforms.h"
#include "pipeline/resources/target_set.h"
#include "pipeline/world_estimation/field_state.h"

using namespace navigatr;

namespace
{

bool same(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
bool samePose(const Pose2D& a, const Pose2D& b) {
    return same(a.x_m, b.x_m) && same(a.y_m, b.y_m) && same(a.heading_rad, b.heading_rad);
}

struct Stats {
    long   n = 0, bit = 0, flip = 0;
    double max_xy = 0.0, max_h = 0.0;
    void   add(const Pose2D& a, const Pose2D& b) {
        ++n;
        bit += samePose(a, b) ? 1 : 0;
        max_xy = std::fmax(max_xy, std::fmax(std::fabs(a.x_m - b.x_m), std::fabs(a.y_m - b.y_m)));
        max_h  = std::fmax(max_h, std::fabs(wrapAngle(a.heading_rad - b.heading_rad)));
        flip += std::fabs(a.heading_rad - b.heading_rad) > kPi ? 1 : 0;
    }
    void print(const char* what) const {
        std::printf("%-46s n=%ld bit_equal=%ld max_xy=%.3g max_h=%.3g raw_pi_flips=%ld\n", what, n,
                    bit, max_xy, max_h, flip);
    }
};

struct Scalar {
    long n = 0, bit = 0;
    void add(double a, double b) {
        ++n;
        bit += same(a, b) ? 1 : 0;
    }
    void print(const char* what) const {
        std::printf("%-46s n=%ld bit_equal=%ld\n", what, n, bit);
    }
};

// ae457ee body
Pose2D oldPolicyNone(const TargetDecl& decl, const Pose2D* map_nominal, const Pose2D* state_pose,
                     const Pose2D& F) {
    Pose2D T_field_landmark;
    if (map_nominal) {
        T_field_landmark = *map_nominal;
    }
    if (state_pose) {
        T_field_landmark = *state_pose;
    }
    const auto T_odom_landmark = compose(inverse(F), T_field_landmark);
    return decl.resolveFromLandmark(T_odom_landmark);
}

// step-3 body
Pose2D newPolicyNone(const TargetDecl& decl, const Pose2D* map_nominal,
                     const FieldObjectState* state, const Pose2D& F) {
    Pose2D        landmark_nominal;
    LandmarkError error;
    if (map_nominal) {
        landmark_nominal = *map_nominal;
    }
    if (state) {
        landmark_nominal = state->nominal.value_or(state->pose.pose);
        error            = state->error;
    }
    const Pose2D T_field_target =
        applyLandmarkError(error, landmark_nominal, decl.resolveFromLandmark(landmark_nominal));
    return compose(inverse(F), T_field_target);
}

} // namespace

int main() {
    std::mt19937_64                        rng(20260928);
    std::uniform_real_distribution<double> pos(-5.0, 5.0), err(-0.5, 0.5), ang(-kPi, kPi),
        small(-1e-9, 1e-9), dth(-0.6, 0.6), tilt(-0.2, 0.2), off(-0.6, 0.6);

    TargetDecl vision; // the chain of vision_targets_gtest
    vision.kind                           = TargetKind::kLandmarkRelative;
    vision.T_landmark_approach            = makeTransform3(-0.14, 0, 0, 0, 0, kPi);
    vision.T_robot_controlled             = makeTransform3(-0.40, 0, 0.05, 0, 0, kPi);
    vision.desired_controlled_in_approach = Pose2D{0.05, 0.0, kPi};

    Stats  roundtrip, identity, mapped_vision, mapped_tilt, unmapped, unobserved, seeded;
    Scalar zero_dx, zero_dy, zero_dth, insp_h, insp_d;
    long   near_pi = 0;
    for (int i = 0; i < 1000000; ++i) {
        const bool   edge = (i % 4) == 0;
        const double nh   = edge ? (i % 8 == 0 ? kPi : -kPi) + small(rng) : ang(rng);
        const Pose2D N{pos(rng), pos(rng), wrapAngle(nh)};
        const double ph = edge ? kPi - std::fabs(small(rng)) * (i % 3 == 0 ? -1 : 1) : ang(rng);
        const Pose2D P{N.x_m + err(rng), N.y_m + err(rng),
                       wrapAngle(edge ? ph : N.heading_rad + dth(rng))};
        near_pi += edge ? 1 : 0;
        const Pose2D F{pos(rng), pos(rng), edge ? wrapAngle(kPi + small(rng)) : ang(rng)};
        const Pose2D T{pos(rng), pos(rng), ang(rng)};

        TargetDecl tilted;
        tilted.kind = TargetKind::kLandmarkRelative;
        tilted.T_landmark_approach =
            makeTransform3(off(rng), off(rng), 0.1, tilt(rng), tilt(rng), ang(rng));
        tilted.T_robot_controlled =
            makeTransform3(off(rng), off(rng), 0.05, tilt(rng), tilt(rng), ang(rng));
        tilted.desired_controlled_in_approach = Pose2D{off(rng), off(rng), ang(rng)};

        // state as landmark_estimator writes it: nominal, then pose with error
        FieldObjectState mapped;
        mapped.nominal   = N;
        mapped.pose.pose = P;
        mapped.error     = landmarkError(N, P);
        FieldObjectState unmapped_state;
        unmapped_state.pose.pose = P;
        FieldObjectState seeded_state;
        seeded_state.nominal   = N;
        seeded_state.pose.pose = N;
        seeded_state.error     = landmarkError(N, N);

        roundtrip.add(applyLandmarkError(mapped.error, N, N), P);
        identity.add(applyLandmarkError(LandmarkError{}, N, T), T);
        zero_dx.add(seeded_state.error.dx, 0.0);
        zero_dy.add(seeded_state.error.dy, 0.0);
        zero_dth.add(seeded_state.error.dtheta, 0.0);

        // policy none, the three branches of estimateTargetPose
        mapped_vision.add(newPolicyNone(vision, &N, &mapped, F),
                          oldPolicyNone(vision, &N, &P, F));
        mapped_tilt.add(newPolicyNone(tilted, &N, &mapped, F), oldPolicyNone(tilted, &N, &P, F));
        unmapped.add(newPolicyNone(vision, nullptr, &unmapped_state, F),
                     oldPolicyNone(vision, nullptr, &P, F));
        unobserved.add(newPolicyNone(vision, &N, nullptr, F), oldPolicyNone(vision, &N, nullptr, F));
        seeded.add(newPolicyNone(vision, &N, &seeded_state, F), oldPolicyNone(vision, &N, &N, F));

        // inspection fields: old formula vs read from the error
        insp_h.add(radToDeg(mapped.error.dtheta), radToDeg(wrapAngle(P.heading_rad - N.heading_rad)));
        insp_d.add(std::hypot(mapped.error.dx, mapped.error.dy),
                   std::hypot(P.x_m - N.x_m, P.y_m - N.y_m));
    }
    std::printf("cases: 1000000, near +-pi: %ld\n", near_pi);
    identity.print("identity: apply(0, N, T) vs T");
    zero_dx.print("seeded error dx == +0.0");
    zero_dy.print("seeded error dy == +0.0");
    zero_dth.print("seeded error dtheta == +0.0");
    roundtrip.print("round trip: apply(e, N, N) vs P");
    insp_h.print("inspection heading_error_deg old vs new");
    insp_d.print("inspection displacement_m old vs new");
    mapped_vision.print("policy none, mapped, vision chain");
    mapped_tilt.print("policy none, mapped, tilted SE(3) chain");
    unmapped.print("policy none, valid unmapped (N := P, e = 0)");
    seeded.print("policy none, seeded (P == N, e = 0)");
    unobserved.print("policy none, no valid state (map nominal)");
    return 0;
}
