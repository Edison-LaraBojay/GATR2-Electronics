// we_probe.cpp
// Numeric probe for the world estimation plan: is nominal + error the stored
// pose bit for bit, is the landmark error helper equal to the old target
// composition, and how big is the difference. Read-only against the worktree
// headers.
//
// g++ -std=c++17 -O0 -I<wt-navi>/pi/naviGATR/src we_probe.cpp -o we_probe.exe

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>

#include "math/angles.h"
#include "math/se3.h"
#include "math/transforms.h"
#include "resources/target_set.h"

using namespace navigatr;

namespace
{

struct LandmarkError {
    double dx = 0.0, dy = 0.0, dtheta = 0.0;
};

LandmarkError landmarkError(const Pose2D& nominal, const Pose2D& estimated) {
    return {estimated.x_m - nominal.x_m, estimated.y_m - nominal.y_m,
            wrapAngle(estimated.heading_rad - nominal.heading_rad)};
}

// form A: nominal landmark + translation + rotated offset
Pose2D applyA(const LandmarkError& e, const Pose2D& lm, const Pose2D& t) {
    const double ox = t.x_m - lm.x_m, oy = t.y_m - lm.y_m;
    const double c = std::cos(e.dtheta), s = std::sin(e.dtheta);
    return {lm.x_m + e.dx + c * ox - s * oy, lm.y_m + e.dy + s * ox + c * oy,
            wrapAngle(t.heading_rad + e.dtheta)};
}

// form B: target + translation + (rotation - identity) of the offset
Pose2D applyB(const LandmarkError& e, const Pose2D& lm, const Pose2D& t) {
    const double ox = t.x_m - lm.x_m, oy = t.y_m - lm.y_m;
    const double c = std::cos(e.dtheta), s = std::sin(e.dtheta);
    return {t.x_m + e.dx + (c - 1.0) * ox - s * oy, t.y_m + e.dy + s * ox + (c - 1.0) * oy,
            wrapAngle(t.heading_rad + e.dtheta)};
}

bool same(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0 || a == b; }
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
        std::printf("%-44s n=%ld bit_equal=%ld max_xy=%.3g max_h=%.3g raw_pi_flips=%ld\n", what,
                    n, bit, max_xy, max_h, flip);
    }
};

} // namespace

int main() {
    std::mt19937_64                        rng(20260928);
    std::uniform_real_distribution<double> pos(-5.0, 5.0), err(-0.5, 0.5), ang(-kPi, kPi),
        small(-1e-9, 1e-9), dth(-0.6, 0.6), tilt(-0.2, 0.2), off(-0.6, 0.6);

    TargetDecl vision;   // the chain of vision_targets_gtest
    vision.T_landmark_approach            = makeTransform3(-0.14, 0, 0, 0, 0, kPi);
    vision.T_robot_controlled             = makeTransform3(-0.40, 0, 0.05, 0, 0, kPi);
    vision.desired_controlled_in_approach = Pose2D{0.05, 0.0, kPi};

    Stats roundtrip, idA, idB, helperA, helperB, helperB_tilt, zero_err_old, nominal_rt;
    long  wrap_near_pi = 0;
    for (int i = 0; i < 1000000; ++i) {
        // near +-pi on a quarter of the cases
        const bool   edge = (i % 4) == 0;
        const double nh   = edge ? (i % 8 == 0 ? kPi : -kPi) + small(rng) : ang(rng);
        const Pose2D N{pos(rng), pos(rng), wrapAngle(nh)};
        const double ph = edge ? kPi - std::fabs(small(rng)) * (i % 3 == 0 ? -1 : 1) : ang(rng);
        const Pose2D P{N.x_m + err(rng), N.y_m + err(rng), wrapAngle(edge ? ph : N.heading_rad + dth(rng))};
        wrap_near_pi += edge ? 1 : 0;
        const Pose2D F{pos(rng), pos(rng), edge ? wrapAngle(kPi + small(rng)) : ang(rng)};

        // error round trip: nominal + error vs the stored pose
        const LandmarkError e = landmarkError(N, P);
        roundtrip.add(applyA(e, N, N), P);
        nominal_rt.add(applyB(e, N, N), P);

        // identity
        const Pose2D T{pos(rng), pos(rng), ang(rng)};
        idA.add(applyA(LandmarkError{}, N, T), T);
        idB.add(applyB(LandmarkError{}, N, T), T);

        // helper vs old target composition (policy none)
        const Pose2D old = vision.resolveFromLandmark(compose(inverse(F), P));
        const Pose2D T_nom = vision.resolveFromLandmark(N);
        helperA.add(compose(inverse(F), applyA(e, N, T_nom)), old);
        helperB.add(compose(inverse(F), applyB(e, N, T_nom)), old);

        // zero error through the helper vs old with P == N
        const Pose2D old_nom = vision.resolveFromLandmark(compose(inverse(F), N));
        zero_err_old.add(compose(inverse(F), applyB(LandmarkError{}, N, T_nom)), old_nom);

        // tilted SE(3) chain
        TargetDecl tilted;
        tilted.T_landmark_approach =
            makeTransform3(off(rng), off(rng), 0.1, tilt(rng), tilt(rng), ang(rng));
        tilted.T_robot_controlled =
            makeTransform3(off(rng), off(rng), 0.05, tilt(rng), tilt(rng), ang(rng));
        tilted.desired_controlled_in_approach = Pose2D{off(rng), off(rng), ang(rng)};
        helperB_tilt.add(
            compose(inverse(F), applyB(e, N, tilted.resolveFromLandmark(N))),
            tilted.resolveFromLandmark(compose(inverse(F), P)));
    }
    std::printf("cases near +-pi: %ld\n", wrap_near_pi);
    roundtrip.print("nominal+error (form A, target=landmark) vs P");
    nominal_rt.print("nominal+error (form B, target=landmark) vs P");
    idA.print("identity form A vs T");
    idB.print("identity form B vs T");
    helperA.print("helper form A vs old composition");
    helperB.print("helper form B vs old composition");
    helperB_tilt.print("helper form B vs old, tilted SE(3) chain");
    zero_err_old.print("zero error helper vs old nominal (P == N)");

    // the textbook counterexample for a + (b - a) == b
    const double a = 0.1, b = 1e-17;
    std::printf("0.1 + (1e-17 - 0.1) == 1e-17: %s (got %.17g)\n", (a + (b - a)) == b ? "yes" : "no",
                a + (b - a));
    return 0;
}
