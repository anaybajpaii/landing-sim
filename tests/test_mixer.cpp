#include <array>
#include <cmath>
#include <cstddef>

#include "check.hpp"
#include "quad/mixer.hpp"
#include "quad/quad_params.hpp"

using core::Vec3d;

namespace {

MixerWrench make(double thrust, double tx, double ty, double tz) {
    MixerWrench w;
    w.thrust_n = thrust;
    w.torque_nm = Vec3d{tx, ty, tz};
    return w;
}

void test_hover_split_equally() {
    const QuadParams p;
    const Mixer mixer(p);
    const double weight = p.mass * p.gravity;
    const double wHover = std::sqrt(weight / (4.0 * p.kThrust));
    const MixerWrench w = make(weight, 0.0, 0.0, 0.0);
    const std::array<double, 4> t = mixer.thrusts(w);
    const std::array<double, 4> om = mixer.omegaCommands(w);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK_NEAR(t[i], weight / 4.0, 1e-12);
        CHECK_NEAR(om[i], wHover, 1e-9);
    }
}

void test_round_trip() {
    const QuadParams p;
    const Mixer mixer(p);
    const MixerWrench w = make(12.0, 0.05, -0.03, 0.01);
    const MixerWrench back = mixer.wrench(mixer.thrusts(w));
    CHECK_NEAR(back.thrust_n, w.thrust_n, 1e-12);
    CHECK_NEAR(back.torque_nm.x, w.torque_nm.x, 1e-12);
    CHECK_NEAR(back.torque_nm.y, w.torque_nm.y, 1e-12);
    CHECK_NEAR(back.torque_nm.z, w.torque_nm.z, 1e-12);
}

void test_roll_speeds_up_left_motors() {
    const QuadParams p;
    const Mixer mixer(p);
    const std::array<double, 4> t = mixer.thrusts(make(p.mass * p.gravity, 0.1, 0.0, 0.0));
    CHECK(t[1] > t[0]);
    CHECK(t[2] > t[3]);
    CHECK_NEAR(t[1], t[2], 1e-12);
    CHECK_NEAR(t[0], t[3], 1e-12);
}

void test_pitch_speeds_up_front_motors() {
    const QuadParams p;
    const Mixer mixer(p);
    const std::array<double, 4> t = mixer.thrusts(make(p.mass * p.gravity, 0.0, 0.1, 0.0));
    CHECK(t[0] > t[2]);
    CHECK(t[1] > t[3]);
    CHECK_NEAR(t[0], t[1], 1e-12);
    CHECK_NEAR(t[2], t[3], 1e-12);
}

void test_yaw_speeds_up_ccw_motors() {
    const QuadParams p;
    const Mixer mixer(p);
    const double weight = p.mass * p.gravity;
    const double yawRatio = p.kTorque / p.kThrust;
    const std::array<double, 4> t = mixer.thrusts(make(weight, 0.0, 0.0, yawRatio));
    CHECK_NEAR(t[0], t[2], 1e-12);
    CHECK_NEAR(t[1], t[3], 1e-12);
    CHECK_NEAR(t[0], 0.25 * (weight + 1.0), 1e-12);
    CHECK_NEAR(t[1], 0.25 * (weight - 1.0), 1e-12);
}

void test_saturation_clamps() {
    const QuadParams p;
    const Mixer mixer(p);
    const double maxT = p.kThrust * p.omegaMax * p.omegaMax;
    const MixerWrench hi = make(100.0, 0.0, 0.0, 0.0);
    const MixerWrench lo = make(-5.0, 0.0, 0.0, 0.0);
    const std::array<double, 4> th = mixer.thrusts(hi);
    const std::array<double, 4> oh = mixer.omegaCommands(hi);
    const std::array<double, 4> tl = mixer.thrusts(lo);
    const std::array<double, 4> ol = mixer.omegaCommands(lo);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK_NEAR(th[i], maxT, 1e-12);
        CHECK_NEAR(oh[i], p.omegaMax, 1e-9);
        CHECK(tl[i] == 0.0);
        CHECK(ol[i] == 0.0);
    }
}

}

int main() {
    test_hover_split_equally();
    test_round_trip();
    test_roll_speeds_up_left_motors();
    test_pitch_speeds_up_front_motors();
    test_yaw_speeds_up_ccw_motors();
    test_saturation_clamps();
    return check::report();
}