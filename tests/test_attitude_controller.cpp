#include <algorithm>
#include <cmath>

#include "check.hpp"
#include "control/attitude_controller.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"

using core::AttitudeConfig;
using core::AttitudeController;
using core::Quatd;
using core::Vec3d;
using sim::Quad;
using sim::State;

namespace {

struct Result {
    Vec3d final_euler;
    Vec3d max_euler;
};

double hover_omega(const QuadParams& p) {
    return std::sqrt(p.mass * p.gravity / (4.0 * p.kThrust));
}

Result fly(const State& initial, const Vec3d& setpoint, double seconds) {
    const QuadParams p;
    Quad q(p);
    q.reset(initial, hover_omega(p));
    AttitudeController ctl;
    const double dt = 1e-3;
    const int steps = static_cast<int>(seconds / dt);
    Vec3d max_e{};
    Vec3d e{};
    for (int i = 0; i < steps; ++i) {
        const State& s = q.state();
        const Vec3d torque = ctl.update(s.attitude, s.omega, setpoint, dt);
        q.step(p.mass * p.gravity, torque, dt);
        e = core::euler_zyx(q.state().attitude);
        max_e.x = std::max(max_e.x, e.x);
        max_e.y = std::max(max_e.y, e.y);
        max_e.z = std::max(max_e.z, e.z);
    }
    return Result{e, max_e};
}

void test_zero_output_at_setpoint() {
    AttitudeController ctl;
    const Vec3d t = ctl.update(Quatd{}, Vec3d{}, Vec3d{}, 1e-3);
    CHECK_NEAR(t.x, 0.0, 1e-12);
    CHECK_NEAR(t.y, 0.0, 1e-12);
    CHECK_NEAR(t.z, 0.0, 1e-12);
}

void test_torque_saturates() {
    const AttitudeConfig cfg;
    AttitudeController ctl(cfg);
    const Vec3d t = ctl.update(Quatd{}, Vec3d{}, Vec3d{2.0, 0.0, 0.0}, 1e-3);
    CHECK_NEAR(t.x, cfg.max_torque_roll_pitch, 1e-12);
}

void test_yaw_error_wraps_the_short_way() {
    AttitudeController ctl;
    const Quatd q = core::from_euler_zyx(0.0, 0.0, -3.0);
    const Vec3d t = ctl.update(q, Vec3d{}, Vec3d{0.0, 0.0, 3.0}, 1e-3);
    CHECK(t.z < 0.0);
    CHECK_NEAR(t.x, 0.0, 1e-12);
    CHECK_NEAR(t.y, 0.0, 1e-12);
}

void test_roll_step() {
    const Result r = fly(State{}, Vec3d{0.2, 0.0, 0.0}, 3.0);
    CHECK_NEAR(r.final_euler.x, 0.2, 1e-3);
    CHECK(r.max_euler.x < 0.22);
    CHECK_NEAR(r.final_euler.y, 0.0, 1e-6);
    CHECK_NEAR(r.final_euler.z, 0.0, 1e-6);
}

void test_pitch_step() {
    const Result r = fly(State{}, Vec3d{0.0, 0.2, 0.0}, 3.0);
    CHECK_NEAR(r.final_euler.y, 0.2, 1e-3);
    CHECK(r.max_euler.y < 0.22);
    CHECK_NEAR(r.final_euler.x, 0.0, 1e-6);
    CHECK_NEAR(r.final_euler.z, 0.0, 1e-6);
}

void test_yaw_step() {
    const Result r = fly(State{}, Vec3d{0.0, 0.0, 0.5}, 5.0);
    CHECK_NEAR(r.final_euler.z, 0.5, 1e-3);
    CHECK(r.max_euler.z < 0.55);
    CHECK_NEAR(r.final_euler.x, 0.0, 1e-6);
    CHECK_NEAR(r.final_euler.y, 0.0, 1e-6);
}

void test_recovers_from_offset() {
    State s;
    s.attitude = core::from_euler_zyx(0.3, -0.2, 0.4);
    const Result r = fly(s, Vec3d{}, 6.0);
    CHECK_NEAR(r.final_euler.x, 0.0, 0.02);
    CHECK_NEAR(r.final_euler.y, 0.0, 0.02);
    CHECK_NEAR(r.final_euler.z, 0.0, 0.02);
}

}

int main() {
    test_zero_output_at_setpoint();
    test_torque_saturates();
    test_yaw_error_wraps_the_short_way();
    test_roll_step();
    test_pitch_step();
    test_yaw_step();
    test_recovers_from_offset();
    return check::report();
}