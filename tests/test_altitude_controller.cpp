#include <algorithm>
#include <cmath>

#include "check.hpp"
#include "control/altitude_controller.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"

using core::AltitudeConfig;
using core::AltitudeController;
using core::Quatd;
using core::Vec3d;
using sim::Quad;
using sim::State;

namespace {

struct Run {
    double final_alt = 0.0;
    double final_climb = 0.0;
    double max_alt = -1e9;
    double min_alt = 1e9;
};

AltitudeConfig make_config(const QuadParams& p) {
    AltitudeConfig cfg;
    cfg.mass = p.mass;
    cfg.gravity = p.gravity;
    cfg.max_thrust = 4.0 * p.kThrust * p.omegaMax * p.omegaMax;
    return cfg;
}

double hover_omega(const QuadParams& p) {
    return std::sqrt(p.mass * p.gravity / (4.0 * p.kThrust));
}

Run fly(const AltitudeConfig& cfg, double start_alt, double alt_sp, double seconds) {
    const QuadParams p;
    Quad q(p);
    State s;
    s.position.z = -start_alt;
    q.reset(s, hover_omega(p));
    AltitudeController ctl(cfg);

    Run r;
    const double dt = 1e-3;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        const State& st = q.state();
        const double thrust = ctl.update(alt_sp, st.position, st.velocity, st.attitude, dt);
        q.step(thrust, Vec3d{}, dt);
        const double alt = -q.state().position.z;
        r.max_alt = std::max(r.max_alt, alt);
        r.min_alt = std::min(r.min_alt, alt);
    }
    r.final_alt = -q.state().position.z;
    r.final_climb = -q.state().velocity.z;
    return r;
}

void test_hover_thrust_is_weight() {
    AltitudeController ctl;
    const double t = ctl.update(0.0, Vec3d{}, Vec3d{}, Quatd{}, 1e-3);
    CHECK_NEAR(t, 9.81, 1e-12);
}

void test_tilt_compensation() {
    AltitudeController ctl;
    const Quatd q = core::from_euler_zyx(0.4, 0.0, 0.0);
    const double t = ctl.update(0.0, Vec3d{}, Vec3d{}, q, 1e-3);
    CHECK_NEAR(t, 9.81 / std::cos(0.4), 1e-9);
}

void test_acceleration_limits() {
    AltitudeController up;
    CHECK_NEAR(up.update(100.0, Vec3d{}, Vec3d{}, Quatd{}, 1e-3), 9.81 + 8.0, 1e-9);
    CHECK_NEAR(up.climb_rate_setpoint(), 3.0, 1e-12);

    AltitudeController down;
    CHECK_NEAR(down.update(-100.0, Vec3d{}, Vec3d{}, Quatd{}, 1e-3), 9.81 - 6.0, 1e-9);
    CHECK_NEAR(down.climb_rate_setpoint(), -2.0, 1e-12);
}

void test_thrust_saturates() {
    AltitudeConfig cfg;
    cfg.max_thrust = 12.0;
    AltitudeController ctl(cfg);
    CHECK_NEAR(ctl.update(100.0, Vec3d{}, Vec3d{}, Quatd{}, 1e-3), 12.0, 1e-12);
}

void test_holds_hover() {
    const QuadParams p;
    const Run r = fly(make_config(p), 0.0, 0.0, 5.0);
    CHECK_NEAR(r.final_alt, 0.0, 1e-6);
    CHECK_NEAR(r.final_climb, 0.0, 1e-6);
}

void test_climb_step() {
    const QuadParams p;
    const Run r = fly(make_config(p), 0.0, 2.0, 8.0);
    CHECK_NEAR(r.final_alt, 2.0, 0.01);
    CHECK_NEAR(r.final_climb, 0.0, 0.01);
    CHECK(r.max_alt < 2.1);
}

void test_descent_step() {
    const QuadParams p;
    const Run r = fly(make_config(p), 2.0, 1.0, 8.0);
    CHECK_NEAR(r.final_alt, 1.0, 0.01);
    CHECK_NEAR(r.final_climb, 0.0, 0.01);
    CHECK(r.min_alt > 0.9);
}

void test_integral_removes_mass_error() {
    const QuadParams p;

    AltitudeConfig no_integral = make_config(p);
    no_integral.mass = 0.9;
    const Run a = fly(no_integral, 1.0, 1.0, 30.0);
    CHECK(std::fabs(a.final_alt - 1.0) > 0.1);
    CHECK(std::fabs(a.final_alt - 1.0) < 0.2);

    AltitudeConfig with_integral = make_config(p);
    with_integral.mass = 0.9;
    with_integral.vel_ki = 1.0;
    const Run b = fly(with_integral, 1.0, 1.0, 30.0);
    CHECK_NEAR(b.final_alt, 1.0, 0.01);
}

}

int main() {
    test_hover_thrust_is_weight();
    test_tilt_compensation();
    test_acceleration_limits();
    test_thrust_saturates();
    test_holds_hover();
    test_climb_step();
    test_descent_step();
    test_integral_removes_mass_error();
    return check::report();
}