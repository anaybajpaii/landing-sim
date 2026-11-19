#include <algorithm>
#include <cmath>

#include "check.hpp"
#include "control/altitude_controller.hpp"
#include "control/attitude_controller.hpp"
#include "control/position_controller.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"

using core::AltitudeConfig;
using core::AltitudeController;
using core::AttitudeController;
using core::PositionConfig;
using core::PositionController;
using core::Quatd;
using core::Vec3d;
using sim::Quad;
using sim::State;

namespace {

struct Target {
    Vec3d pos;
    Vec3d vel;
};

using TargetFn = Target (*)(double);

struct Flight {
    Vec3d pos;
    Vec3d vel;
    double max_tilt = 0.0;
    double min_alt = 1e9;
    double max_alt = -1e9;
};

double hover_omega(const QuadParams& p) {
    return std::sqrt(p.mass * p.gravity / (4.0 * p.kThrust));
}

AltitudeConfig make_alt_config(const QuadParams& p) {
    AltitudeConfig cfg;
    cfg.mass = p.mass;
    cfg.gravity = p.gravity;
    cfg.max_thrust = 4.0 * p.kThrust * p.omegaMax * p.omegaMax;
    return cfg;
}

Flight fly(TargetFn target_at, double seconds) {
    const QuadParams p;
    Quad q(p);
    State s0;
    s0.position.z = -1.0;
    q.reset(s0, hover_omega(p));

    AltitudeController altitude(make_alt_config(p));
    AttitudeController attitude;
    PositionController position;

    Flight f;
    const double dt = 1e-3;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        const double t = i * dt;
        const Target tgt = target_at(t);
        const State& s = q.state();
        const double thrust = altitude.update(1.0, s.position, s.velocity, s.attitude, dt);
        const Vec3d tilt =
            position.update(tgt.pos, tgt.vel, s.position, s.velocity, s.attitude, dt);
        const Vec3d torque =
            attitude.update(s.attitude, s.omega, Vec3d{tilt.x, tilt.y, 0.0}, dt);
        q.step(thrust, torque, dt);

        const State& n = q.state();
        const Vec3d e = core::euler_zyx(n.attitude);
        f.max_tilt = std::max(f.max_tilt, std::max(std::fabs(e.x), std::fabs(e.y)));
        const double alt = -n.position.z;
        f.min_alt = std::min(f.min_alt, alt);
        f.max_alt = std::max(f.max_alt, alt);
    }
    f.pos = q.state().position;
    f.vel = q.state().velocity;
    return f;
}

void test_zero_error_gives_level_attitude() {
    PositionController pc;
    const Vec3d tilt = pc.update(Vec3d{}, Vec3d{}, Vec3d{}, Vec3d{}, Quatd{}, 1e-3);
    CHECK_NEAR(tilt.x, 0.0, 1e-12);
    CHECK_NEAR(tilt.y, 0.0, 1e-12);
}

void test_forward_error_noses_down() {
    PositionController pc;
    const Vec3d tilt = pc.update(Vec3d{1.0, 0.0, 0.0}, Vec3d{}, Vec3d{}, Vec3d{}, Quatd{}, 1e-3);
    CHECK(tilt.y < 0.0);
    CHECK_NEAR(tilt.y, -std::atan(2.0 / 9.81), 1e-9);
    CHECK_NEAR(tilt.x, 0.0, 1e-12);
}

void test_rightward_error_rolls_right() {
    PositionController pc;
    const Vec3d tilt = pc.update(Vec3d{0.0, 1.0, 0.0}, Vec3d{}, Vec3d{}, Vec3d{}, Quatd{}, 1e-3);
    CHECK(tilt.x > 0.0);
    CHECK_NEAR(tilt.x, std::atan(2.0 / 9.81), 1e-9);
    CHECK_NEAR(tilt.y, 0.0, 1e-12);
}

void test_error_is_rotated_into_yaw_frame() {
    PositionController pc;
    const Quatd q = core::from_euler_zyx(0.0, 0.0, 1.5707963267948966);
    const Vec3d tilt = pc.update(Vec3d{1.0, 0.0, 0.0}, Vec3d{}, Vec3d{}, Vec3d{}, q, 1e-3);
    CHECK(tilt.x < 0.0);
    CHECK_NEAR(tilt.y, 0.0, 1e-9);
}

void test_acceleration_limit() {
    PositionController pc;
    const Vec3d tilt =
        pc.update(Vec3d{100.0, 0.0, 0.0}, Vec3d{}, Vec3d{}, Vec3d{}, Quatd{}, 1e-3);
    CHECK_NEAR(tilt.y, -std::atan(3.0 / 9.81), 1e-9);
    CHECK_NEAR(tilt.x, 0.0, 1e-12);
}

void test_tilt_geometry_matches_commanded_acceleration() {
    PositionConfig cfg;
    cfg.pos_kp = 0.0;
    cfg.vel_kp = 1.0;
    PositionController pc(cfg);
    const double yaw = 0.6;
    const Quatd att = core::from_euler_zyx(0.0, 0.0, yaw);
    const Vec3d tilt = pc.update(Vec3d{}, Vec3d{1.2, -0.8, 0.0}, Vec3d{}, Vec3d{}, att, 1e-3);
    const Quatd target = core::from_euler_zyx(tilt.x, tilt.y, yaw);
    const Vec3d d = core::rotate(target, Vec3d{0.0, 0.0, -1.0});
    CHECK_NEAR(d.x / -d.z, 1.2 / 9.81, 1e-9);
    CHECK_NEAR(d.y / -d.z, -0.8 / 9.81, 1e-9);
}

void test_flies_to_waypoint() {
    const Flight f = fly([](double) { return Target{Vec3d{3.0, 2.0, 0.0}, Vec3d{}}; }, 15.0);
    CHECK_NEAR(f.pos.x, 3.0, 0.02);
    CHECK_NEAR(f.pos.y, 2.0, 0.02);
    CHECK_NEAR(-f.pos.z, 1.0, 0.01);
    CHECK_NEAR(f.vel.x, 0.0, 0.02);
    CHECK_NEAR(f.vel.y, 0.0, 0.02);
    CHECK(f.max_tilt < std::atan(3.0 / 9.81) + 0.05);
    CHECK(f.min_alt > 0.7);
    CHECK(f.max_alt < 1.3);
}

void test_tracks_constant_velocity_target() {
    const Flight f = fly(
        [](double t) { return Target{Vec3d{0.5 * t, 0.0, 0.0}, Vec3d{0.5, 0.0, 0.0}}; }, 20.0);
    CHECK_NEAR(f.pos.x, 0.5 * 20.0, 0.03);
    CHECK_NEAR(f.pos.y, 0.0, 0.01);
    CHECK_NEAR(f.vel.x, 0.5, 0.02);
    CHECK_NEAR(f.vel.y, 0.0, 0.02);
}

}

int main() {
    test_zero_error_gives_level_attitude();
    test_forward_error_noses_down();
    test_rightward_error_rolls_right();
    test_error_is_rotated_into_yaw_frame();
    test_acceleration_limit();
    test_tilt_geometry_matches_commanded_acceleration();
    test_flies_to_waypoint();
    test_tracks_constant_velocity_target();
    return check::report();
}