#include <cmath>

#include "check.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"

using core::Quatd;
using core::Quatf;
using core::Vec3d;
using core::Vec3f;

namespace {

constexpr double kPi = 3.14159265358979323846;

void check_quat(const Quatd& q, double w, double x, double y, double z, double tol = 1e-12) {
    CHECK_NEAR(q.w, w, tol);
    CHECK_NEAR(q.x, x, tol);
    CHECK_NEAR(q.y, y, tol);
    CHECK_NEAR(q.z, z, tol);
}

void check_vec(const Vec3d& v, double x, double y, double z, double tol = 1e-12) {
    CHECK_NEAR(v.x, x, tol);
    CHECK_NEAR(v.y, y, tol);
    CHECK_NEAR(v.z, z, tol);
}

void test_integrate_zero_rate() {
    const Quatd q = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    const Quatd r = core::integrate(q, Vec3d{0.0, 0.0, 0.0}, 0.01);
    check_quat(r, q.w, q.x, q.y, q.z);
}

void test_integrate_spin_about_z() {
    const Quatd expected = core::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, 1.0);

    Quatd q{};
    for (int i = 0; i < 100; ++i) {
        q = core::integrate(q, Vec3d{0.0, 0.0, 1.0}, 0.01);
    }
    check_quat(q, expected.w, expected.x, expected.y, expected.z);

    const Quatd one_step = core::integrate(Quatd{}, Vec3d{0.0, 0.0, 1.0}, 1.0);
    check_quat(one_step, expected.w, expected.x, expected.y, expected.z);
}

void test_integrate_uses_body_frame_rates() {
    const Vec3d ex{1.0, 0.0, 0.0};
    const Vec3d ez{0.0, 0.0, 1.0};
    const Quatd q0 = core::from_axis_angle(ex, kPi / 2.0);

    Quatd q = q0;
    for (int i = 0; i < 50; ++i) {
        q = core::integrate(q, Vec3d{0.0, 0.0, 2.0}, 0.01);
    }

    const Quatd body = q0 * core::from_axis_angle(ez, 1.0);
    const Quatd world = core::from_axis_angle(ez, 1.0) * q0;
    check_quat(q, body.w, body.x, body.y, body.z);
    const Vec3d body_z_in_world = core::rotate(q, ez);
    check_vec(body_z_in_world, 0.0, -1.0, 0.0);
    CHECK(std::fabs(q.y - world.y) > 1e-3);
}

void test_integrate_matches_derivative() {
    const Quatd q = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    const Vec3d omega{0.4, -1.2, 0.8};
    const double dt = 1e-7;
    const Quatd next = core::integrate(q, omega, dt);
    const Quatd qdot = q * Quatd{0.0, omega.x, omega.y, omega.z};
    CHECK_NEAR((next.w - q.w) / dt, 0.5 * qdot.w, 1e-5);
    CHECK_NEAR((next.x - q.x) / dt, 0.5 * qdot.x, 1e-5);
    CHECK_NEAR((next.y - q.y) / dt, 0.5 * qdot.y, 1e-5);
    CHECK_NEAR((next.z - q.z) / dt, 0.5 * qdot.z, 1e-5);
}

void test_integrate_keeps_unit_norm() {
    Quatd q{};
    for (int i = 0; i < 100000; ++i) {
        const double t = i * 0.001;
        q = core::integrate(q, Vec3d{0.3 + 0.1 * std::sin(t), -0.2, 0.5}, 0.001);
    }
    CHECK_NEAR(core::norm(q), 1.0, 1e-12);

    Quatf qf{};
    for (int i = 0; i < 100000; ++i) {
        qf = core::integrate(qf, Vec3f{0.3f, -0.2f, 0.5f}, 0.001f);
    }
    CHECK_NEAR(static_cast<double>(core::norm(qf)), 1.0, 1e-5);
}

void test_euler_identity_and_single_axes() {
    const Vec3d e0 = core::euler_zyx(Quatd{});
    check_vec(e0, 0.0, 0.0, 0.0);
    check_vec(core::euler_zyx(core::from_euler_zyx(0.3, 0.0, 0.0)), 0.3, 0.0, 0.0);
    check_vec(core::euler_zyx(core::from_euler_zyx(0.0, 0.4, 0.0)), 0.0, 0.4, 0.0);
    check_vec(core::euler_zyx(core::from_euler_zyx(0.0, 0.0, -1.0)), 0.0, 0.0, -1.0);
}

void test_euler_round_trip() {
    const double rolls[] = {-3.0, -1.2, 0.0, 0.5, 2.9};
    const double pitches[] = {-1.4, -0.6, 0.0, 0.7, 1.4};
    const double yaws[] = {-3.1, -0.8, 0.0, 1.3, 3.0};
    for (const double r : rolls) {
        for (const double p : pitches) {
            for (const double y : yaws) {
                const Vec3d e = core::euler_zyx(core::from_euler_zyx(r, p, y));
                CHECK_NEAR(e.x, r, 1e-10);
                CHECK_NEAR(e.y, p, 1e-10);
                CHECK_NEAR(e.z, y, 1e-10);
            }
        }
    }
}

void test_euler_is_yaw_then_pitch_then_roll() {
    const double r = 0.4;
    const double p = -0.3;
    const double y = 1.1;
    const Quatd expected = core::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, y) *
                           core::from_axis_angle(Vec3d{0.0, 1.0, 0.0}, p) *
                           core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, r);
    const Quatd q = core::from_euler_zyx(r, p, y);
    check_quat(q, expected.w, expected.x, expected.y, expected.z);
}

void test_euler_ned_signs() {
    const Vec3d forward{1.0, 0.0, 0.0};
    const Vec3d right{0.0, 1.0, 0.0};
    const double r30 = kPi / 6.0;
    const Quatd yaw90 = core::from_euler_zyx(0.0, 0.0, kPi / 2.0);
    check_vec(core::rotate(yaw90, forward), 0.0, 1.0, 0.0);
    const Quatd nose_up = core::from_euler_zyx(0.0, r30, 0.0);
    check_vec(core::rotate(nose_up, forward), std::cos(r30), 0.0, -std::sin(r30));
    const Quatd roll_right = core::from_euler_zyx(r30, 0.0, 0.0);
    check_vec(core::rotate(roll_right, right), 0.0, std::cos(r30), std::sin(r30));
}

void test_euler_pitch_at_limit_is_finite() {
    const Quatd q = core::from_euler_zyx(0.2, kPi / 2.0, -0.4);
    const Vec3d e = core::euler_zyx(q);
    CHECK(std::isfinite(e.x));
    CHECK(std::isfinite(e.y));
    CHECK(std::isfinite(e.z));
    CHECK_NEAR(e.y, kPi / 2.0, 1e-6);
}

void test_euler_clamps_rounding_overshoot() {
    const Quatd q{0.7071068, 0.0, 0.7071068, 0.0};
    const Vec3d e = core::euler_zyx(q);
    CHECK(std::isfinite(e.y));
    CHECK_NEAR(e.y, kPi / 2.0, 1e-9);
}

void test_euler_float() {
    const Quatf q = core::from_euler_zyx(0.3f, -0.2f, 1.0f);
    const Vec3f e = core::euler_zyx(q);
    CHECK_NEAR(static_cast<double>(e.x), 0.3, 1e-5);
    CHECK_NEAR(static_cast<double>(e.y), -0.2, 1e-5);
    CHECK_NEAR(static_cast<double>(e.z), 1.0, 1e-5);
}

}

int main() {
    test_integrate_zero_rate();
    test_integrate_spin_about_z();
    test_integrate_uses_body_frame_rates();
    test_integrate_matches_derivative();
    test_integrate_keeps_unit_norm();
    test_euler_identity_and_single_axes();
    test_euler_round_trip();
    test_euler_is_yaw_then_pitch_then_roll();
    test_euler_ned_signs();
    test_euler_pitch_at_limit_is_finite();
    test_euler_clamps_rounding_overshoot();
    test_euler_float();
    return check::report();
}