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

void test_identity_default() {
    const Quatd q{};
    check_quat(q, 1.0, 0.0, 0.0, 0.0);
    const Quatd r{0.5, 0.5, 0.5, 0.5};
    check_quat(r, 0.5, 0.5, 0.5, 0.5);
}

void test_conjugate() {
    const Quatd q{0.1, 0.2, -0.3, 0.4};
    check_quat(core::conjugate(q), 0.1, -0.2, 0.3, -0.4);
}

void test_norm_and_normalized() {
    const Quatd q{1.0, 2.0, 3.0, 4.0};
    CHECK_NEAR(core::norm(q), std::sqrt(30.0), 1e-12);
    const Quatd u = core::normalized(q);
    CHECK_NEAR(core::norm(u), 1.0, 1e-12);
    const double n = std::sqrt(30.0);
    check_quat(u, 1.0 / n, 2.0 / n, 3.0 / n, 4.0 / n);
    check_quat(core::normalized(Quatd{0.0, 0.0, 0.0, 0.0}), 1.0, 0.0, 0.0, 0.0);
}

void test_multiply_hamilton_rules() {
    const Quatd i{0.0, 1.0, 0.0, 0.0};
    const Quatd j{0.0, 0.0, 1.0, 0.0};
    const Quatd k{0.0, 0.0, 0.0, 1.0};
    check_quat(i * j, 0.0, 0.0, 0.0, 1.0);
    check_quat(j * k, 0.0, 1.0, 0.0, 0.0);
    check_quat(k * i, 0.0, 0.0, 1.0, 0.0);
    check_quat(j * i, 0.0, 0.0, 0.0, -1.0);
    check_quat(i * i, -1.0, 0.0, 0.0, 0.0);
}

void test_multiply_identity_and_inverse() {
    const Quatd q = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    const Quatd id{};
    check_quat(id * q, q.w, q.x, q.y, q.z);
    check_quat(q * id, q.w, q.x, q.y, q.z);
    check_quat(q * core::conjugate(q), 1.0, 0.0, 0.0, 0.0);
    check_quat(core::conjugate(q) * q, 1.0, 0.0, 0.0, 0.0);
}

void test_axis_angle() {
    const Vec3d ez{0.0, 0.0, 1.0};
    const double c = std::cos(kPi / 4.0);
    check_quat(core::from_axis_angle(ez, kPi / 2.0), c, 0.0, 0.0, c);
    check_quat(core::from_axis_angle(Vec3d{0.0, 0.0, 2.0}, kPi / 2.0), c, 0.0, 0.0, c);
    check_quat(core::from_axis_angle(ez, 0.0), 1.0, 0.0, 0.0, 0.0);
    check_quat(core::from_axis_angle(Vec3d{0.0, 0.0, 0.0}, 1.0), 1.0, 0.0, 0.0, 0.0);
    check_quat(core::from_axis_angle(ez, kPi), 0.0, 0.0, 0.0, 1.0, 1e-12);
}

void test_rotate_basis() {
    const Vec3d ex{1.0, 0.0, 0.0};
    const Vec3d ey{0.0, 1.0, 0.0};
    const Vec3d ez{0.0, 0.0, 1.0};
    const double q90 = kPi / 2.0;
    check_vec(core::rotate(core::from_axis_angle(ez, q90), ex), 0.0, 1.0, 0.0);
    check_vec(core::rotate(core::from_axis_angle(ex, q90), ey), 0.0, 0.0, 1.0);
    check_vec(core::rotate(core::from_axis_angle(ey, q90), ez), 1.0, 0.0, 0.0);
    check_vec(core::rotate(core::from_axis_angle(ez, kPi), ex), -1.0, 0.0, 0.0);
    check_vec(core::rotate(Quatd{}, Vec3d{1.0, 2.0, 3.0}), 1.0, 2.0, 3.0);
}

void test_rotate_preserves_length() {
    const Quatd q = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    const Vec3d v{1.5, -2.0, 0.75};
    CHECK_NEAR(core::norm(core::rotate(q, v)), core::norm(v), 1e-12);
}

void test_rotate_composition() {
    const Quatd q1 = core::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, 0.7);
    const Quatd q2 = core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, 1.2);
    const Vec3d v{0.4, -1.1, 2.0};
    const Vec3d combined = core::rotate(q1 * q2, v);
    const Vec3d stepwise = core::rotate(q1, core::rotate(q2, v));
    check_vec(combined, stepwise.x, stepwise.y, stepwise.z);
    const Vec3d swapped = core::rotate(q2 * q1, v);
    CHECK(std::fabs(swapped.x - stepwise.x) > 1e-3);
}

void test_conjugate_undoes_rotation() {
    const Quatd q = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    const Vec3d v{1.5, -2.0, 0.75};
    check_vec(core::rotate(core::conjugate(q), core::rotate(q, v)), v.x, v.y, v.z);
}

void test_ned_frd_convention() {
    const Vec3d thrust{0.0, 0.0, -1.0};
    const Vec3d forward{1.0, 0.0, 0.0};
    const double q90 = kPi / 2.0;
    const Quatd roll_right = core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, q90);
    const Quatd nose_up = core::from_axis_angle(Vec3d{0.0, 1.0, 0.0}, q90);
    const Quatd yaw_right = core::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, q90);
    check_vec(core::rotate(Quatd{}, thrust), 0.0, 0.0, -1.0);
    check_vec(core::rotate(roll_right, thrust), 0.0, 1.0, 0.0);
    check_vec(core::rotate(nose_up, thrust), -1.0, 0.0, 0.0);
    check_vec(core::rotate(yaw_right, thrust), 0.0, 0.0, -1.0);
    check_vec(core::rotate(yaw_right, forward), 0.0, 1.0, 0.0);
}

void test_float() {
    const Quatf q = core::from_axis_angle(Vec3f{0.0f, 0.0f, 1.0f}, 1.57079632679f);
    const Vec3f r = core::rotate(q, Vec3f{1.0f, 0.0f, 0.0f});
    CHECK_NEAR(static_cast<double>(r.x), 0.0, 1e-5);
    CHECK_NEAR(static_cast<double>(r.y), 1.0, 1e-5);
    CHECK_NEAR(static_cast<double>(r.z), 0.0, 1e-5);
    CHECK_NEAR(static_cast<double>(core::norm(q)), 1.0, 1e-5);
}

}

int main() {
    test_identity_default();
    test_conjugate();
    test_norm_and_normalized();
    test_multiply_hamilton_rules();
    test_multiply_identity_and_inverse();
    test_axis_angle();
    test_rotate_basis();
    test_rotate_preserves_length();
    test_rotate_composition();
    test_conjugate_undoes_rotation();
    test_ned_frd_convention();
    test_float();
    return check::report();
}