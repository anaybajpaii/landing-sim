#include "check.hpp"
#include "math/vec3.hpp"

using core::Vec3d;
using core::Vec3f;

namespace {

void check_vec(const Vec3d& v, double x, double y, double z, double tol = 1e-12) {
    CHECK_NEAR(v.x, x, tol);
    CHECK_NEAR(v.y, y, tol);
    CHECK_NEAR(v.z, z, tol);
}

void test_default_and_aggregate() {
    const Vec3d zero{};
    check_vec(zero, 0.0, 0.0, 0.0);
    const Vec3d v{1.0, 2.0, 3.0};
    check_vec(v, 1.0, 2.0, 3.0);
}

void test_arithmetic() {
    const Vec3d a{1.0, 2.0, 3.0};
    const Vec3d b{-4.0, 0.5, 2.0};
    check_vec(a + b, -3.0, 2.5, 5.0);
    check_vec(a - b, 5.0, 1.5, 1.0);
    check_vec(-a, -1.0, -2.0, -3.0);
    check_vec(a * 2.0, 2.0, 4.0, 6.0);
    check_vec(2.0 * a, 2.0, 4.0, 6.0);
    check_vec(a / 2.0, 0.5, 1.0, 1.5);

    Vec3d c = a;
    c += b;
    check_vec(c, -3.0, 2.5, 5.0);
    c -= b;
    check_vec(c, 1.0, 2.0, 3.0);
    c *= 3.0;
    check_vec(c, 3.0, 6.0, 9.0);
    c /= 3.0;
    check_vec(c, 1.0, 2.0, 3.0);
}

void test_dot() {
    const Vec3d a{1.0, 2.0, 3.0};
    const Vec3d b{-4.0, 0.5, 2.0};
    CHECK_NEAR(core::dot(a, b), 3.0, 1e-12);
    CHECK_NEAR(core::dot(a, b), core::dot(b, a), 1e-12);
    CHECK_NEAR(core::dot(Vec3d{1, 0, 0}, Vec3d{0, 1, 0}), 0.0, 1e-12);
}

void test_cross_basis() {
    const Vec3d ex{1.0, 0.0, 0.0};
    const Vec3d ey{0.0, 1.0, 0.0};
    const Vec3d ez{0.0, 0.0, 1.0};
    check_vec(core::cross(ex, ey), 0.0, 0.0, 1.0);
    check_vec(core::cross(ey, ez), 1.0, 0.0, 0.0);
    check_vec(core::cross(ez, ex), 0.0, 1.0, 0.0);
    check_vec(core::cross(ey, ex), 0.0, 0.0, -1.0);
}

void test_cross_identities() {
    const Vec3d a{1.0, -2.0, 0.5};
    const Vec3d b{3.0, 0.25, -1.5};
    const Vec3d axb = core::cross(a, b);
    const Vec3d bxa = core::cross(b, a);
    check_vec(axb, -bxa.x, -bxa.y, -bxa.z);
    check_vec(core::cross(a, a), 0.0, 0.0, 0.0);
    CHECK_NEAR(core::dot(a, axb), 0.0, 1e-12);
    CHECK_NEAR(core::dot(b, axb), 0.0, 1e-12);
    const double lhs = core::norm_sq(axb);
    const double rhs = core::norm_sq(a) * core::norm_sq(b) - core::dot(a, b) * core::dot(a, b);
    CHECK_NEAR(lhs, rhs, 1e-12);
}

void test_norm() {
    const Vec3d v{3.0, 4.0, 12.0};
    CHECK_NEAR(core::norm_sq(v), 169.0, 1e-12);
    CHECK_NEAR(core::norm(v), 13.0, 1e-12);
}

void test_normalized() {
    const Vec3d v{3.0, 4.0, 12.0};
    const Vec3d u = core::normalized(v);
    CHECK_NEAR(core::norm(u), 1.0, 1e-12);
    check_vec(u, 3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0);
    check_vec(core::normalized(Vec3d{}), 0.0, 0.0, 0.0);
    check_vec(core::normalized(Vec3d{1e-15, 0.0, 0.0}), 0.0, 0.0, 0.0);
}

void test_float() {
    const Vec3f a{1.0f, 2.0f, 3.0f};
    const Vec3f b{-4.0f, 0.5f, 2.0f};
    const Vec3f c = core::cross(a, b);
    CHECK_NEAR(static_cast<double>(c.x), 2.5, 1e-5);
    CHECK_NEAR(static_cast<double>(c.y), -14.0, 1e-5);
    CHECK_NEAR(static_cast<double>(c.z), 8.5, 1e-5);
    CHECK_NEAR(static_cast<double>(core::norm(Vec3f{3.0f, 4.0f, 12.0f})), 13.0, 1e-5);
}

}

int main() {
    test_default_and_aggregate();
    test_arithmetic();
    test_dot();
    test_cross_basis();
    test_cross_identities();
    test_norm();
    test_normalized();
    test_float();
    return check::report();
}