#include "check.hpp"
#include "math/vec3.hpp"
#include "plant/platform.hpp"

using core::Vec3d;
using sim::Platform;

namespace {

void test_default_is_stationary() {
    Platform p;
    p.step(1.0);
    CHECK_NEAR(p.position.x, 0.0, 1e-15);
    CHECK_NEAR(p.position.y, 0.0, 1e-15);
    CHECK_NEAR(p.position.z, 0.0, 1e-15);
}

void test_moves_in_a_straight_line() {
    Platform p;
    p.position = Vec3d{2.0, -1.0, 0.0};
    p.velocity = Vec3d{0.5, 0.25, 0.0};
    for (int i = 0; i < 4000; ++i) {
        p.step(1e-3);
    }
    CHECK_NEAR(p.position.x, 2.0 + 0.5 * 4.0, 1e-9);
    CHECK_NEAR(p.position.y, -1.0 + 0.25 * 4.0, 1e-9);
    CHECK_NEAR(p.position.z, 0.0, 1e-15);
    CHECK_NEAR(p.velocity.x, 0.5, 1e-15);
}

void test_contains_xy() {
    Platform p;
    p.position = Vec3d{1.0, 2.0, 0.0};
    p.half_size = 0.5;
    CHECK(p.contains_xy(Vec3d{1.0, 2.0, 0.0}));
    CHECK(p.contains_xy(Vec3d{1.4, 1.6, 0.0}));
    CHECK(p.contains_xy(Vec3d{1.5, 2.5, 0.0}));
    CHECK(!p.contains_xy(Vec3d{1.6, 2.0, 0.0}));
    CHECK(!p.contains_xy(Vec3d{1.0, 1.4, 0.0}));
}

void test_contains_ignores_height() {
    Platform p;
    CHECK(p.contains_xy(Vec3d{0.1, 0.1, -5.0}));
}

void test_zone_follows_the_platform() {
    Platform p;
    p.velocity = Vec3d{0.5, 0.0, 0.0};
    const Vec3d probe{2.0, 0.0, 0.0};
    CHECK(!p.contains_xy(probe));
    for (int i = 0; i < 3500; ++i) {
        p.step(1e-3);
    }
    CHECK(p.contains_xy(probe));
}

}

int main() {
    test_default_is_stationary();
    test_moves_in_a_straight_line();
    test_contains_xy();
    test_contains_ignores_height();
    test_zone_follows_the_platform();
    return check::report();
}