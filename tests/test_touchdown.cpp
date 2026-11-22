#include <cmath>

#include "check.hpp"
#include "guidance/landing_state_machine.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "pipeline/landing_pipeline.hpp"
#include "pipeline/touchdown.hpp"
#include "plant/platform.hpp"
#include "plant/rigid_body.hpp"

using core::Vec3d;
using sim::TouchdownCriteria;
using sim::TouchdownResult;

namespace {

sim::Platform make_platform() {
    sim::Platform p;
    p.position = Vec3d{2.0, 1.0, 0.0};
    p.velocity = Vec3d{0.5, 0.0, 0.0};
    p.half_size = 0.5;
    return p;
}

sim::State make_quad(double dx, double dy, double vz, double dvx, double roll) {
    sim::State s;
    s.position = Vec3d{2.0 + dx, 1.0 + dy, -0.05};
    s.velocity = Vec3d{0.5 + dvx, 0.0, vz};
    s.attitude = core::from_euler_zyx(roll, 0.0, 0.0);
    return s;
}

void test_contact_height() {
    const TouchdownCriteria c;
    CHECK(c.in_contact(0.05));
    CHECK(c.in_contact(0.01));
    CHECK(c.in_contact(-0.1));
    CHECK(!c.in_contact(0.0501));
    CHECK(!c.in_contact(1.0));
}

void test_clean_touchdown_succeeds() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 9.5, make_quad(0.1, -0.1, 0.2, 0.1, 0.05), make_platform());
    CHECK(r.touched);
    CHECK(r.success);
    CHECK(r.on_platform);
    CHECK(r.vertical_ok);
    CHECK(r.rel_speed_ok);
    CHECK(r.tilt_ok);
    CHECK_NEAR(r.time, 9.5, 1e-15);
    CHECK_NEAR(r.vertical_speed, 0.2, 1e-15);
    CHECK_NEAR(r.rel_speed, 0.1, 1e-12);
    CHECK_NEAR(r.tilt, 0.05, 1e-9);
    CHECK_NEAR(r.offset, std::sqrt(0.02), 1e-12);
}

void test_off_platform_fails() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 1.0, make_quad(0.6, 0.0, 0.2, 0.0, 0.0), make_platform());
    CHECK(r.touched);
    CHECK(!r.on_platform);
    CHECK(r.vertical_ok);
    CHECK(r.rel_speed_ok);
    CHECK(r.tilt_ok);
    CHECK(!r.success);
}

void test_hard_landing_fails() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 1.0, make_quad(0.0, 0.0, 0.8, 0.0, 0.0), make_platform());
    CHECK(r.on_platform);
    CHECK(!r.vertical_ok);
    CHECK(!r.success);
}

void test_relative_speed_fails() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 1.0, make_quad(0.0, 0.0, 0.2, 0.4, 0.0), make_platform());
    CHECK(r.on_platform);
    CHECK(r.vertical_ok);
    CHECK(!r.rel_speed_ok);
    CHECK(!r.success);
}

void test_tilt_fails() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 1.0, make_quad(0.0, 0.0, 0.2, 0.0, 0.3), make_platform());
    CHECK_NEAR(r.tilt, 0.3, 1e-9);
    CHECK(!r.tilt_ok);
    CHECK(!r.success);
}

void test_upward_motion_is_not_a_hard_landing() {
    const TouchdownCriteria c;
    const TouchdownResult r =
        sim::evaluate_touchdown(c, 1.0, make_quad(0.0, 0.0, -0.3, 0.0, 0.0), make_platform());
    CHECK_NEAR(r.vertical_speed, -0.3, 1e-15);
    CHECK(r.vertical_ok);
}

void test_closed_loop_touchdown_succeeds() {
    sim::LandingPipeline p;
    double t_descend = -1.0;
    for (int i = 0; i < 40000 && !p.touchdown().touched; ++i) {
        p.step();
        if (p.last_output().state == core::LandingState::Descend && t_descend < 0.0) {
            t_descend = p.time();
        }
    }
    const TouchdownResult r = p.touchdown();
    CHECK(r.touched);
    CHECK(r.success);
    CHECK(r.on_platform);
    CHECK(r.vertical_ok);
    CHECK(r.rel_speed_ok);
    CHECK(r.tilt_ok);
    CHECK(r.vertical_speed < 0.5);
    CHECK(r.rel_speed < 0.3);
    CHECK(r.tilt < 0.26);
    CHECK(t_descend > 0.0);
    CHECK(r.time > t_descend);
    CHECK(r.time < 20.0);
    CHECK_NEAR(r.time, p.time(), 1e-9);

    for (int i = 0; i < 2000; ++i) {
        p.step();
    }
    CHECK_NEAR(p.altitude(), 0.05, 1e-12);
    CHECK_NEAR(p.state().velocity.z, 0.0, 1e-12);
    CHECK(p.platform().contains_xy(p.state().position));
    CHECK_NEAR(p.touchdown().time, r.time, 1e-15);
}

void test_closed_loop_off_platform_is_not_a_success() {
    sim::PipelineConfig cfg;
    cfg.platform_half_size = 0.0;
    sim::LandingPipeline p(cfg);
    for (int i = 0; i < 40000 && !p.touchdown().touched; ++i) {
        p.step();
    }
    CHECK(p.touchdown().touched);
    CHECK(!p.touchdown().on_platform);
    CHECK(!p.touchdown().success);
}

void test_no_touchdown_without_a_target() {
    sim::PipelineConfig cfg;
    cfg.camera.dropout_prob = 1.0;
    sim::LandingPipeline p(cfg);
    for (int i = 0; i < 15000; ++i) {
        p.step();
    }
    CHECK(!p.touchdown().touched);
    CHECK_NEAR(p.altitude(), 1.0, 0.05);
}

}

int main() {
    test_contact_height();
    test_clean_touchdown_succeeds();
    test_off_platform_fails();
    test_hard_landing_fails();
    test_relative_speed_fails();
    test_tilt_fails();
    test_upward_motion_is_not_a_hard_landing();
    test_closed_loop_touchdown_succeeds();
    test_closed_loop_off_platform_is_not_a_success();
    test_no_touchdown_without_a_target();
    return check::report();
}