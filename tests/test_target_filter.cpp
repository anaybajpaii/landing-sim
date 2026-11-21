#include <cmath>
#include <cstddef>

#include "check.hpp"
#include "estimation/history.hpp"
#include "estimation/target_filter.hpp"
#include "math/vec3.hpp"
#include "sensors/camera.hpp"

using core::PositionHistory;
using core::TargetEstimate;
using core::TargetFilter;
using core::Vec3d;
using sim::Camera;
using sim::CameraConfig;
using sim::CameraMeasurement;

namespace {

constexpr double kFrameDt = 1.0 / 30.0;

Vec3d truth_pos(double t) {
    return Vec3d{1.0 + 0.5 * t, -2.0 + 0.25 * t, 0.0};
}

double feed_clean(TargetFilter& f, int updates) {
    double t = 0.0;
    for (int k = 0; k <= updates; ++k) {
        t = k * kFrameDt;
        f.update(t, truth_pos(t), 0.02);
    }
    return t;
}

void test_history_interpolates() {
    PositionHistory<8> h;
    h.push(0.0, Vec3d{0.0, 0.0, 0.0});
    h.push(1.0, Vec3d{1.0, 2.0, 3.0});
    h.push(2.0, Vec3d{2.0, 4.0, 6.0});
    Vec3d p;
    CHECK(h.at(0.5, p));
    CHECK_NEAR(p.x, 0.5, 1e-12);
    CHECK_NEAR(p.y, 1.0, 1e-12);
    CHECK_NEAR(p.z, 1.5, 1e-12);
    CHECK(h.at(1.5, p));
    CHECK_NEAR(p.x, 1.5, 1e-12);
    CHECK_NEAR(p.y, 3.0, 1e-12);
    CHECK_NEAR(p.z, 4.5, 1e-12);
    CHECK(h.at(2.0, p));
    CHECK_NEAR(p.x, 2.0, 1e-12);
    CHECK(h.at(0.0, p));
    CHECK_NEAR(p.x, 0.0, 1e-12);
    CHECK(!h.at(2.1, p));
    CHECK(!h.at(-0.1, p));
}

void test_history_wraps_when_full() {
    PositionHistory<4> h;
    for (int i = 0; i < 10; ++i) {
        h.push(static_cast<double>(i), Vec3d{static_cast<double>(i), 0.0, 0.0});
    }
    CHECK(h.size() == 4);
    Vec3d p;
    CHECK(!h.at(5.5, p));
    CHECK(h.at(6.5, p));
    CHECK_NEAR(p.x, 6.5, 1e-12);
    CHECK(h.at(9.0, p));
    CHECK_NEAR(p.x, 9.0, 1e-12);
}

void test_history_edge_cases() {
    PositionHistory<4> h;
    Vec3d p;
    CHECK(!h.at(0.0, p));
    h.push(1.0, Vec3d{4.0, 5.0, 6.0});
    CHECK(h.at(1.0, p));
    CHECK_NEAR(p.y, 5.0, 1e-12);
    CHECK(!h.at(1.1, p));
}

void test_first_measurement_initializes() {
    TargetFilter f;
    CHECK(!f.initialized());
    CHECK(!f.estimate(0.0).valid);
    CHECK(f.update(1.0, Vec3d{2.0, 3.0, 0.0}, 0.02));
    CHECK(f.initialized());
    const TargetEstimate e = f.estimate(1.0);
    CHECK(e.valid);
    CHECK_NEAR(e.position.x, 2.0, 1e-12);
    CHECK_NEAR(e.position.y, 3.0, 1e-12);
    CHECK_NEAR(e.velocity.x, 0.0, 1e-12);
    CHECK_NEAR(e.velocity.y, 0.0, 1e-12);
    CHECK_NEAR(e.pos_sigma, 0.02, 1e-12);
    CHECK_NEAR(e.age_s, 0.0, 1e-12);
}

void test_converges_on_constant_velocity() {
    TargetFilter f;
    const double t = feed_clean(f, 300);
    const TargetEstimate e = f.estimate(t);
    const Vec3d truth = truth_pos(t);
    CHECK_NEAR(e.position.x, truth.x, 1e-3);
    CHECK_NEAR(e.position.y, truth.y, 1e-3);
    CHECK_NEAR(e.velocity.x, 0.5, 1e-3);
    CHECK_NEAR(e.velocity.y, 0.25, 1e-3);
}

void test_extrapolates_to_now() {
    TargetFilter f;
    const double t = feed_clean(f, 300);
    const TargetEstimate e = f.estimate(t + 0.05);
    const Vec3d truth = truth_pos(t + 0.05);
    CHECK_NEAR(e.position.x, truth.x, 2e-3);
    CHECK_NEAR(e.position.y, truth.y, 2e-3);
    CHECK_NEAR(e.age_s, 0.05, 1e-9);
}

void test_coasts_through_a_dropout() {
    TargetFilter f;
    const double t = feed_clean(f, 300);
    const TargetEstimate now = f.estimate(t);
    const TargetEstimate later = f.estimate(t + 1.0);
    const Vec3d truth = truth_pos(t + 1.0);
    CHECK_NEAR(later.position.x, truth.x, 5e-3);
    CHECK_NEAR(later.position.y, truth.y, 5e-3);
    CHECK(later.pos_sigma > now.pos_sigma);
}

void test_uncertainty_shrinks_with_updates() {
    TargetFilter f;
    f.update(0.0, truth_pos(0.0), 0.02);
    const double start = f.estimate(0.0).pos_sigma;
    const double t = feed_clean(f, 300);
    const double end = f.estimate(t).pos_sigma;
    CHECK_NEAR(start, 0.02, 1e-9);
    CHECK(end < 0.015);
}

void test_out_of_order_measurement_is_rejected() {
    TargetFilter f;
    f.update(1.0, Vec3d{0.0, 0.0, 0.0}, 0.02);
    f.update(2.0, Vec3d{0.5, 0.0, 0.0}, 0.02);
    const TargetEstimate before = f.estimate(2.0);
    CHECK(!f.update(1.5, Vec3d{100.0, 100.0, 0.0}, 0.02));
    const TargetEstimate after = f.estimate(2.0);
    CHECK_NEAR(after.position.x, before.position.x, 1e-15);
    CHECK_NEAR(after.position.y, before.position.y, 1e-15);
    CHECK_NEAR(after.velocity.x, before.velocity.x, 1e-15);
}

void test_reset_clears_the_filter() {
    TargetFilter f;
    f.update(1.0, Vec3d{1.0, 1.0, 0.0}, 0.02);
    f.reset();
    CHECK(!f.initialized());
    CHECK(!f.estimate(1.0).valid);
}

void test_tracks_through_noisy_delayed_camera() {
    CameraConfig cam_cfg;
    cam_cfg.seed = 3;
    Camera camera(cam_cfg);
    TargetFilter filter;
    PositionHistory<256> history;

    const Vec3d quad{0.0, 0.0, -1.0};
    const double altitude = 1.0;
    const double sigma = cam_cfg.sigma_base_m + cam_cfg.sigma_per_m * altitude;
    const double dt = 1e-3;

    double sum_pos = 0.0;
    double sum_pos_sq = 0.0;
    double sum_vel_sq = 0.0;
    int n = 0;
    bool always_valid = true;

    for (int i = 0; i < 20000; ++i) {
        const double t = i * dt;
        const Vec3d platform{3.0 + 0.5 * t, 0.0, 0.0};

        history.push(t, quad);
        camera.capture(t, Vec3d{platform.x - quad.x, platform.y - quad.y, 0.0}, altitude);
        CameraMeasurement m;
        while (camera.poll(t, m)) {
            Vec3d quad_then;
            if (history.at(m.capture_time, quad_then)) {
                filter.update(m.capture_time,
                              Vec3d{quad_then.x + m.rel_pos.x, quad_then.y + m.rel_pos.y, 0.0},
                              sigma);
            }
        }

        if (t >= 5.0) {
            const TargetEstimate e = filter.estimate(t);
            if (!e.valid) {
                always_valid = false;
                continue;
            }
            const double pe = e.position.x - platform.x;
            const double ve = e.velocity.x - 0.5;
            sum_pos += pe;
            sum_pos_sq += pe * pe;
            sum_vel_sq += ve * ve;
            ++n;
        }
    }

    CHECK(always_valid);
    CHECK(n > 0);
    const double pos_rms = std::sqrt(sum_pos_sq / n);
    const double vel_rms = std::sqrt(sum_vel_sq / n);
    CHECK(pos_rms < 0.03);
    CHECK(vel_rms < 0.15);
    CHECK(std::fabs(sum_pos / n) < 0.01);
}

}

int main() {
    test_history_interpolates();
    test_history_wraps_when_full();
    test_history_edge_cases();
    test_first_measurement_initializes();
    test_converges_on_constant_velocity();
    test_extrapolates_to_now();
    test_coasts_through_a_dropout();
    test_uncertainty_shrinks_with_updates();
    test_out_of_order_measurement_is_rejected();
    test_reset_clears_the_filter();
    test_tracks_through_noisy_delayed_camera();
    return check::report();
}