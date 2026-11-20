#include <cmath>

#include "check.hpp"
#include "math/vec3.hpp"
#include "sensors/camera.hpp"
#include "sensors/rng.hpp"

using core::Vec3d;
using sim::Camera;
using sim::CameraConfig;
using sim::CameraMeasurement;
using sim::Rng;

namespace {

struct Stats {
    int n = 0;
    double sum = 0.0;
    double sum_sq = 0.0;

    double mean() const { return sum / n; }
    double stddev() const {
        const double m = mean();
        return std::sqrt(sum_sq / n - m * m);
    }
};

CameraConfig clean_config() {
    CameraConfig cfg;
    cfg.delay_s = 0.0;
    cfg.sigma_base_m = 0.0;
    cfg.sigma_per_m = 0.0;
    cfg.dropout_prob = 0.0;
    return cfg;
}

Stats run_x(const CameraConfig& cfg, double altitude, double seconds) {
    Camera cam(cfg);
    Stats s;
    const double dt = 1e-3;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        const double t = i * dt;
        cam.capture(t, Vec3d{}, altitude);
        CameraMeasurement m;
        while (cam.poll(t, m)) {
            s.n += 1;
            s.sum += m.rel_pos.x;
            s.sum_sq += m.rel_pos.x * m.rel_pos.x;
        }
    }
    return s;
}

void test_rng_is_repeatable() {
    Rng a(42);
    Rng b(42);
    Rng c(43);
    bool differs = false;
    for (int i = 0; i < 100; ++i) {
        const std::uint64_t va = a.next();
        CHECK(va == b.next());
        if (va != c.next()) {
            differs = true;
        }
    }
    CHECK(differs);
}

void test_rng_uniform() {
    Rng r(7);
    double sum = 0.0;
    bool in_range = true;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const double u = r.uniform();
        if (u < 0.0 || u >= 1.0) {
            in_range = false;
        }
        sum += u;
    }
    CHECK(in_range);
    CHECK_NEAR(sum / n, 0.5, 0.01);
}

void test_rng_normal() {
    Rng r(11);
    double sum = 0.0;
    double sum_sq = 0.0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const double x = r.normal();
        sum += x;
        sum_sq += x * x;
    }
    const double mean = sum / n;
    const double sd = std::sqrt(sum_sq / n - mean * mean);
    CHECK_NEAR(mean, 0.0, 0.02);
    CHECK_NEAR(sd, 1.0, 0.02);
}

void test_frame_rate() {
    const Stats s = run_x(clean_config(), 1.0, 1.0);
    CHECK(s.n == 30);
}

void test_clean_camera_returns_truth() {
    Camera cam(clean_config());
    cam.capture(0.0, Vec3d{1.5, -2.0, 0.7}, 1.0);
    CameraMeasurement m;
    CHECK(cam.poll(0.0, m));
    CHECK_NEAR(m.rel_pos.x, 1.5, 1e-15);
    CHECK_NEAR(m.rel_pos.y, -2.0, 1e-15);
    CHECK_NEAR(m.rel_pos.z, 0.7, 1e-15);
}

void test_nothing_arrives_before_the_delay() {
    CameraConfig cfg = clean_config();
    cfg.delay_s = 0.05;
    Camera cam(cfg);
    int early = 0;
    CameraMeasurement m;
    for (int i = 0; i < 50; ++i) {
        const double t = i * 1e-3;
        cam.capture(t, Vec3d{}, 1.0);
        while (cam.poll(t, m)) {
            ++early;
        }
    }
    CHECK(early == 0);
    cam.capture(0.05, Vec3d{}, 1.0);
    CHECK(cam.poll(0.05, m));
    CHECK_NEAR(m.capture_time, 0.0, 1e-15);
}

void test_measurement_is_old_by_the_delay() {
    CameraConfig cfg = clean_config();
    cfg.delay_s = 0.05;
    Camera cam(cfg);
    const double dt = 1e-3;
    int received = 0;
    for (int i = 0; i < 2000; ++i) {
        const double t = i * dt;
        cam.capture(t, Vec3d{t, 0.0, 0.0}, 1.0);
        CameraMeasurement m;
        while (cam.poll(t, m)) {
            ++received;
            CHECK_NEAR(m.rel_pos.x, m.capture_time, 1e-12);
            CHECK(t - m.capture_time >= 0.05 - 1e-9);
            CHECK(t - m.capture_time <= 0.05 + 1.5e-3);
        }
    }
    CHECK(received > 50);
}

void test_dropout_fraction() {
    CameraConfig cfg = clean_config();
    cfg.dropout_prob = 0.3;
    const Stats s = run_x(cfg, 1.0, 100.0);
    const double fraction = static_cast<double>(s.n) / 3000.0;
    CHECK_NEAR(fraction, 0.7, 0.05);
}

void test_dropout_extremes() {
    CameraConfig none = clean_config();
    none.dropout_prob = 0.0;
    CHECK(run_x(none, 1.0, 10.0).n == 300);

    CameraConfig all = clean_config();
    all.dropout_prob = 1.0;
    CHECK(run_x(all, 1.0, 10.0).n == 0);
}

void test_noise_statistics() {
    CameraConfig cfg = clean_config();
    cfg.sigma_base_m = 0.1;
    const Stats s = run_x(cfg, 1.0, 100.0);
    CHECK_NEAR(s.mean(), 0.0, 0.01);
    CHECK_NEAR(s.stddev(), 0.1, 0.01);
}

void test_noise_grows_with_altitude() {
    CameraConfig cfg = clean_config();
    cfg.sigma_per_m = 0.1;
    const Stats low = run_x(cfg, 1.0, 100.0);
    const Stats high = run_x(cfg, 4.0, 100.0);
    CHECK_NEAR(low.stddev(), 0.1, 0.01);
    CHECK_NEAR(high.stddev(), 0.4, 0.04);
    CHECK(high.stddev() > 3.0 * low.stddev());
}

void test_same_seed_same_measurements() {
    CameraConfig cfg;
    cfg.seed = 5;
    Camera a(cfg);
    Camera b(cfg);
    CameraConfig other = cfg;
    other.seed = 6;
    Camera c(other);
    bool differs = false;
    for (int i = 0; i < 3000; ++i) {
        const double t = i * 1e-3;
        a.capture(t, Vec3d{1.0, 1.0, 0.0}, 1.0);
        b.capture(t, Vec3d{1.0, 1.0, 0.0}, 1.0);
        c.capture(t, Vec3d{1.0, 1.0, 0.0}, 1.0);
        CameraMeasurement ma;
        CameraMeasurement mb;
        CameraMeasurement mc;
        const bool ga = a.poll(t, ma);
        const bool gb = b.poll(t, mb);
        const bool gc = c.poll(t, mc);
        CHECK(ga == gb);
        if (ga && gb) {
            CHECK(ma.rel_pos.x == mb.rel_pos.x);
            CHECK(ma.rel_pos.y == mb.rel_pos.y);
        }
        if (ga && gc && ma.rel_pos.x != mc.rel_pos.x) {
            differs = true;
        }
    }
    CHECK(differs);
}

void test_reset_replays_the_sequence() {
    CameraConfig cfg;
    cfg.seed = 9;
    Camera cam(cfg);
    CameraMeasurement first;
    cam.capture(0.0, Vec3d{1.0, 0.0, 0.0}, 1.0);
    CHECK(cam.poll(0.05, first));
    cam.reset();
    CameraMeasurement again;
    cam.capture(0.0, Vec3d{1.0, 0.0, 0.0}, 1.0);
    CHECK(cam.poll(0.05, again));
    CHECK(first.rel_pos.x == again.rel_pos.x);
}

}

int main() {
    test_rng_is_repeatable();
    test_rng_uniform();
    test_rng_normal();
    test_frame_rate();
    test_clean_camera_returns_truth();
    test_nothing_arrives_before_the_delay();
    test_measurement_is_old_by_the_delay();
    test_dropout_fraction();
    test_dropout_extremes();
    test_noise_statistics();
    test_noise_grows_with_altitude();
    test_same_seed_same_measurements();
    test_reset_replays_the_sequence();
    return check::report();
}