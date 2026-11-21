#include <algorithm>
#include <cmath>

#include "check.hpp"
#include "control/altitude_controller.hpp"
#include "control/attitude_controller.hpp"
#include "control/position_controller.hpp"
#include "control/tracking_guidance.hpp"
#include "estimation/history.hpp"
#include "estimation/target_filter.hpp"
#include "math/vec3.hpp"
#include "plant/platform.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"
#include "sensors/camera.hpp"

using core::GuidanceOutput;
using core::TargetEstimate;
using core::TrackingGuidance;
using core::Vec3d;

namespace {

TargetEstimate make_est(double x, double y, double vx, double vy, double sigma, double age) {
    TargetEstimate e;
    e.valid = true;
    e.position = Vec3d{x, y, 0.0};
    e.velocity = Vec3d{vx, vy, 0.0};
    e.pos_sigma = sigma;
    e.age_s = age;
    return e;
}

GuidanceOutput run_steps(TrackingGuidance& g, const TargetEstimate& est, const Vec3d& quad,
                         double approach_alt, int steps) {
    GuidanceOutput out;
    for (int i = 0; i < steps; ++i) {
        out = g.update(est, quad, approach_alt, 1e-3);
    }
    return out;
}

void test_lost_target_holds_position() {
    TrackingGuidance g;
    const GuidanceOutput o = g.update(TargetEstimate{}, Vec3d{1.0, 2.0, -1.5}, 0.2, 1e-3);
    CHECK(!o.tracking);
    CHECK(!o.descent_allowed);
    CHECK_NEAR(o.pos_sp.x, 1.0, 1e-15);
    CHECK_NEAR(o.pos_sp.y, 2.0, 1e-15);
    CHECK_NEAR(o.vel_ff.x, 0.0, 1e-15);
    CHECK_NEAR(o.vel_ff.y, 0.0, 1e-15);
    CHECK_NEAR(o.alt_sp, 1.5, 1e-15);
}

void test_stale_estimate_is_treated_as_lost() {
    TrackingGuidance g;
    const GuidanceOutput o =
        g.update(make_est(2.0, 2.0, 0.5, 0.0, 0.02, 0.6), Vec3d{0.0, 0.0, -1.5}, 0.2, 1e-3);
    CHECK(!o.tracking);
    CHECK_NEAR(o.pos_sp.x, 0.0, 1e-15);
    CHECK_NEAR(o.vel_ff.x, 0.0, 1e-15);
    CHECK_NEAR(o.alt_sp, 1.5, 1e-15);
}

void test_estimate_passes_through() {
    TrackingGuidance g;
    const GuidanceOutput o =
        g.update(make_est(2.0, -1.0, 0.5, 0.25, 0.02, 0.0), Vec3d{0.0, 0.0, -1.5}, 1.5, 1e-3);
    CHECK(o.tracking);
    CHECK(o.descent_allowed);
    CHECK_NEAR(o.pos_sp.x, 2.0, 1e-15);
    CHECK_NEAR(o.pos_sp.y, -1.0, 1e-15);
    CHECK_NEAR(o.vel_ff.x, 0.5, 1e-15);
    CHECK_NEAR(o.vel_ff.y, 0.25, 1e-15);
    CHECK_NEAR(o.horiz_error, std::sqrt(5.0), 1e-12);
    CHECK_NEAR(o.alt_sp, 1.5, 1e-15);
}

void test_funnel_holds_altitude_when_far() {
    TrackingGuidance g;
    const GuidanceOutput o = run_steps(g, make_est(1.0, 0.0, 0.0, 0.0, 0.02, 0.0),
                                       Vec3d{0.0, 0.0, -1.5}, 0.2, 3000);
    CHECK_NEAR(o.alt_sp, 0.5 * (1.0 - 0.15), 1e-12);
}

void test_funnel_opens_when_aligned() {
    TrackingGuidance g;
    const GuidanceOutput o = run_steps(g, make_est(0.05, 0.0, 0.0, 0.0, 0.02, 0.0),
                                       Vec3d{0.0, 0.0, -1.5}, 0.2, 3000);
    CHECK_NEAR(o.alt_sp, 0.2, 1e-12);
}

void test_descent_rate_limit() {
    TrackingGuidance g;
    const GuidanceOutput o = run_steps(g, make_est(0.0, 0.0, 0.0, 0.0, 0.02, 0.0),
                                       Vec3d{0.0, 0.0, -1.5}, 0.2, 1000);
    CHECK_NEAR(o.alt_sp, 1.5 - 0.5 * 1.0, 1e-9);
}

void test_climbs_immediately_when_error_grows() {
    TrackingGuidance g;
    const GuidanceOutput o =
        g.update(make_est(3.0, 0.0, 0.0, 0.0, 0.02, 0.0), Vec3d{0.0, 0.0, -1.0}, 1.0, 1e-3);
    CHECK_NEAR(o.alt_sp, 0.5 * (3.0 - 0.15), 1e-12);
}

void test_altitude_cap() {
    TrackingGuidance g;
    const GuidanceOutput o =
        g.update(make_est(10.0, 0.0, 0.0, 0.0, 0.02, 0.0), Vec3d{0.0, 0.0, -1.0}, 1.0, 1e-3);
    CHECK_NEAR(o.alt_sp, 3.0, 1e-12);
}

void test_uncertain_estimate_blocks_descent() {
    TrackingGuidance g;
    const GuidanceOutput o = run_steps(g, make_est(0.05, 0.0, 0.0, 0.0, 0.5, 0.0),
                                       Vec3d{0.0, 0.0, -1.5}, 0.2, 3000);
    CHECK(o.tracking);
    CHECK(!o.descent_allowed);
    CHECK_NEAR(o.alt_sp, 1.5, 1e-12);
}

void test_reset_reinitializes_from_current_altitude() {
    TrackingGuidance g;
    run_steps(g, make_est(0.0, 0.0, 0.0, 0.0, 0.02, 0.0), Vec3d{0.0, 0.0, -1.5}, 0.2, 500);
    g.reset();
    const GuidanceOutput o = g.update(TargetEstimate{}, Vec3d{0.0, 0.0, -2.0}, 0.2, 1e-3);
    CHECK_NEAR(o.alt_sp, 2.0, 1e-15);
}

class Pipeline {
public:
    explicit Pipeline(const sim::CameraConfig& cam_cfg)
        : quad_(params_),
          altitude_(alt_config(params_)),
          camera_(cam_cfg),
          cam_cfg_(cam_cfg) {
        platform_.position = Vec3d{3.0, 0.0, 0.0};
        platform_.velocity = Vec3d{0.5, 0.0, 0.0};
        sim::State s;
        s.position.z = -1.0;
        quad_.reset(s, std::sqrt(params_.mass * params_.gravity / (4.0 * params_.kThrust)));
    }

    void step(double approach_alt) {
        const double dt = 1e-3;
        const sim::State& s = quad_.state();
        const double altitude_m = -s.position.z;

        history_.push(t_, s.position);
        camera_.capture(t_,
                        Vec3d{platform_.position.x - s.position.x,
                              platform_.position.y - s.position.y, 0.0},
                        altitude_m);
        sim::CameraMeasurement m;
        while (camera_.poll(t_, m)) {
            Vec3d then;
            if (history_.at(m.capture_time, then)) {
                const double sigma =
                    cam_cfg_.sigma_base_m + cam_cfg_.sigma_per_m * std::max(altitude_m, 0.0);
                filter_.update(m.capture_time,
                               Vec3d{then.x + m.rel_pos.x, then.y + m.rel_pos.y, 0.0}, sigma);
            }
        }

        const TargetEstimate est = filter_.estimate(t_);
        last_ = guidance_.update(est, s.position, approach_alt, dt);
        const double thrust = altitude_.update(last_.alt_sp, s.position, s.velocity, s.attitude, dt);
        const Vec3d tilt =
            position_.update(last_.pos_sp, last_.vel_ff, s.position, s.velocity, s.attitude, dt);
        const Vec3d torque = attitude_.update(s.attitude, s.omega, Vec3d{tilt.x, tilt.y, 0.0}, dt);

        quad_.step(thrust, torque, dt);
        platform_.step(dt);
        t_ += dt;
    }

    const sim::State& state() const { return quad_.state(); }
    const sim::Platform& platform() const { return platform_; }
    const GuidanceOutput& last() const { return last_; }

    double altitude() const { return -quad_.state().position.z; }
    double horiz_error() const {
        return std::hypot(platform_.position.x - quad_.state().position.x,
                          platform_.position.y - quad_.state().position.y);
    }

private:
    static core::AltitudeConfig alt_config(const QuadParams& p) {
        core::AltitudeConfig cfg;
        cfg.mass = p.mass;
        cfg.gravity = p.gravity;
        cfg.max_thrust = 4.0 * p.kThrust * p.omegaMax * p.omegaMax;
        return cfg;
    }

    QuadParams params_;
    sim::Quad quad_;
    core::AttitudeController attitude_;
    core::PositionController position_;
    core::AltitudeController altitude_;
    sim::Platform platform_;
    sim::Camera camera_;
    sim::CameraConfig cam_cfg_;
    core::TargetFilter filter_;
    core::PositionHistory<256> history_;
    TrackingGuidance guidance_;
    GuidanceOutput last_;
    double t_ = 0.0;
};

void test_tracks_and_holds_above_platform() {
    sim::CameraConfig cam;
    cam.seed = 3;
    Pipeline p(cam);
    double worst = 0.0;
    for (int i = 0; i < 20000; ++i) {
        p.step(1.5);
        if (i >= 18000) {
            worst = std::max(worst, p.horiz_error());
        }
    }
    CHECK(worst < 0.2);
    CHECK_NEAR(p.altitude(), 1.5, 0.1);
    CHECK(p.platform().contains_xy(p.state().position));
    CHECK(p.last().tracking);
}

void test_descends_when_aligned() {
    sim::CameraConfig cam;
    cam.seed = 3;
    Pipeline p(cam);
    double lowest = 1e9;
    for (int i = 0; i < 25000; ++i) {
        p.step(i < 10000 ? 1.5 : 0.3);
        if (i >= 10000) {
            lowest = std::min(lowest, p.altitude());
        }
    }
    CHECK_NEAR(p.altitude(), 0.3, 0.05);
    CHECK(lowest > 0.15);
    CHECK(p.horiz_error() < 0.2);
    CHECK(p.platform().contains_xy(p.state().position));
}

void test_holds_when_blind() {
    sim::CameraConfig cam;
    cam.dropout_prob = 1.0;
    Pipeline p(cam);
    for (int i = 0; i < 5000; ++i) {
        p.step(0.3);
    }
    CHECK(!p.last().tracking);
    CHECK_NEAR(p.altitude(), 1.0, 0.02);
    CHECK_NEAR(p.state().position.x, 0.0, 0.02);
    CHECK_NEAR(p.state().position.y, 0.0, 0.02);
}

}

int main() {
    test_lost_target_holds_position();
    test_stale_estimate_is_treated_as_lost();
    test_estimate_passes_through();
    test_funnel_holds_altitude_when_far();
    test_funnel_opens_when_aligned();
    test_descent_rate_limit();
    test_climbs_immediately_when_error_grows();
    test_altitude_cap();
    test_uncertain_estimate_blocks_descent();
    test_reset_reinitializes_from_current_altitude();
    test_tracks_and_holds_above_platform();
    test_descends_when_aligned();
    test_holds_when_blind();
    return check::report();
}