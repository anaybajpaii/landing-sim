#pragma once

#include <algorithm>
#include <cmath>

#include "control/altitude_controller.hpp"
#include "control/attitude_controller.hpp"
#include "control/position_controller.hpp"
#include "control/tracking_guidance.hpp"
#include "estimation/history.hpp"
#include "estimation/target_filter.hpp"
#include "guidance/landing_state_machine.hpp"
#include "math/vec3.hpp"
#include "plant/platform.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"
#include "sensors/camera.hpp"

namespace sim {

struct PipelineConfig {
    CameraConfig camera;
    core::LandingConfig landing;
    core::TrackingConfig tracking;
    core::Vec3d platform_position{3.0, 0.0, 0.0};
    core::Vec3d platform_velocity{0.5, 0.0, 0.0};
    double start_altitude = 1.0;
};

class LandingPipeline {
public:
    static constexpr double kDt = 1e-3;

    explicit LandingPipeline(const PipelineConfig& config = PipelineConfig{})
        : cfg_(config),
          quad_(params_),
          altitude_(make_alt_config(params_)),
          camera_(config.camera),
          guidance_(config.tracking),
          machine_(config.landing) {
        platform_.position = config.platform_position;
        platform_.velocity = config.platform_velocity;
        State s;
        s.position.z = -config.start_altitude;
        quad_.reset(s, std::sqrt(params_.mass * params_.gravity / (4.0 * params_.kThrust)));
    }

    void set_blind(bool blind) { blind_ = blind; }

    void step() {
        const State& s = quad_.state();
        const double altitude_m = -s.position.z;

        history_.push(t_, s.position);
        camera_.capture(t_,
                        core::Vec3d{platform_.position.x - s.position.x,
                                    platform_.position.y - s.position.y, 0.0},
                        altitude_m);
        CameraMeasurement m;
        while (camera_.poll(t_, m)) {
            if (blind_) {
                continue;
            }
            core::Vec3d then;
            if (history_.at(m.capture_time, then)) {
                const double sigma =
                    cfg_.camera.sigma_base_m + cfg_.camera.sigma_per_m * std::max(altitude_m, 0.0);
                filter_.update(m.capture_time,
                               core::Vec3d{then.x + m.rel_pos.x, then.y + m.rel_pos.y, 0.0},
                               sigma);
            }
        }

        est_ = filter_.estimate(t_);
        out_ = machine_.update(est_, s.position, s.velocity, kDt);
        guide_ = guidance_.update(est_, s.position, out_.approach_alt, kDt);

        const double thrust =
            altitude_.update(guide_.alt_sp, s.position, s.velocity, s.attitude, kDt);
        const core::Vec3d tilt =
            position_.update(guide_.pos_sp, guide_.vel_ff, s.position, s.velocity, s.attitude, kDt);
        const core::Vec3d torque =
            attitude_.update(s.attitude, s.omega, core::Vec3d{tilt.x, tilt.y, 0.0}, kDt);

        quad_.step(thrust, torque, kDt);
        platform_.step(kDt);
        t_ += kDt;
    }

    double time() const { return t_; }
    const State& state() const { return quad_.state(); }
    const Platform& platform() const { return platform_; }
    const core::LandingOutput& last_output() const { return out_; }
    const core::GuidanceOutput& last_guidance() const { return guide_; }
    const core::TargetEstimate& estimate() const { return est_; }

    double altitude() const { return -quad_.state().position.z; }

    double true_horiz_error() const {
        return std::hypot(platform_.position.x - quad_.state().position.x,
                          platform_.position.y - quad_.state().position.y);
    }

    double true_rel_speed() const {
        return std::hypot(quad_.state().velocity.x - platform_.velocity.x,
                          quad_.state().velocity.y - platform_.velocity.y);
    }

private:
    static core::AltitudeConfig make_alt_config(const QuadParams& p) {
        core::AltitudeConfig cfg;
        cfg.mass = p.mass;
        cfg.gravity = p.gravity;
        cfg.max_thrust = 4.0 * p.kThrust * p.omegaMax * p.omegaMax;
        return cfg;
    }

    PipelineConfig cfg_;
    QuadParams params_;
    Quad quad_;
    core::AttitudeController attitude_;
    core::PositionController position_;
    core::AltitudeController altitude_;
    Platform platform_;
    Camera camera_;
    core::TargetFilter filter_;
    core::PositionHistory<256> history_;
    core::TrackingGuidance guidance_;
    core::LandingStateMachine machine_;
    core::TargetEstimate est_;
    core::LandingOutput out_;
    core::GuidanceOutput guide_;
    double t_ = 0.0;
    bool blind_ = false;
};

}