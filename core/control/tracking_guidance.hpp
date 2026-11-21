#pragma once

#include <algorithm>
#include <cmath>

#include "estimation/target_filter.hpp"
#include "math/vec3.hpp"

namespace core {

struct TrackingConfig {
    double funnel_slope = 0.5;
    double funnel_deadband = 0.15;
    double max_alt = 3.0;
    double max_descent_rate = 0.5;
    double max_age_s = 0.5;
    double max_sigma = 0.1;
};

struct GuidanceOutput {
    Vec3d pos_sp{};
    Vec3d vel_ff{};
    double alt_sp = 0.0;
    double horiz_error = 0.0;
    bool tracking = false;
    bool descent_allowed = false;
};

class TrackingGuidance {
public:
    explicit TrackingGuidance(const TrackingConfig& config = TrackingConfig{})
        : cfg_(config) {}

    void reset() {
        initialized_ = false;
        alt_sp_ = 0.0;
    }

    GuidanceOutput update(const TargetEstimate& est, const Vec3d& quad_pos, double approach_alt,
                          double dt) {
        const double altitude = -quad_pos.z;
        if (!initialized_) {
            alt_sp_ = altitude;
            initialized_ = true;
        }

        GuidanceOutput out;
        out.alt_sp = alt_sp_;

        const bool fresh = est.valid && est.age_s <= cfg_.max_age_s;
        if (!fresh) {
            out.pos_sp = Vec3d{quad_pos.x, quad_pos.y, 0.0};
            return out;
        }

        out.tracking = true;
        out.pos_sp = Vec3d{est.position.x, est.position.y, 0.0};
        out.vel_ff = Vec3d{est.velocity.x, est.velocity.y, 0.0};
        out.horiz_error = std::hypot(est.position.x - quad_pos.x, est.position.y - quad_pos.y);

        const double funnel =
            cfg_.funnel_slope * std::max(0.0, out.horiz_error - cfg_.funnel_deadband);
        double desired = std::min(std::max(approach_alt, funnel), cfg_.max_alt);

        out.descent_allowed = est.pos_sigma <= cfg_.max_sigma;
        if (!out.descent_allowed) {
            desired = std::max(desired, alt_sp_);
        }

        if (desired >= alt_sp_) {
            alt_sp_ = desired;
        } else {
            alt_sp_ = std::max(desired, alt_sp_ - cfg_.max_descent_rate * std::max(dt, 0.0));
        }
        out.alt_sp = alt_sp_;
        return out;
    }

    const TrackingConfig& config() const { return cfg_; }

private:
    TrackingConfig cfg_;
    bool initialized_ = false;
    double alt_sp_ = 0.0;
};

}