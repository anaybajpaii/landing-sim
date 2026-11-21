#pragma once

#include <algorithm>
#include <cmath>

#include "math/vec3.hpp"

namespace core {

struct TargetFilterConfig {
    double accel_sigma = 0.1;
    double initial_vel_sigma = 1.0;
};

struct TargetEstimate {
    bool valid = false;
    Vec3d position{};
    Vec3d velocity{};
    double pos_sigma = 0.0;
    double age_s = 0.0;
};

class TargetFilter {
public:
    explicit TargetFilter(const TargetFilterConfig& config = TargetFilterConfig{})
        : cfg_(config) {}

    void reset() {
        initialized_ = false;
        state_time_ = 0.0;
        x_ = Axis{};
        y_ = Axis{};
    }

    bool initialized() const { return initialized_; }

    bool update(double t_capture, const Vec3d& world_pos, double sigma) {
        const double r = std::max(sigma * sigma, 1e-12);
        if (!initialized_) {
            x_ = make_axis(world_pos.x, r);
            y_ = make_axis(world_pos.y, r);
            state_time_ = t_capture;
            initialized_ = true;
            return true;
        }
        const double dt = t_capture - state_time_;
        if (dt < 0.0) {
            return false;
        }
        const double q = cfg_.accel_sigma * cfg_.accel_sigma;
        predict(x_, dt, q);
        predict(y_, dt, q);
        correct(x_, world_pos.x, r);
        correct(y_, world_pos.y, r);
        state_time_ = t_capture;
        return true;
    }

    TargetEstimate estimate(double t_now) const {
        TargetEstimate out;
        if (!initialized_) {
            return out;
        }
        const double dt = std::max(0.0, t_now - state_time_);
        const double q = cfg_.accel_sigma * cfg_.accel_sigma;
        Axis ax = x_;
        Axis ay = y_;
        predict(ax, dt, q);
        predict(ay, dt, q);
        out.valid = true;
        out.position = Vec3d{ax.p, ay.p, 0.0};
        out.velocity = Vec3d{ax.v, ay.v, 0.0};
        out.pos_sigma = std::sqrt(0.5 * (ax.p00 + ay.p00));
        out.age_s = dt;
        return out;
    }

    const TargetFilterConfig& config() const { return cfg_; }

private:
    struct Axis {
        double p = 0.0;
        double v = 0.0;
        double p00 = 0.0;
        double p01 = 0.0;
        double p11 = 0.0;
    };

    Axis make_axis(double z, double r) const {
        Axis a;
        a.p = z;
        a.v = 0.0;
        a.p00 = r;
        a.p01 = 0.0;
        a.p11 = cfg_.initial_vel_sigma * cfg_.initial_vel_sigma;
        return a;
    }

    static void predict(Axis& a, double dt, double q) {
        const double dt2 = dt * dt;
        a.p += a.v * dt;
        a.p00 += 2.0 * dt * a.p01 + dt2 * a.p11 + 0.25 * q * dt2 * dt2;
        a.p01 += dt * a.p11 + 0.5 * q * dt2 * dt;
        a.p11 += q * dt2;
    }

    static void correct(Axis& a, double z, double r) {
        const double s = a.p00 + r;
        const double k0 = a.p00 / s;
        const double k1 = a.p01 / s;
        const double innovation = z - a.p;
        a.p += k0 * innovation;
        a.v += k1 * innovation;
        const double m = 1.0 - k0;
        const double n00 = m * m * a.p00 + r * k0 * k0;
        const double n01 = m * (a.p01 - k1 * a.p00) + r * k0 * k1;
        const double n11 = a.p11 - 2.0 * k1 * a.p01 + k1 * k1 * a.p00 + r * k1 * k1;
        a.p00 = n00;
        a.p01 = n01;
        a.p11 = n11;
    }

    TargetFilterConfig cfg_;
    bool initialized_ = false;
    double state_time_ = 0.0;
    Axis x_;
    Axis y_;
};

}