#pragma once

#include <algorithm>

#include "control/pid.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"

namespace core {

struct AltitudeConfig {
    double mass = 1.0;
    double gravity = 9.81;
    double max_thrust = 32.0;
    double alt_kp = 1.5;
    double max_climb = 3.0;
    double max_descent = 2.0;
    double vel_kp = 5.0;
    double vel_ki = 0.0;
    double max_accel_up = 8.0;
    double max_accel_down = 6.0;
    double min_tilt_cos = 0.5;
};

class AltitudeController {
public:
    explicit AltitudeController(const AltitudeConfig& config = AltitudeConfig{})
        : cfg_(config), vel_(make_pid(config)) {}

    void reset() {
        vel_.reset();
        climb_sp_ = 0.0;
    }

    double update(double altitude_sp, const Vec3d& position, const Vec3d& velocity,
                  const Quatd& attitude, double dt) {
        const double altitude = -position.z;
        const double climb = -velocity.z;

        climb_sp_ = std::clamp(cfg_.alt_kp * (altitude_sp - altitude), -cfg_.max_descent,
                               cfg_.max_climb);
        const double accel = vel_.update(climb_sp_, climb, dt);

        const Vec3d thrust_dir = rotate(attitude, Vec3d{0.0, 0.0, -1.0});
        const double tilt_cos = std::max(-thrust_dir.z, cfg_.min_tilt_cos);

        const double thrust = cfg_.mass * (cfg_.gravity + accel) / tilt_cos;
        return std::clamp(thrust, 0.0, cfg_.max_thrust);
    }

    double climb_rate_setpoint() const { return climb_sp_; }
    const AltitudeConfig& config() const { return cfg_; }

private:
    static Pid make_pid(const AltitudeConfig& c) {
        PidConfig p;
        p.kp = c.vel_kp;
        p.ki = c.vel_ki;
        p.out_min = -c.max_accel_down;
        p.out_max = c.max_accel_up;
        return Pid(p);
    }

    AltitudeConfig cfg_;
    Pid vel_;
    double climb_sp_ = 0.0;
};

}