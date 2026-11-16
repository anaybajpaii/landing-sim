#pragma once

#include <algorithm>
#include <cmath>

#include "control/pid.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"

namespace core {

struct AttitudeConfig {
    double angle_kp_roll_pitch = 5.0;
    double angle_kp_yaw = 1.5;
    double max_rate_roll_pitch = 6.0;
    double max_rate_yaw = 1.0;
    double rate_kp_roll_pitch = 0.15;
    double rate_ki_roll_pitch = 0.0;
    double rate_kd_roll_pitch = 0.0;
    double rate_kp_yaw = 0.1;
    double rate_ki_yaw = 0.0;
    double rate_kd_yaw = 0.0;
    double max_torque_roll_pitch = 0.5;
    double max_torque_yaw = 0.1;
    double rate_deriv_tau = 0.0;
};

class AttitudeController {
public:
    explicit AttitudeController(const AttitudeConfig& config = AttitudeConfig{})
        : cfg_(config),
          roll_rate_(make_pid(config.rate_kp_roll_pitch, config.rate_ki_roll_pitch,
                              config.rate_kd_roll_pitch, config.max_torque_roll_pitch,
                              config.rate_deriv_tau)),
          pitch_rate_(make_pid(config.rate_kp_roll_pitch, config.rate_ki_roll_pitch,
                               config.rate_kd_roll_pitch, config.max_torque_roll_pitch,
                               config.rate_deriv_tau)),
          yaw_rate_(make_pid(config.rate_kp_yaw, config.rate_ki_yaw, config.rate_kd_yaw,
                             config.max_torque_yaw, config.rate_deriv_tau)) {}

    void reset() {
        roll_rate_.reset();
        pitch_rate_.reset();
        yaw_rate_.reset();
    }

    Vec3d update(const Quatd& attitude, const Vec3d& omega, const Vec3d& euler_setpoint,
                 double dt) {
        const Vec3d e = euler_zyx(attitude);

        const double roll_err = euler_setpoint.x - e.x;
        const double pitch_err = euler_setpoint.y - e.y;
        const double yaw_err = std::remainder(euler_setpoint.z - e.z, kTwoPi);

        const double rate_x = std::clamp(cfg_.angle_kp_roll_pitch * roll_err,
                                         -cfg_.max_rate_roll_pitch, cfg_.max_rate_roll_pitch);
        const double rate_y = std::clamp(cfg_.angle_kp_roll_pitch * pitch_err,
                                         -cfg_.max_rate_roll_pitch, cfg_.max_rate_roll_pitch);
        const double rate_z =
            std::clamp(cfg_.angle_kp_yaw * yaw_err, -cfg_.max_rate_yaw, cfg_.max_rate_yaw);

        return Vec3d{roll_rate_.update(rate_x, omega.x, dt),
                     pitch_rate_.update(rate_y, omega.y, dt),
                     yaw_rate_.update(rate_z, omega.z, dt)};
    }

    const AttitudeConfig& config() const { return cfg_; }

private:
    static constexpr double kTwoPi = 6.283185307179586;

    static Pid make_pid(double kp, double ki, double kd, double max_torque, double tau) {
        PidConfig c;
        c.kp = kp;
        c.ki = ki;
        c.kd = kd;
        c.out_min = -max_torque;
        c.out_max = max_torque;
        c.deriv_tau = tau;
        return Pid(c);
    }

    AttitudeConfig cfg_;
    Pid roll_rate_;
    Pid pitch_rate_;
    Pid yaw_rate_;
};

}