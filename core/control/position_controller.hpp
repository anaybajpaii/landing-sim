#pragma once

#include <algorithm>
#include <cmath>

#include "control/pid.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"

namespace core {

struct PositionConfig {
    double gravity = 9.81;
    double pos_kp = 0.8;
    double max_speed = 3.0;
    double vel_kp = 2.5;
    double vel_ki = 0.0;
    double max_accel = 3.0;
    double max_tilt = 0.5;
};

class PositionController {
public:
    explicit PositionController(const PositionConfig& config = PositionConfig{})
        : cfg_(config), vel_x_(make_pid(config)), vel_y_(make_pid(config)) {}

    void reset() {
        vel_x_.reset();
        vel_y_.reset();
        accel_cmd_ = Vec3d{};
    }

    Vec3d update(const Vec3d& pos_sp, const Vec3d& vel_ff, const Vec3d& position,
                 const Vec3d& velocity, const Quatd& attitude, double dt) {
        double vsx = cfg_.pos_kp * (pos_sp.x - position.x) + vel_ff.x;
        double vsy = cfg_.pos_kp * (pos_sp.y - position.y) + vel_ff.y;
        limit_norm(vsx, vsy, cfg_.max_speed);

        double ax = vel_x_.update(vsx, velocity.x, dt);
        double ay = vel_y_.update(vsy, velocity.y, dt);
        limit_norm(ax, ay, cfg_.max_accel);
        accel_cmd_ = Vec3d{ax, ay, 0.0};

        const double yaw = euler_zyx(attitude).z;
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double ax_b = c * ax + s * ay;
        const double ay_b = -s * ax + c * ay;

        const double pitch = std::atan(-ax_b / cfg_.gravity);
        const double roll = std::atan(std::cos(pitch) * ay_b / cfg_.gravity);

        return Vec3d{std::clamp(roll, -cfg_.max_tilt, cfg_.max_tilt),
                     std::clamp(pitch, -cfg_.max_tilt, cfg_.max_tilt), 0.0};
    }

    const Vec3d& accel_command() const { return accel_cmd_; }
    const PositionConfig& config() const { return cfg_; }

private:
    static Pid make_pid(const PositionConfig& c) {
        PidConfig p;
        p.kp = c.vel_kp;
        p.ki = c.vel_ki;
        p.out_min = -c.max_accel;
        p.out_max = c.max_accel;
        return Pid(p);
    }

    static void limit_norm(double& x, double& y, double limit) {
        const double n = std::hypot(x, y);
        if (n > limit && n > 0.0) {
            const double k = limit / n;
            x *= k;
            y *= k;
        }
    }

    PositionConfig cfg_;
    Pid vel_x_;
    Pid vel_y_;
    Vec3d accel_cmd_{};
};

}