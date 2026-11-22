#pragma once

#include <algorithm>
#include <cmath>

#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/platform.hpp"
#include "plant/rigid_body.hpp"

namespace sim {

struct TouchdownCriteria {
    double contact_height = 0.05;
    double max_vertical_speed = 0.5;
    double max_rel_speed = 0.3;
    double max_tilt = 0.26;

    bool in_contact(double altitude) const { return altitude <= contact_height; }
};

struct TouchdownResult {
    bool touched = false;
    bool success = false;
    bool on_platform = false;
    bool vertical_ok = false;
    bool rel_speed_ok = false;
    bool tilt_ok = false;
    double time = 0.0;
    double vertical_speed = 0.0;
    double rel_speed = 0.0;
    double tilt = 0.0;
    double offset = 0.0;
};

inline TouchdownResult evaluate_touchdown(const TouchdownCriteria& c, double t, const State& quad,
                                          const Platform& platform) {
    TouchdownResult r;
    r.touched = true;
    r.time = t;
    r.vertical_speed = quad.velocity.z;
    r.rel_speed = std::hypot(quad.velocity.x - platform.velocity.x,
                             quad.velocity.y - platform.velocity.y);
    const core::Vec3d thrust_dir = core::rotate(quad.attitude, core::Vec3d{0.0, 0.0, -1.0});
    r.tilt = std::acos(std::clamp(-thrust_dir.z, -1.0, 1.0));
    r.offset = std::hypot(quad.position.x - platform.position.x,
                          quad.position.y - platform.position.y);
    r.on_platform = platform.contains_xy(quad.position);
    r.vertical_ok = r.vertical_speed <= c.max_vertical_speed;
    r.rel_speed_ok = r.rel_speed <= c.max_rel_speed;
    r.tilt_ok = r.tilt <= c.max_tilt;
    r.success = r.on_platform && r.vertical_ok && r.rel_speed_ok && r.tilt_ok;
    return r;
}
}