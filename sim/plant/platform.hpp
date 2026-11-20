#pragma once

#include <cmath>

#include "math/vec3.hpp"

namespace sim {

struct Platform {
    core::Vec3d position{};
    core::Vec3d velocity{};
    double half_size = 0.5;

    void step(double dt) {
        position += velocity * dt;
    }

    bool contains_xy(const core::Vec3d& p) const {
        return std::fabs(p.x - position.x) <= half_size &&
               std::fabs(p.y - position.y) <= half_size;
    }
};

}