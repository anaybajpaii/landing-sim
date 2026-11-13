#pragma once

#include "math/vec3.hpp"

namespace sim {

struct VehicleParams {
    double mass_kg = 1.0;
    core::Vec3d inertia_kgm2{0.010, 0.010, 0.018};
    double arm_length_m = 0.25;
    double yaw_torque_per_thrust_m = 0.016;
    double max_motor_thrust_n = 8.0;
    double gravity_mps2 = 9.81;
};

}