#pragma once

#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/vehicle_params.hpp"

namespace sim {

struct State {
    core::Vec3d position{};
    core::Vec3d velocity{};
    core::Quatd attitude{};
    core::Vec3d omega{};
};

struct Wrench {
    double thrust_n = 0.0;
    core::Vec3d torque_nm{};
};

struct StateDerivative {
    core::Vec3d d_position{};
    core::Vec3d d_velocity{};
    core::Quatd d_attitude{0.0, 0.0, 0.0, 0.0};
    core::Vec3d d_omega{};
};

inline StateDerivative derivative(
    const State& s,
    const Wrench& w,
    const VehicleParams& p)
{
    StateDerivative d;

    d.d_position = s.velocity;

    const core::Vec3d thrust_body{0.0, 0.0, -w.thrust_n};
    const core::Vec3d thrust_world =
        core::rotate(s.attitude, thrust_body);

    const core::Vec3d gravity{0.0, 0.0, p.gravity_mps2};

    d.d_velocity =
        gravity + thrust_world * (1.0 / p.mass_kg);

    const core::Quatd omega_quat{
        0.0, s.omega.x, s.omega.y, s.omega.z
    };

    const core::Quatd qdot = s.attitude * omega_quat;

    d.d_attitude = core::Quatd{
        0.5 * qdot.w,
        0.5 * qdot.x,
        0.5 * qdot.y,
        0.5 * qdot.z
    };

    const core::Vec3d angular_momentum{
        p.inertia_kgm2.x * s.omega.x,
        p.inertia_kgm2.y * s.omega.y,
        p.inertia_kgm2.z * s.omega.z
    };

    const core::Vec3d gyroscopic =
        core::cross(s.omega, angular_momentum);

    d.d_omega = core::Vec3d{
        (w.torque_nm.x - gyroscopic.x) / p.inertia_kgm2.x,
        (w.torque_nm.y - gyroscopic.y) / p.inertia_kgm2.y,
        (w.torque_nm.z - gyroscopic.z) / p.inertia_kgm2.z
    };

    return d;
}

}