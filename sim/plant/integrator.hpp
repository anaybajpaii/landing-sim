#pragma once

#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/rigid_body.hpp"
#include "plant/vehicle_params.hpp"

namespace sim {

inline State add_scaled(
    const State& s,
    const StateDerivative& d,
    double h)
{
    State result;

    result.position = s.position + d.d_position * h;
    result.velocity = s.velocity + d.d_velocity * h;

    result.attitude = core::Quatd{
        s.attitude.w + h * d.d_attitude.w,
        s.attitude.x + h * d.d_attitude.x,
        s.attitude.y + h * d.d_attitude.y,
        s.attitude.z + h * d.d_attitude.z
    };

    result.omega = s.omega + d.d_omega * h;

    return result;
}

inline State rk4_step(
    const State& s,
    const Wrench& w,
    const VehicleParams& p,
    double dt)
{
    const StateDerivative k1 = derivative(s, w, p);

    const State s2 = add_scaled(s, k1, dt / 2.0);
    const StateDerivative k2 = derivative(s2, w, p);

    const State s3 = add_scaled(s, k2, dt / 2.0);
    const StateDerivative k3 = derivative(s3, w, p);

    const State s4 = add_scaled(s, k3, dt);
    const StateDerivative k4 = derivative(s4, w, p);

    const double scale = dt / 6.0;

    State result;

    result.position = s.position +
        (k1.d_position +
         k2.d_position * 2.0 +
         k3.d_position * 2.0 +
         k4.d_position) * scale;

    result.velocity = s.velocity +
        (k1.d_velocity +
         k2.d_velocity * 2.0 +
         k3.d_velocity * 2.0 +
         k4.d_velocity) * scale;

    result.omega = s.omega +
        (k1.d_omega +
         k2.d_omega * 2.0 +
         k3.d_omega * 2.0 +
         k4.d_omega) * scale;

    const core::Quatd attitude{
        s.attitude.w + scale * (
            k1.d_attitude.w +
            2.0 * k2.d_attitude.w +
            2.0 * k3.d_attitude.w +
            k4.d_attitude.w
        ),
        s.attitude.x + scale * (
            k1.d_attitude.x +
            2.0 * k2.d_attitude.x +
            2.0 * k3.d_attitude.x +
            k4.d_attitude.x
        ),
        s.attitude.y + scale * (
            k1.d_attitude.y +
            2.0 * k2.d_attitude.y +
            2.0 * k3.d_attitude.y +
            k4.d_attitude.y
        ),
        s.attitude.z + scale * (
            k1.d_attitude.z +
            2.0 * k2.d_attitude.z +
            2.0 * k3.d_attitude.z +
            k4.d_attitude.z
        )
    };

    result.attitude = core::normalized(attitude);

    return result;
}

}