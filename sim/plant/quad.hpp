#pragma once

#include <array>
#include <cstddef>

#include "math/vec3.hpp"
#include "plant/integrator.hpp"
#include "plant/rigid_body.hpp"
#include "plant/vehicle_params.hpp"
#include "quad/mixer.hpp"
#include "quad/motor_model.hpp"
#include "quad/quad_params.hpp"

namespace sim {

class Quad {
public:
    explicit Quad(const QuadParams& params) : motors_(params), mixer_(params) {
        vehicle_.mass_kg = params.mass;
        vehicle_.inertia_kgm2 = core::Vec3d{params.ixx, params.iyy, params.izz};
        vehicle_.gravity_mps2 = params.gravity;
    }

    void reset(const State& initial, double motorOmega = 0.0) {
        state_ = initial;
        motors_.reset(motorOmega);
        wrench_ = Wrench{};
    }

    void step(double thrustN, const core::Vec3d& torqueNm, double dt) {
        MixerWrench demand;
        demand.thrust_n = thrustN;
        demand.torque_nm = torqueNm;
        motors_.step(mixer_.omegaCommands(demand), dt);

        std::array<double, 4> motorThrust{};
        for (std::size_t i = 0; i < 4; ++i) {
            motorThrust[i] = motors_.thrust(i);
        }
        const MixerWrench actual = mixer_.wrench(motorThrust);
        wrench_.thrust_n = actual.thrust_n;
        wrench_.torque_nm = actual.torque_nm;

        state_ = rk4_step(state_, wrench_, vehicle_, dt);
    }

    const State& state() const { return state_; }
    const Wrench& wrench() const { return wrench_; }
    const MotorModel& motors() const { return motors_; }

private:
    VehicleParams vehicle_;
    MotorModel motors_;
    Mixer mixer_;
    State state_;
    Wrench wrench_;
};

}