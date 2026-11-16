#pragma once

#include <array>

#include "math/vec3.hpp"
#include "quad_params.hpp"

struct MixerWrench {
    double thrust_n = 0.0;
    core::Vec3d torque_nm{};
};

class Mixer {
public:
    explicit Mixer(const QuadParams& params);

    std::array<double, 4> thrusts(const MixerWrench& w) const;
    std::array<double, 4> omegaCommands(const MixerWrench& w) const;
    MixerWrench wrench(const std::array<double, 4>& motorThrusts) const;

private:
    double lever_;
    double yawRatio_;
    double kThrust_;
    double maxThrust_;
};