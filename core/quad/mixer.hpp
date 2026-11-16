#pragma once

#include <array>

#include "quad_params.hpp"

struct Wrench {
    double thrust = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
};

class Mixer {
public:
    explicit Mixer(const QuadParams& params);

    std::array<double, 4> thrusts(const Wrench& w) const;
    std::array<double, 4> omegaCommands(const Wrench& w) const;
    Wrench wrench(const std::array<double, 4>& motorThrusts) const;

private:
    double lever_;
    double yawRatio_;
    double kThrust_;
    double maxThrust_;
};