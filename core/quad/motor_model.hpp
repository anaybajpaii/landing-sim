#pragma once

#include <array>
#include <cstddef>

#include "quad_params.hpp"

class MotorModel {
public:
    explicit MotorModel(const QuadParams& params);

    void reset(double value = 0.0);
    void step(const std::array<double, 4>& omegaCmd, double dt);

    double omega(std::size_t i) const;
    double thrust(std::size_t i) const;
    double torque(std::size_t i) const;
    const std::array<double, 4>& omegas() const;

private:
    QuadParams p_;
    std::array<double, 4> omega_;
};