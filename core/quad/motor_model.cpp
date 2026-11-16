#include "motor_model.hpp"

#include <algorithm>
#include <cmath>

MotorModel::MotorModel(const QuadParams& params) : p_(params), omega_{0.0, 0.0, 0.0, 0.0} {}

void MotorModel::reset(double value) {
    omega_.fill(std::clamp(value, 0.0, p_.omegaMax));
}

void MotorModel::step(const std::array<double, 4>& omegaCmd, double dt) {
    const double alpha = (p_.motorTau > 0.0) ? 1.0 - std::exp(-dt / p_.motorTau) : 1.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const double target = std::clamp(omegaCmd[i], 0.0, p_.omegaMax);
        omega_[i] += alpha * (target - omega_[i]);
    }
}

double MotorModel::omega(std::size_t i) const {
    return omega_[i];
}

double MotorModel::thrust(std::size_t i) const {
    return p_.kThrust * omega_[i] * omega_[i];
}

double MotorModel::torque(std::size_t i) const {
    return p_.kTorque * omega_[i] * omega_[i];
}

const std::array<double, 4>& MotorModel::omegas() const {
    return omega_;
}