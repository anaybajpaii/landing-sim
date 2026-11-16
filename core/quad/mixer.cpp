#include "mixer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr std::array<double, 4> kRollSign{-1.0, 1.0, 1.0, -1.0};
constexpr std::array<double, 4> kPitchSign{1.0, 1.0, -1.0, -1.0};
constexpr std::array<double, 4> kYawSign{1.0, -1.0, 1.0, -1.0};

}

Mixer::Mixer(const QuadParams& params)
    : lever_(params.armLength / std::sqrt(2.0)),
      yawRatio_(params.kTorque / params.kThrust),
      kThrust_(params.kThrust),
      maxThrust_(params.kThrust * params.omegaMax * params.omegaMax) {}

std::array<double, 4> Mixer::thrusts(const MixerWrench& w) const {
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double raw = 0.25 * (w.thrust_n + kRollSign[i] * w.torque_nm.x / lever_ +
                                   kPitchSign[i] * w.torque_nm.y / lever_ +
                                   kYawSign[i] * w.torque_nm.z / yawRatio_);
        out[i] = std::clamp(raw, 0.0, maxThrust_);
    }
    return out;
}

std::array<double, 4> Mixer::omegaCommands(const MixerWrench& w) const {
    const std::array<double, 4> t = thrusts(w);
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        out[i] = std::sqrt(t[i] / kThrust_);
    }
    return out;
}

MixerWrench Mixer::wrench(const std::array<double, 4>& motorThrusts) const {
    MixerWrench w;
    for (std::size_t i = 0; i < 4; ++i) {
        w.thrust_n += motorThrusts[i];
        w.torque_nm.x += kRollSign[i] * motorThrusts[i];
        w.torque_nm.y += kPitchSign[i] * motorThrusts[i];
        w.torque_nm.z += kYawSign[i] * motorThrusts[i];
    }
    w.torque_nm.x *= lever_;
    w.torque_nm.y *= lever_;
    w.torque_nm.z *= yawRatio_;
    return w;
}