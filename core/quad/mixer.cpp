#include "mixer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr std::array<double, 4> kRollSign{-1.0, 1.0, 1.0, -1.0};
constexpr std::array<double, 4> kPitchSign{-1.0, -1.0, 1.0, 1.0};
constexpr std::array<double, 4> kYawSign{-1.0, 1.0, -1.0, 1.0};

}

Mixer::Mixer(const QuadParams& params)
    : lever_(params.armLength / std::sqrt(2.0)),
      yawRatio_(params.kTorque / params.kThrust),
      kThrust_(params.kThrust),
      maxThrust_(params.kThrust * params.omegaMax * params.omegaMax) {}

std::array<double, 4> Mixer::thrusts(const Wrench& w) const {
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double raw = 0.25 * (w.thrust + kRollSign[i] * w.tx / lever_ +
                                   kPitchSign[i] * w.ty / lever_ +
                                   kYawSign[i] * w.tz / yawRatio_);
        out[i] = std::clamp(raw, 0.0, maxThrust_);
    }
    return out;
}

std::array<double, 4> Mixer::omegaCommands(const Wrench& w) const {
    const std::array<double, 4> t = thrusts(w);
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        out[i] = std::sqrt(t[i] / kThrust_);
    }
    return out;
}

Wrench Mixer::wrench(const std::array<double, 4>& motorThrusts) const {
    Wrench w;
    for (std::size_t i = 0; i < 4; ++i) {
        w.thrust += motorThrusts[i];
        w.tx += kRollSign[i] * motorThrusts[i];
        w.ty += kPitchSign[i] * motorThrusts[i];
        w.tz += kYawSign[i] * motorThrusts[i];
    }
    w.tx *= lever_;
    w.ty *= lever_;
    w.tz *= yawRatio_;
    return w;
}