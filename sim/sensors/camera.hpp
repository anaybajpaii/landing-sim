#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "math/vec3.hpp"
#include "sensors/rng.hpp"

namespace sim {

struct CameraConfig {
    double rate_hz = 30.0;
    double delay_s = 0.05;
    double sigma_base_m = 0.02;
    double sigma_per_m = 0.01;
    double dropout_prob = 0.05;
    std::uint64_t seed = 1;
};

struct CameraMeasurement {
    double capture_time = 0.0;
    core::Vec3d rel_pos{};
};

class Camera {
public:
    explicit Camera(const CameraConfig& config = CameraConfig{})
        : cfg_(config), rng_(config.seed) {}

    void reset() {
        rng_ = Rng(cfg_.seed);
        next_frame_ = 0.0;
        head_ = 0;
        count_ = 0;
    }

    void capture(double t, const core::Vec3d& true_rel, double altitude) {
        if (t + kEps < next_frame_) {
            return;
        }
        next_frame_ += 1.0 / cfg_.rate_hz;

        const bool dropped = rng_.uniform() < cfg_.dropout_prob;
        const double nx = rng_.normal();
        const double ny = rng_.normal();
        if (dropped) {
            return;
        }

        const double sigma = cfg_.sigma_base_m + cfg_.sigma_per_m * std::max(altitude, 0.0);
        Pending p;
        p.ready_time = t + cfg_.delay_s;
        p.m.capture_time = t;
        p.m.rel_pos = core::Vec3d{true_rel.x + sigma * nx, true_rel.y + sigma * ny, true_rel.z};
        push(p);
    }

    bool poll(double t, CameraMeasurement& out) {
        if (count_ == 0) {
            return false;
        }
        const Pending& p = buf_[head_];
        if (p.ready_time > t + kEps) {
            return false;
        }
        out = p.m;
        head_ = (head_ + 1) % kCapacity;
        --count_;
        return true;
    }

    const CameraConfig& config() const { return cfg_; }

private:
    struct Pending {
        double ready_time = 0.0;
        CameraMeasurement m;
    };

    static constexpr std::size_t kCapacity = 8;
    static constexpr double kEps = 1e-9;

    void push(const Pending& p) {
        if (count_ == kCapacity) {
            head_ = (head_ + 1) % kCapacity;
            --count_;
        }
        buf_[(head_ + count_) % kCapacity] = p;
        ++count_;
    }

    CameraConfig cfg_;
    Rng rng_;
    double next_frame_ = 0.0;
    std::array<Pending, kCapacity> buf_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

}