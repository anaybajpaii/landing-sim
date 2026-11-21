#pragma once

#include <array>
#include <cstddef>

#include "math/vec3.hpp"

namespace core {

template <std::size_t N>
class PositionHistory {
    static_assert(N >= 2, "history needs at least two samples");

    struct Sample {
        double t = 0.0;
        Vec3d p{};
    };

public:
    void clear() {
        head_ = 0;
        count_ = 0;
    }

    void push(double t, const Vec3d& p) {
        if (count_ == N) {
            head_ = (head_ + 1) % N;
            --count_;
        }
        buf_[(head_ + count_) % N] = Sample{t, p};
        ++count_;
    }

    bool at(double t, Vec3d& out) const {
        if (count_ == 0) {
            return false;
        }
        const Sample& oldest = get(0);
        const Sample& newest = get(count_ - 1);
        if (t < oldest.t || t > newest.t) {
            return false;
        }
        for (std::size_t k = count_ - 1; k > 0; --k) {
            const Sample& hi = get(k);
            const Sample& lo = get(k - 1);
            if (t >= lo.t) {
                const double span = hi.t - lo.t;
                const double w = (span > 0.0) ? (t - lo.t) / span : 1.0;
                out = lo.p + (hi.p - lo.p) * w;
                return true;
            }
        }
        out = oldest.p;
        return true;
    }

    std::size_t size() const { return count_; }

private:
    const Sample& get(std::size_t k) const { return buf_[(head_ + k) % N]; }

    std::array<Sample, N> buf_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

}