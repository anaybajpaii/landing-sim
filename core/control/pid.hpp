#pragma once

#include <algorithm>
#include <limits>

namespace core {

struct PidConfig {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double out_min = -std::numeric_limits<double>::infinity();
    double out_max = std::numeric_limits<double>::infinity();
    double deriv_tau = 0.0;
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidConfig& config) : cfg_(config) {}

    void reset() {
        integral_ = 0.0;
        deriv_filtered_ = 0.0;
        prev_measurement_ = 0.0;
        last_output_ = 0.0;
        has_prev_ = false;
    }

    double update(double setpoint, double measurement, double dt) {
        if (!(dt > 0.0)) {
            return last_output_;
        }

        const double error = setpoint - measurement;

        double d_meas = 0.0;
        if (has_prev_) {
            const double raw = (measurement - prev_measurement_) / dt;
            if (cfg_.deriv_tau > 0.0) {
                const double alpha = dt / (cfg_.deriv_tau + dt);
                deriv_filtered_ += alpha * (raw - deriv_filtered_);
            } else {
                deriv_filtered_ = raw;
            }
            d_meas = deriv_filtered_;
        }
        prev_measurement_ = measurement;
        has_prev_ = true;

        const double p = cfg_.kp * error;
        const double d = -cfg_.kd * d_meas;

        const double candidate = integral_ + cfg_.ki * error * dt;
        const double unsaturated = p + candidate + d;
        const bool pushing_high = unsaturated > cfg_.out_max && error > 0.0;
        const bool pushing_low = unsaturated < cfg_.out_min && error < 0.0;
        if (!pushing_high && !pushing_low) {
            integral_ = candidate;
        }

        last_output_ = std::clamp(p + integral_ + d, cfg_.out_min, cfg_.out_max);
        return last_output_;
    }

    double integral() const { return integral_; }
    double last_output() const { return last_output_; }
    const PidConfig& config() const { return cfg_; }

private:
    PidConfig cfg_;
    double integral_ = 0.0;
    double deriv_filtered_ = 0.0;
    double prev_measurement_ = 0.0;
    double last_output_ = 0.0;
    bool has_prev_ = false;
};

}