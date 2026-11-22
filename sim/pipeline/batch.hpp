#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "math/vec3.hpp"
#include "pipeline/landing_pipeline.hpp"
#include "pipeline/touchdown.hpp"
#include "sensors/rng.hpp"

namespace sim {

struct BatchConfig {
    int runs = 50;
    std::uint64_t seed = 1;
    double max_time_s = 40.0;
    double dist_min = 2.0;
    double dist_max = 5.0;
    double speed = 0.5;
    double alt_min = 0.8;
    double alt_max = 1.5;
    double platform_half_size = 0.5;
};

struct RunResult {
    int index = 0;
    std::uint64_t camera_seed = 0;
    double platform_x0 = 0.0;
    double platform_y0 = 0.0;
    double heading = 0.0;
    double start_alt = 0.0;
    bool touched = false;
    double end_time = 0.0;
    TouchdownResult td;
};

enum class Outcome { Success, Timeout, OffPlatform, HardLanding, RelativeSpeed, Tilt };

inline Outcome classify(const RunResult& r) {
    if (!r.touched) {
        return Outcome::Timeout;
    }
    if (r.td.success) {
        return Outcome::Success;
    }
    if (!r.td.on_platform) {
        return Outcome::OffPlatform;
    }
    if (!r.td.vertical_ok) {
        return Outcome::HardLanding;
    }
    if (!r.td.rel_speed_ok) {
        return Outcome::RelativeSpeed;
    }
    return Outcome::Tilt;
}

inline const char* outcome_name(Outcome o) {
    switch (o) {
        case Outcome::Success:
            return "success";
        case Outcome::Timeout:
            return "timeout";
        case Outcome::OffPlatform:
            return "off_platform";
        case Outcome::HardLanding:
            return "hard_landing";
        case Outcome::RelativeSpeed:
            return "relative_speed";
        case Outcome::Tilt:
            return "tilt";
    }
    return "unknown";
}

inline RunResult sample_run(const BatchConfig& b, int index) {
    constexpr double kTwoPi = 6.283185307179586;
    Rng rng(b.seed ^ ((static_cast<std::uint64_t>(index) + 1ULL) * 0x9E3779B97F4A7C15ULL));

    RunResult r;
    r.index = index;
    const double dist = b.dist_min + (b.dist_max - b.dist_min) * rng.uniform();
    const double angle = kTwoPi * rng.uniform();
    r.heading = kTwoPi * rng.uniform();
    r.start_alt = b.alt_min + (b.alt_max - b.alt_min) * rng.uniform();
    r.platform_x0 = dist * std::cos(angle);
    r.platform_y0 = dist * std::sin(angle);
    r.camera_seed = rng.next();
    return r;
}

inline PipelineConfig make_pipeline_config(const BatchConfig& b, const RunResult& r) {
    PipelineConfig cfg;
    cfg.camera.seed = r.camera_seed;
    cfg.platform_position = core::Vec3d{r.platform_x0, r.platform_y0, 0.0};
    cfg.platform_velocity =
        core::Vec3d{b.speed * std::cos(r.heading), b.speed * std::sin(r.heading), 0.0};
    cfg.platform_half_size = b.platform_half_size;
    cfg.start_altitude = r.start_alt;
    return cfg;
}

inline RunResult run_one(const BatchConfig& b, int index) {
    RunResult r = sample_run(b, index);
    LandingPipeline pipeline(make_pipeline_config(b, r));
    const int max_steps = static_cast<int>(b.max_time_s / LandingPipeline::kDt);
    for (int i = 0; i < max_steps && !pipeline.touchdown().touched; ++i) {
        pipeline.step();
    }
    r.touched = pipeline.touchdown().touched;
    r.td = pipeline.touchdown();
    r.end_time = pipeline.time();
    return r;
}

inline std::vector<RunResult> run_batch(const BatchConfig& b) {
    std::vector<RunResult> results;
    results.reserve(static_cast<std::size_t>(std::max(b.runs, 0)));
    for (int i = 0; i < b.runs; ++i) {
        results.push_back(run_one(b, i));
    }
    return results;
}

inline double wilson_lower(int successes, int runs) {
    if (runs <= 0) {
        return 0.0;
    }
    const double z = 1.96;
    const double n = static_cast<double>(runs);
    const double p = static_cast<double>(successes) / n;
    const double z2 = z * z;
    const double centre = p + z2 / (2.0 * n);
    const double margin = z * std::sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n));
    const double lower = (centre - margin) / (1.0 + z2 / n);
    return lower > 0.0 ? lower : 0.0;
}

struct BatchSummary {
    int runs = 0;
    int successes = 0;
    int timeouts = 0;
    int off_platform = 0;
    int hard_landings = 0;
    int relative_speed = 0;
    int tilt = 0;
    double success_rate = 0.0;
    double wilson_lower = 0.0;
    double mean_success_time = 0.0;
    double max_success_offset = 0.0;
    double max_success_vertical = 0.0;
};

inline BatchSummary summarize(const std::vector<RunResult>& results) {
    BatchSummary s;
    s.runs = static_cast<int>(results.size());
    double time_sum = 0.0;
    for (const RunResult& r : results) {
        switch (classify(r)) {
            case Outcome::Success:
                ++s.successes;
                time_sum += r.td.time;
                s.max_success_offset = std::max(s.max_success_offset, r.td.offset);
                s.max_success_vertical = std::max(s.max_success_vertical, r.td.vertical_speed);
                break;
            case Outcome::Timeout:
                ++s.timeouts;
                break;
            case Outcome::OffPlatform:
                ++s.off_platform;
                break;
            case Outcome::HardLanding:
                ++s.hard_landings;
                break;
            case Outcome::RelativeSpeed:
                ++s.relative_speed;
                break;
            case Outcome::Tilt:
                ++s.tilt;
                break;
        }
    }
    if (s.runs > 0) {
        s.success_rate = static_cast<double>(s.successes) / static_cast<double>(s.runs);
    }
    if (s.successes > 0) {
        s.mean_success_time = time_sum / static_cast<double>(s.successes);
    }
    s.wilson_lower = wilson_lower(s.successes, s.runs);
    return s;
}

}