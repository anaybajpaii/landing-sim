#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "check.hpp"
#include "pipeline/batch.hpp"

using sim::BatchConfig;
using sim::BatchSummary;
using sim::Outcome;
using sim::RunResult;

namespace {

constexpr double kTwoPi = 6.283185307179586;

RunResult make_result(bool touched, bool success, bool on_platform, bool vertical_ok,
                      bool rel_ok, bool tilt_ok, double time, double offset, double vz) {
    RunResult r;
    r.touched = touched;
    r.td.touched = touched;
    r.td.success = success;
    r.td.on_platform = on_platform;
    r.td.vertical_ok = vertical_ok;
    r.td.rel_speed_ok = rel_ok;
    r.td.tilt_ok = tilt_ok;
    r.td.time = time;
    r.td.offset = offset;
    r.td.vertical_speed = vz;
    return r;
}

void test_wilson_lower_bound() {
    CHECK_NEAR(sim::wilson_lower(45, 50), 0.7864, 0.002);
    CHECK_NEAR(sim::wilson_lower(50, 50), 0.9286, 0.002);
    CHECK_NEAR(sim::wilson_lower(0, 50), 0.0, 1e-9);
    CHECK_NEAR(sim::wilson_lower(0, 0), 0.0, 1e-15);
    CHECK(sim::wilson_lower(30, 50) < 0.6);
    CHECK(sim::wilson_lower(48, 50) > sim::wilson_lower(45, 50));
}

void test_sampling_is_repeatable() {
    BatchConfig b;
    BatchConfig other;
    other.seed = 2;
    bool differs = false;
    for (int i = 0; i < 20; ++i) {
        const RunResult a = sim::sample_run(b, i);
        const RunResult a2 = sim::sample_run(b, i);
        const RunResult c = sim::sample_run(other, i);
        CHECK(a.camera_seed == a2.camera_seed);
        CHECK(a.platform_x0 == a2.platform_x0);
        CHECK(a.platform_y0 == a2.platform_y0);
        CHECK(a.heading == a2.heading);
        CHECK(a.start_alt == a2.start_alt);
        if (a.camera_seed != c.camera_seed || a.platform_x0 != c.platform_x0) {
            differs = true;
        }
    }
    CHECK(differs);
}

void test_sampling_respects_ranges() {
    BatchConfig b;
    std::vector<std::uint64_t> seeds;
    for (int i = 0; i < 200; ++i) {
        const RunResult r = sim::sample_run(b, i);
        const double dist = std::hypot(r.platform_x0, r.platform_y0);
        CHECK(dist >= b.dist_min - 1e-12);
        CHECK(dist <= b.dist_max + 1e-12);
        CHECK(r.start_alt >= b.alt_min);
        CHECK(r.start_alt <= b.alt_max);
        CHECK(r.heading >= 0.0);
        CHECK(r.heading < kTwoPi);
        CHECK(r.index == i);
        seeds.push_back(r.camera_seed);
    }
    bool unique = true;
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        for (std::size_t j = i + 1; j < seeds.size(); ++j) {
            if (seeds[i] == seeds[j]) {
                unique = false;
            }
        }
    }
    CHECK(unique);
}

void test_pipeline_config_from_run() {
    BatchConfig b;
    const RunResult r = sim::sample_run(b, 7);
    const sim::PipelineConfig cfg = sim::make_pipeline_config(b, r);
    CHECK(cfg.camera.seed == r.camera_seed);
    CHECK_NEAR(cfg.platform_position.x, r.platform_x0, 1e-15);
    CHECK_NEAR(cfg.platform_position.y, r.platform_y0, 1e-15);
    CHECK_NEAR(std::hypot(cfg.platform_velocity.x, cfg.platform_velocity.y), 0.5, 1e-12);
    CHECK_NEAR(cfg.platform_velocity.x, 0.5 * std::cos(r.heading), 1e-12);
    CHECK_NEAR(cfg.start_altitude, r.start_alt, 1e-15);
    CHECK_NEAR(cfg.platform_half_size, 0.5, 1e-15);
}

void test_classification() {
    CHECK(sim::classify(make_result(false, false, false, false, false, false, 0, 0, 0)) ==
          Outcome::Timeout);
    CHECK(sim::classify(make_result(true, true, true, true, true, true, 1, 0, 0)) ==
          Outcome::Success);
    CHECK(sim::classify(make_result(true, false, false, true, true, true, 1, 0, 0)) ==
          Outcome::OffPlatform);
    CHECK(sim::classify(make_result(true, false, true, false, true, true, 1, 0, 0)) ==
          Outcome::HardLanding);
    CHECK(sim::classify(make_result(true, false, true, true, false, true, 1, 0, 0)) ==
          Outcome::RelativeSpeed);
    CHECK(sim::classify(make_result(true, false, true, true, true, false, 1, 0, 0)) ==
          Outcome::Tilt);
    CHECK(sim::classify(make_result(true, false, false, false, false, false, 1, 0, 0)) ==
          Outcome::OffPlatform);
}

void test_summary_counts() {
    std::vector<RunResult> rs;
    rs.push_back(make_result(true, true, true, true, true, true, 10.0, 0.1, 0.1));
    rs.push_back(make_result(true, true, true, true, true, true, 12.0, 0.3, 0.2));
    rs.push_back(make_result(false, false, false, false, false, false, 0.0, 0.0, 0.0));
    rs.push_back(make_result(true, false, false, true, true, true, 9.0, 0.7, 0.1));
    rs.push_back(make_result(true, false, true, false, true, true, 9.0, 0.1, 0.9));
    const BatchSummary s = sim::summarize(rs);
    CHECK(s.runs == 5);
    CHECK(s.successes == 2);
    CHECK(s.timeouts == 1);
    CHECK(s.off_platform == 1);
    CHECK(s.hard_landings == 1);
    CHECK(s.relative_speed == 0);
    CHECK(s.tilt == 0);
    CHECK_NEAR(s.success_rate, 0.4, 1e-12);
    CHECK_NEAR(s.mean_success_time, 11.0, 1e-12);
    CHECK_NEAR(s.max_success_offset, 0.3, 1e-12);
    CHECK_NEAR(s.max_success_vertical, 0.2, 1e-12);
    CHECK_NEAR(s.wilson_lower, sim::wilson_lower(2, 5), 1e-15);
}

void test_empty_summary() {
    const BatchSummary s = sim::summarize({});
    CHECK(s.runs == 0);
    CHECK(s.successes == 0);
    CHECK_NEAR(s.success_rate, 0.0, 1e-15);
    CHECK_NEAR(s.wilson_lower, 0.0, 1e-15);
}

void test_batch_is_deterministic() {
    BatchConfig b;
    b.runs = 3;
    const std::vector<RunResult> first = sim::run_batch(b);
    const std::vector<RunResult> second = sim::run_batch(b);
    CHECK(first.size() == 3);
    CHECK(second.size() == 3);
    for (std::size_t i = 0; i < first.size(); ++i) {
        CHECK(first[i].touched == second[i].touched);
        CHECK(first[i].td.success == second[i].td.success);
        CHECK(first[i].td.time == second[i].td.time);
        CHECK(first[i].td.offset == second[i].td.offset);
        CHECK(first[i].td.vertical_speed == second[i].td.vertical_speed);
        CHECK(first[i].end_time == second[i].end_time);
    }
}

void test_small_batch_runs_and_mostly_lands() {
    BatchConfig b;
    b.runs = 3;
    const std::vector<RunResult> results = sim::run_batch(b);
    const BatchSummary s = sim::summarize(results);
    CHECK(s.runs == 3);
    CHECK(s.successes + s.timeouts + s.off_platform + s.hard_landings + s.relative_speed +
              s.tilt ==
          3);
    CHECK(s.successes >= 1);
    for (const RunResult& r : results) {
        CHECK(r.end_time <= b.max_time_s + 1e-9);
    }
}

void test_impossible_platform_never_succeeds() {
    BatchConfig b;
    b.runs = 2;
    b.platform_half_size = 0.0;
    const BatchSummary s = sim::summarize(sim::run_batch(b));
    CHECK(s.runs == 2);
    CHECK(s.successes == 0);
}

}

int main() {
    test_wilson_lower_bound();
    test_sampling_is_repeatable();
    test_sampling_respects_ranges();
    test_pipeline_config_from_run();
    test_classification();
    test_summary_counts();
    test_empty_summary();
    test_batch_is_deterministic();
    test_small_batch_runs_and_mostly_lands();
    test_impossible_platform_never_succeeds();
    return check::report();
}