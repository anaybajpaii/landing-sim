#include <cmath>

#include "check.hpp"
#include "estimation/target_filter.hpp"
#include "guidance/landing_state_machine.hpp"
#include "math/vec3.hpp"
#include "pipeline/landing_pipeline.hpp"

using core::LandingConfig;
using core::LandingOutput;
using core::LandingState;
using core::LandingStateMachine;
using core::TargetEstimate;
using core::Vec3d;

namespace {

TargetEstimate make_est(double x, double y, double vx, double vy, double sigma, double age) {
    TargetEstimate e;
    e.valid = true;
    e.position = Vec3d{x, y, 0.0};
    e.velocity = Vec3d{vx, vy, 0.0};
    e.pos_sigma = sigma;
    e.age_s = age;
    return e;
}

TargetEstimate aligned() {
    return make_est(0.0, 0.0, 0.5, 0.0, 0.02, 0.0);
}

Vec3d quad_pos(double alt = 1.5) {
    return Vec3d{0.0, 0.0, -alt};
}

Vec3d quad_vel() {
    return Vec3d{0.5, 0.0, 0.0};
}

LandingOutput tick(LandingStateMachine& m, const TargetEstimate& e, const Vec3d& p,
                   const Vec3d& v, int n) {
    LandingOutput out;
    for (int i = 0; i < n; ++i) {
        out = m.update(e, p, v, 1e-3);
    }
    return out;
}

void to_match(LandingStateMachine& m) {
    tick(m, aligned(), quad_pos(), quad_vel(), 2);
}

void to_descend(LandingStateMachine& m) {
    to_match(m);
    tick(m, aligned(), quad_pos(), quad_vel(), 1100);
}

void test_starts_in_search() {
    LandingStateMachine m;
    const LandingOutput o = tick(m, TargetEstimate{}, quad_pos(), quad_vel(), 100);
    CHECK(m.state() == LandingState::Search);
    CHECK_NEAR(o.approach_alt, 1.5, 1e-15);
    CHECK(!o.landed);
}

void test_stale_estimate_stays_in_search() {
    LandingStateMachine m;
    tick(m, make_est(0.0, 0.0, 0.0, 0.0, 0.02, 0.6), quad_pos(), quad_vel(), 100);
    CHECK(m.state() == LandingState::Search);
}

void test_fresh_estimate_starts_tracking() {
    LandingStateMachine m;
    const LandingOutput o = tick(m, make_est(2.0, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(),
                                 quad_vel(), 1);
    CHECK(m.state() == LandingState::Track);
    CHECK_NEAR(o.approach_alt, 1.5, 1e-15);
}

void test_track_does_not_match_when_not_aligned() {
    LandingStateMachine far;
    tick(far, make_est(0.5, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 100);
    CHECK(far.state() == LandingState::Track);

    LandingStateMachine fast;
    tick(fast, make_est(0.0, 0.0, 0.0, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 100);
    CHECK(fast.state() == LandingState::Track);

    LandingStateMachine unsure;
    tick(unsure, make_est(0.0, 0.0, 0.5, 0.0, 0.5, 0.0), quad_pos(), quad_vel(), 100);
    CHECK(unsure.state() == LandingState::Track);
}

void test_aligned_track_enters_match() {
    LandingStateMachine m;
    to_match(m);
    CHECK(m.state() == LandingState::Match);
}

void test_match_holds_before_descending() {
    LandingStateMachine m;
    to_match(m);
    const LandingOutput held = tick(m, aligned(), quad_pos(), quad_vel(), 900);
    CHECK(m.state() == LandingState::Match);
    CHECK_NEAR(held.approach_alt, 1.5, 1e-15);
    const LandingOutput d = tick(m, aligned(), quad_pos(), quad_vel(), 200);
    CHECK(m.state() == LandingState::Descend);
    CHECK_NEAR(d.approach_alt, 0.0, 1e-15);
}

void test_hysteresis() {
    LandingStateMachine in_match;
    to_match(in_match);
    tick(in_match, make_est(0.3, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 100);
    CHECK(in_match.state() == LandingState::Match);
    tick(in_match, make_est(0.4, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 1);
    CHECK(in_match.state() == LandingState::Track);

    LandingStateMachine in_track;
    tick(in_track, make_est(0.3, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 100);
    CHECK(in_track.state() == LandingState::Track);
}

void test_descend_aborts_when_misaligned() {
    LandingStateMachine m;
    to_descend(m);
    CHECK(m.state() == LandingState::Descend);
    const LandingOutput o =
        tick(m, make_est(0.5, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 1);
    CHECK(m.state() == LandingState::Track);
    CHECK_NEAR(o.approach_alt, 1.5, 1e-15);
}

void test_descend_aborts_on_speed_mismatch() {
    LandingStateMachine m;
    to_descend(m);
    tick(m, make_est(0.0, 0.0, -0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 1);
    CHECK(m.state() == LandingState::Track);
}

void test_touchdown() {
    LandingStateMachine m;
    to_descend(m);
    tick(m, aligned(), quad_pos(0.2), quad_vel(), 10);
    CHECK(m.state() == LandingState::Descend);
    const LandingOutput o = tick(m, aligned(), quad_pos(0.05), quad_vel(), 1);
    CHECK(m.state() == LandingState::Landed);
    CHECK(o.landed);
}

void test_landed_is_terminal() {
    LandingStateMachine m;
    to_descend(m);
    tick(m, aligned(), quad_pos(0.05), quad_vel(), 1);
    const LandingOutput o =
        tick(m, make_est(5.0, 5.0, 1.0, 1.0, 0.5, 2.0), quad_pos(1.5), quad_vel(), 3000);
    CHECK(m.state() == LandingState::Landed);
    CHECK(o.landed);
    CHECK_NEAR(o.approach_alt, 0.0, 1e-15);
}

void test_lost_target_times_out() {
    const TargetEstimate stale = make_est(0.0, 0.0, 0.5, 0.0, 0.02, 0.6);

    LandingStateMachine brief;
    tick(brief, make_est(2.0, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 1);
    tick(brief, stale, quad_pos(), quad_vel(), 900);
    CHECK(brief.state() == LandingState::Track);

    LandingStateMachine long_gap;
    tick(long_gap, make_est(2.0, 0.0, 0.5, 0.0, 0.02, 0.0), quad_pos(), quad_vel(), 1);
    tick(long_gap, stale, quad_pos(), quad_vel(), 1100);
    CHECK(long_gap.state() == LandingState::Search);
}

void test_fresh_data_resets_the_lost_timer() {
    const TargetEstimate stale = make_est(0.0, 0.0, 0.5, 0.0, 0.02, 0.6);
    const TargetEstimate far = make_est(2.0, 0.0, 0.5, 0.0, 0.02, 0.0);
    LandingStateMachine m;
    tick(m, far, quad_pos(), quad_vel(), 1);
    tick(m, stale, quad_pos(), quad_vel(), 800);
    tick(m, far, quad_pos(), quad_vel(), 1);
    tick(m, stale, quad_pos(), quad_vel(), 800);
    CHECK(m.state() == LandingState::Track);
}

void test_brief_loss_during_descent_keeps_descending() {
    LandingStateMachine m;
    to_descend(m);
    tick(m, make_est(0.0, 0.0, 0.5, 0.0, 0.02, 0.6), quad_pos(), quad_vel(), 500);
    CHECK(m.state() == LandingState::Descend);
}

void test_lost_target_during_descent_gives_up() {
    LandingStateMachine m;
    to_descend(m);
    tick(m, make_est(0.0, 0.0, 0.5, 0.0, 0.02, 0.6), quad_pos(), quad_vel(), 1100);
    CHECK(m.state() == LandingState::Search);
}

void test_reset_returns_to_search() {
    LandingStateMachine m;
    to_descend(m);
    m.reset();
    CHECK(m.state() == LandingState::Search);
}

void test_lands_on_the_platform() {
    sim::LandingPipeline p;
    double t_track = -1.0;
    double t_match = -1.0;
    double t_descend = -1.0;
    for (int i = 0; i < 30000 && !p.last_output().landed; ++i) {
        p.step();
        const LandingState st = p.last_output().state;
        if (st == LandingState::Track && t_track < 0.0) {
            t_track = p.time();
        }
        if (st == LandingState::Match && t_match < 0.0) {
            t_match = p.time();
        }
        if (st == LandingState::Descend && t_descend < 0.0) {
            t_descend = p.time();
        }
    }
    CHECK(p.last_output().landed);
    CHECK(t_track >= 0.0);
    CHECK(t_match > t_track);
    CHECK(t_descend > t_match);
    CHECK(p.time() > t_descend);
    CHECK(p.time() < 25.0);
    CHECK(p.true_horiz_error() < 0.25);
    CHECK(p.platform().contains_xy(p.state().position));
    CHECK(p.true_rel_speed() < 0.3);
    CHECK(p.altitude() < 0.15);
}

void test_blind_start_never_lands() {
    sim::PipelineConfig cfg;
    cfg.camera.dropout_prob = 1.0;
    sim::LandingPipeline p(cfg);
    for (int i = 0; i < 15000; ++i) {
        p.step();
    }
    CHECK(p.last_output().state == LandingState::Search);
    CHECK(!p.last_output().landed);
    CHECK_NEAR(p.altitude(), 1.0, 0.05);
}

void test_recovers_from_a_blackout_and_lands() {
    sim::LandingPipeline p;
    for (int i = 0; i < 30000 && p.last_output().state != LandingState::Descend; ++i) {
        p.step();
    }
    CHECK(p.last_output().state == LandingState::Descend);

    p.set_blind(true);
    for (int i = 0; i < 2500; ++i) {
        p.step();
    }
    CHECK(p.last_output().state == LandingState::Search);
    CHECK(!p.last_output().landed);

    p.set_blind(false);
    for (int i = 0; i < 40000 && !p.last_output().landed; ++i) {
        p.step();
    }
    CHECK(p.last_output().landed);
    CHECK(p.platform().contains_xy(p.state().position));
}

}

int main() {
    test_starts_in_search();
    test_stale_estimate_stays_in_search();
    test_fresh_estimate_starts_tracking();
    test_track_does_not_match_when_not_aligned();
    test_aligned_track_enters_match();
    test_match_holds_before_descending();
    test_hysteresis();
    test_descend_aborts_when_misaligned();
    test_descend_aborts_on_speed_mismatch();
    test_touchdown();
    test_landed_is_terminal();
    test_lost_target_times_out();
    test_fresh_data_resets_the_lost_timer();
    test_brief_loss_during_descent_keeps_descending();
    test_lost_target_during_descent_gives_up();
    test_reset_returns_to_search();
    test_lands_on_the_platform();
    test_blind_start_never_lands();
    test_recovers_from_a_blackout_and_lands();
    return check::report();
}