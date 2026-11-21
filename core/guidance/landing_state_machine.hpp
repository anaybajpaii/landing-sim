#pragma once

#include <cmath>

#include "estimation/target_filter.hpp"
#include "math/vec3.hpp"

namespace core {

enum class LandingState { Search, Track, Match, Descend, Landed };

inline const char* to_string(LandingState s) {
    switch (s) {
        case LandingState::Search:
            return "search";
        case LandingState::Track:
            return "track";
        case LandingState::Match:
            return "match";
        case LandingState::Descend:
            return "descend";
        case LandingState::Landed:
            return "landed";
    }
    return "unknown";
}

struct LandingConfig {
    double search_alt = 1.5;
    double track_alt = 1.5;
    double descent_alt = 0.0;
    double match_err_enter = 0.20;
    double match_err_exit = 0.35;
    double match_speed_enter = 0.25;
    double match_speed_exit = 0.5;
    double match_hold_s = 1.0;
    double descend_err_abort = 0.35;
    double descend_speed_abort = 0.6;
    double lost_timeout_s = 1.0;
    double touchdown_alt = 0.10;
    double max_age_s = 0.5;
    double max_sigma = 0.1;
};

struct LandingOutput {
    LandingState state = LandingState::Search;
    double approach_alt = 0.0;
    bool landed = false;
    double horiz_error = 0.0;
    double rel_speed = 0.0;
};

class LandingStateMachine {
public:
    explicit LandingStateMachine(const LandingConfig& config = LandingConfig{})
        : cfg_(config) {}

    void reset() {
        state_ = LandingState::Search;
        time_in_state_ = 0.0;
        lost_time_ = 0.0;
        hold_time_ = 0.0;
    }

    LandingOutput update(const TargetEstimate& est, const Vec3d& quad_pos, const Vec3d& quad_vel,
                         double dt) {
        const bool fresh = est.valid && est.age_s <= cfg_.max_age_s;
        const bool sigma_ok = fresh && est.pos_sigma <= cfg_.max_sigma;
        const double altitude = -quad_pos.z;

        LandingOutput out;
        if (fresh) {
            out.horiz_error = std::hypot(est.position.x - quad_pos.x, est.position.y - quad_pos.y);
            out.rel_speed = std::hypot(quad_vel.x - est.velocity.x, quad_vel.y - est.velocity.y);
        }

        time_in_state_ += dt;
        if (state_ != LandingState::Landed) {
            if (fresh) {
                lost_time_ = 0.0;
            } else {
                lost_time_ += dt;
            }
        }

        switch (state_) {
            case LandingState::Search:
                if (fresh) {
                    enter(LandingState::Track);
                }
                break;
            case LandingState::Track:
                if (lost_time_ > cfg_.lost_timeout_s) {
                    enter(LandingState::Search);
                } else if (sigma_ok && out.horiz_error < cfg_.match_err_enter &&
                           out.rel_speed < cfg_.match_speed_enter) {
                    enter(LandingState::Match);
                }
                break;
            case LandingState::Match:
                if (lost_time_ > cfg_.lost_timeout_s) {
                    enter(LandingState::Search);
                } else if (!sigma_ok || out.horiz_error >= cfg_.match_err_exit ||
                           out.rel_speed >= cfg_.match_speed_exit) {
                    enter(LandingState::Track);
                } else {
                    hold_time_ += dt;
                    if (hold_time_ >= cfg_.match_hold_s) {
                        enter(LandingState::Descend);
                    }
                }
                break;
            case LandingState::Descend:
                if (lost_time_ > cfg_.lost_timeout_s) {
                    enter(LandingState::Search);
                } else if (fresh && (!sigma_ok || out.horiz_error >= cfg_.descend_err_abort ||
                                     out.rel_speed >= cfg_.descend_speed_abort)) {
                    enter(LandingState::Track);
                } else if (altitude <= cfg_.touchdown_alt) {
                    enter(LandingState::Landed);
                }
                break;
            case LandingState::Landed:
                break;
        }

        out.state = state_;
        out.landed = (state_ == LandingState::Landed);
        out.approach_alt = approach_altitude();
        return out;
    }

    LandingState state() const { return state_; }
    double time_in_state() const { return time_in_state_; }
    const LandingConfig& config() const { return cfg_; }

private:
    void enter(LandingState s) {
        state_ = s;
        time_in_state_ = 0.0;
        hold_time_ = 0.0;
    }

    double approach_altitude() const {
        switch (state_) {
            case LandingState::Search:
                return cfg_.search_alt;
            case LandingState::Track:
            case LandingState::Match:
                return cfg_.track_alt;
            case LandingState::Descend:
            case LandingState::Landed:
                return cfg_.descent_alt;
        }
        return cfg_.track_alt;
    }

    LandingConfig cfg_;
    LandingState state_ = LandingState::Search;
    double time_in_state_ = 0.0;
    double lost_time_ = 0.0;
    double hold_time_ = 0.0;
};

}