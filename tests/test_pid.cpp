#include <cmath>

#include "check.hpp"
#include "control/pid.hpp"

using core::Pid;
using core::PidConfig;

namespace {

void test_proportional_only() {
    PidConfig c;
    c.kp = 2.0;
    Pid pid(c);
    CHECK_NEAR(pid.update(1.0, 0.5, 0.01), 1.0, 1e-12);
}

void test_integral_accumulates() {
    PidConfig c;
    c.ki = 1.0;
    Pid pid(c);
    double out = 0.0;
    for (int i = 0; i < 10; ++i) {
        out = pid.update(1.0, 0.0, 0.1);
    }
    CHECK_NEAR(out, 1.0, 1e-12);
    CHECK_NEAR(pid.integral(), 1.0, 1e-12);
}

void test_derivative_on_measurement_has_no_setpoint_kick() {
    PidConfig c;
    c.kd = 1.0;
    Pid pid(c);
    pid.update(0.0, 0.0, 0.1);
    const double out = pid.update(10.0, 0.0, 0.1);
    CHECK_NEAR(out, 0.0, 1e-12);
}

void test_derivative_opposes_measurement_motion() {
    PidConfig c;
    c.kd = 1.0;
    Pid pid(c);
    pid.update(0.0, 0.0, 0.1);
    const double out = pid.update(0.0, 0.1, 0.1);
    CHECK_NEAR(out, -1.0, 1e-12);
}

void test_derivative_filter_smooths_step() {
    PidConfig c;
    c.kd = 1.0;
    c.deriv_tau = 0.1;
    Pid pid(c);
    pid.update(0.0, 0.0, 0.01);
    const double out = pid.update(0.0, 1.0, 0.01);
    CHECK_NEAR(out, -100.0 * 0.01 / 0.11, 1e-9);
}

void test_output_clamps() {
    PidConfig c;
    c.kp = 10.0;
    c.out_min = -1.0;
    c.out_max = 1.0;
    Pid pid(c);
    CHECK_NEAR(pid.update(5.0, 0.0, 0.01), 1.0, 1e-12);
    CHECK_NEAR(pid.update(-5.0, 0.0, 0.01), -1.0, 1e-12);
}

void test_anti_windup_recovers_quickly() {
    PidConfig c;
    c.ki = 1.0;
    c.out_min = -1.0;
    c.out_max = 1.0;
    Pid pid(c);
    double out = 0.0;
    for (int i = 0; i < 100; ++i) {
        out = pid.update(10.0, 0.0, 0.1);
    }
    CHECK_NEAR(out, 1.0, 1e-12);
    CHECK_NEAR(pid.integral(), 1.0, 1e-12);
    out = pid.update(-10.0, 0.0, 0.1);
    CHECK(out < 0.5);
}

void test_reset_clears_state() {
    PidConfig c;
    c.ki = 1.0;
    c.kd = 1.0;
    Pid pid(c);
    pid.update(1.0, 0.0, 0.1);
    pid.update(1.0, 0.3, 0.1);
    pid.reset();
    CHECK_NEAR(pid.integral(), 0.0, 1e-15);
    const double out = pid.update(0.0, 5.0, 0.1);
    CHECK_NEAR(out, -0.5, 1e-12);
}

void test_non_positive_dt_returns_last_output() {
    PidConfig c;
    c.kp = 1.0;
    Pid pid(c);
    const double first = pid.update(1.0, 0.0, 0.1);
    CHECK_NEAR(first, 1.0, 1e-12);
    CHECK_NEAR(pid.update(5.0, 0.0, 0.0), 1.0, 1e-12);
    CHECK_NEAR(pid.update(5.0, 0.0, -0.1), 1.0, 1e-12);
}

void test_closed_loop_first_order_plant_converges() {
    PidConfig c;
    c.kp = 2.0;
    Pid pid(c);
    double x = 0.0;
    for (int i = 0; i < 500; ++i) {
        const double u = pid.update(1.0, x, 0.01);
        x += u * 0.01;
    }
    CHECK_NEAR(x, 1.0, 1e-3);
}

}

int main() {
    test_proportional_only();
    test_integral_accumulates();
    test_derivative_on_measurement_has_no_setpoint_kick();
    test_derivative_opposes_measurement_motion();
    test_derivative_filter_smooths_step();
    test_output_clamps();
    test_anti_windup_recovers_quickly();
    test_reset_clears_state();
    test_non_positive_dt_returns_last_output();
    test_closed_loop_first_order_plant_converges();
    return check::report();
}