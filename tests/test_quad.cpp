#include <cmath>

#include "check.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"

using core::Vec3d;
using sim::Quad;
using sim::State;

namespace {

double hover_omega(const QuadParams& p) {
    return std::sqrt(p.mass * p.gravity / (4.0 * p.kThrust));
}

void run(Quad& q, double thrust, const Vec3d& torque, double dt, int steps) {
    for (int i = 0; i < steps; ++i) {
        q.step(thrust, torque, dt);
    }
}

void test_free_fall_with_motors_off() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, 0.0);
    run(q, 0.0, Vec3d{}, 1e-3, 1000);
    CHECK_NEAR(q.state().position.z, 0.5 * p.gravity, 1e-9);
    CHECK_NEAR(q.state().velocity.z, p.gravity, 1e-9);
    CHECK_NEAR(q.state().position.x, 0.0, 1e-12);
    CHECK_NEAR(q.state().position.y, 0.0, 1e-12);
}

void test_hover_stays_put() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, hover_omega(p));
    run(q, p.mass * p.gravity, Vec3d{}, 1e-3, 5000);
    CHECK_NEAR(q.state().position.x, 0.0, 1e-8);
    CHECK_NEAR(q.state().position.y, 0.0, 1e-8);
    CHECK_NEAR(q.state().position.z, 0.0, 1e-8);
    CHECK_NEAR(q.state().velocity.z, 0.0, 1e-8);
    CHECK_NEAR(q.state().attitude.w, 1.0, 1e-12);
}

void test_motor_lag_then_reaches_hover_thrust() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, 0.0);
    const double weight = p.mass * p.gravity;
    q.step(weight, Vec3d{}, 1e-3);
    CHECK(q.wrench().thrust_n < 0.1 * weight);
    run(q, weight, Vec3d{}, 1e-3, 2000);
    CHECK_NEAR(q.wrench().thrust_n, weight, 1e-6);
}

void test_positive_roll_torque_rolls_right_and_moves_right() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, hover_omega(p));
    run(q, p.mass * p.gravity, Vec3d{0.02, 0.0, 0.0}, 1e-3, 200);
    CHECK(q.state().omega.x > 0.0);
    CHECK_NEAR(q.state().omega.y, 0.0, 1e-9);
    CHECK_NEAR(q.state().omega.z, 0.0, 1e-9);
    CHECK(core::euler_zyx(q.state().attitude).x > 0.0);
    CHECK(q.state().velocity.y > 0.0);
}

void test_positive_pitch_torque_noses_up_and_moves_back() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, hover_omega(p));
    run(q, p.mass * p.gravity, Vec3d{0.0, 0.02, 0.0}, 1e-3, 200);
    CHECK(q.state().omega.y > 0.0);
    CHECK_NEAR(q.state().omega.x, 0.0, 1e-9);
    CHECK_NEAR(q.state().omega.z, 0.0, 1e-9);
    CHECK(core::euler_zyx(q.state().attitude).y > 0.0);
    CHECK(q.state().velocity.x < 0.0);
}

void test_positive_yaw_torque_yaws_right() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, hover_omega(p));
    run(q, p.mass * p.gravity, Vec3d{0.0, 0.0, 0.005}, 1e-3, 200);
    CHECK(q.state().omega.z > 0.0);
    CHECK_NEAR(q.state().omega.x, 0.0, 1e-9);
    CHECK_NEAR(q.state().omega.y, 0.0, 1e-9);
    CHECK(core::euler_zyx(q.state().attitude).z > 0.0);
}

void test_quaternion_stays_unit_under_torque() {
    const QuadParams p;
    Quad q(p);
    q.reset(State{}, hover_omega(p));
    run(q, p.mass * p.gravity, Vec3d{0.02, -0.01, 0.003}, 1e-3, 3000);
    CHECK_NEAR(core::norm(q.state().attitude), 1.0, 1e-12);
}

}

int main() {
    test_free_fall_with_motors_off();
    test_hover_stays_put();
    test_motor_lag_then_reaches_hover_thrust();
    test_positive_roll_torque_rolls_right_and_moves_right();
    test_positive_pitch_torque_noses_up_and_moves_back();
    test_positive_yaw_torque_yaws_right();
    test_quaternion_stays_unit_under_torque();
    return check::report();
}