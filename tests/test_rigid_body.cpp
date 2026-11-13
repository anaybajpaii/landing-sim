#include <cmath>

#include "check.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/rigid_body.hpp"
#include "plant/vehicle_params.hpp"

using core::Quatd;
using core::Vec3d;
using sim::State;
using sim::StateDerivative;
using sim::VehicleParams;
using sim::Wrench;

namespace {

constexpr double kPi = 3.14159265358979323846;

VehicleParams test_params() {
    VehicleParams p;
    p.mass_kg = 2.0;
    p.inertia_kgm2 = Vec3d{0.01, 0.02, 0.03};
    p.gravity_mps2 = 10.0;
    return p;
}

void check_vec(const Vec3d& v, double x, double y, double z, double tol = 1e-12) {
    CHECK_NEAR(v.x, x, tol);
    CHECK_NEAR(v.y, y, tol);
    CHECK_NEAR(v.z, z, tol);
}

void check_quat(const Quatd& q, double w, double x, double y, double z, double tol = 1e-12) {
    CHECK_NEAR(q.w, w, tol);
    CHECK_NEAR(q.x, x, tol);
    CHECK_NEAR(q.y, y, tol);
    CHECK_NEAR(q.z, z, tol);
}

void test_position_derivative_is_velocity() {
    State s;
    s.velocity = Vec3d{1.0, -2.0, 0.5};
    const StateDerivative d = sim::derivative(s, Wrench{}, test_params());
    check_vec(d.d_position, 1.0, -2.0, 0.5);
}

void test_free_fall() {
    const StateDerivative d = sim::derivative(State{}, Wrench{}, test_params());
    check_vec(d.d_velocity, 0.0, 0.0, 10.0);
}

void test_hover_thrust_equals_weight() {
    const VehicleParams p = test_params();
    Wrench w;
    w.thrust_n = p.mass_kg * p.gravity_mps2;
    const StateDerivative d = sim::derivative(State{}, w, p);
    check_vec(d.d_velocity, 0.0, 0.0, 0.0);
}

void test_excess_thrust_accelerates_up() {
    const VehicleParams p = test_params();
    Wrench w;
    w.thrust_n = 2.0 * p.mass_kg * p.gravity_mps2;
    const StateDerivative d = sim::derivative(State{}, w, p);
    check_vec(d.d_velocity, 0.0, 0.0, -10.0);
}

void test_tilted_thrust_goes_sideways() {
    const VehicleParams p = test_params();
    Wrench w;
    w.thrust_n = p.mass_kg * p.gravity_mps2;

    State rolled;
    rolled.attitude = core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, kPi / 2.0);
    check_vec(sim::derivative(rolled, w, p).d_velocity, 0.0, 10.0, 10.0);

    State pitched;
    pitched.attitude = core::from_axis_angle(Vec3d{0.0, 1.0, 0.0}, kPi / 2.0);
    check_vec(sim::derivative(pitched, w, p).d_velocity, -10.0, 0.0, 10.0);
}

void test_tilt_keeps_vertical_component() {
    const VehicleParams p = test_params();
    const double tilt = 0.3;
    State s;
    s.attitude = core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, tilt);
    Wrench w;
    w.thrust_n = 30.0;
    const StateDerivative d = sim::derivative(s, w, p);
    CHECK_NEAR(d.d_velocity.y, 15.0 * std::sin(tilt), 1e-12);
    CHECK_NEAR(d.d_velocity.z, 10.0 - 15.0 * std::cos(tilt), 1e-12);
}

void test_torque_gives_angular_acceleration() {
    const VehicleParams p = test_params();
    Wrench w;
    w.torque_nm = Vec3d{0.002, -0.004, 0.006};
    const StateDerivative d = sim::derivative(State{}, w, p);
    check_vec(d.d_omega, 0.2, -0.2, 0.2);
}

void test_gyroscopic_coupling() {
    const VehicleParams p = test_params();
    State s;
    s.omega = Vec3d{1.0, 2.0, 3.0};
    const StateDerivative d = sim::derivative(s, Wrench{}, p);
    check_vec(d.d_omega, -6.0, 3.0, -2.0 / 3.0);
}

void test_no_gyroscopic_term_on_principal_axis() {
    const VehicleParams p = test_params();
    State s;
    s.omega = Vec3d{0.0, 0.0, 5.0};
    const StateDerivative d = sim::derivative(s, Wrench{}, p);
    check_vec(d.d_omega, 0.0, 0.0, 0.0);
}

void test_attitude_derivative_identity() {
    State s;
    s.omega = Vec3d{0.0, 0.0, 2.0};
    const StateDerivative d = sim::derivative(s, Wrench{}, test_params());
    check_quat(d.d_attitude, 0.0, 0.0, 0.0, 1.0);
}

void test_attitude_derivative_general() {
    State s;
    s.attitude = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    s.omega = Vec3d{0.4, -1.2, 0.8};
    const StateDerivative d = sim::derivative(s, Wrench{}, test_params());
    const Quatd expected = s.attitude * Quatd{0.0, 0.4, -1.2, 0.8};
    check_quat(d.d_attitude, 0.5 * expected.w, 0.5 * expected.x, 0.5 * expected.y, 0.5 * expected.z);
    const Quatd wrong_order = Quatd{0.0, 0.4, -1.2, 0.8} * s.attitude;
    CHECK(std::fabs(d.d_attitude.x - 0.5 * wrong_order.x) > 1e-3);
}

void test_derivative_does_not_depend_on_position() {
    State a;
    State b;
    b.position = Vec3d{100.0, -50.0, -20.0};
    const VehicleParams p = test_params();
    Wrench w;
    w.thrust_n = 17.0;
    const StateDerivative da = sim::derivative(a, w, p);
    const StateDerivative db = sim::derivative(b, w, p);
    check_vec(da.d_velocity, db.d_velocity.x, db.d_velocity.y, db.d_velocity.z);
}

}

int main() {
    test_position_derivative_is_velocity();
    test_free_fall();
    test_hover_thrust_equals_weight();
    test_excess_thrust_accelerates_up();
    test_tilted_thrust_goes_sideways();
    test_tilt_keeps_vertical_component();
    test_torque_gives_angular_acceleration();
    test_gyroscopic_coupling();
    test_no_gyroscopic_term_on_principal_axis();
    test_attitude_derivative_identity();
    test_attitude_derivative_general();
    test_derivative_does_not_depend_on_position();
    return check::report();
}