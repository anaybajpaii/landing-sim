#pragma once
#include "math/vec3.hpp"
#include <cmath>
#include <algorithm>

namespace core {

template <typename T>
struct Quat {
    T w{1}, x{}, y{}, z{};
};

using Quatd = Quat<double>;
using Quatf = Quat<float>;

template <typename T>
constexpr Quat<T> operator*(Quat<T> a, Quat<T> b) {
    Vec3<T> u{a.x, a.y, a.z};
    Vec3<T> v{b.x, b.y, b.z};

    T w = a.w * b.w - dot(u, v);

    Vec3<T> vec =
        u * b.w +
        v * a.w +
        cross(u, v);

    return {w, vec.x, vec.y, vec.z};
}

template <typename T>
constexpr Quat<T> conjugate(Quat<T> q) {
    return {q.w, -q.x, -q.y, -q.z};
}

template <typename T>
T norm(Quat<T> q) {
    return std::sqrt(
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z
    );
}

template <typename T>
Quat<T> normalized(Quat<T> q) {
    T n = norm(q);

    if (n < T(1e-9)) {
        return {};
    }

    return {
        q.w / n,
        q.x / n,
        q.y / n,
        q.z / n
    };
}

template <typename T>
constexpr Vec3<T> rotate(Quat<T> q, Vec3<T> v) {
    Vec3<T> u{q.x, q.y, q.z};

    Vec3<T> t = cross(u, v) * T(2);

    return v + t * q.w + cross(u, t);
}

template <typename T>
Quat<T> from_axis_angle(Vec3<T> axis, T angle) {
    if (norm(axis) < T(1e-9)) {
        return {};
    }

    axis = normalized(axis);

    T half_angle = angle * T(0.5);
    T s = std::sin(half_angle);

    return Quat<T>{
        std::cos(half_angle),
        axis.x * s,
        axis.y * s,
        axis.z * s
    };
}

template <typename T>
Quat<T> integrate(Quat<T> q, Vec3<T> omega, T dt) {
    T angle = norm(omega) * dt;

    Quat<T> delta = from_axis_angle(omega, angle);

    return normalized(q * delta);
}

template <typename T>
Vec3<T> euler_zyx(Quat<T> q) {
    q = normalized(q);

    T roll = std::atan2(
        T(2) * (q.w * q.x + q.y * q.z),
        T(1) - T(2) * (q.x * q.x + q.y * q.y)
    );

    T sin_pitch = T(2) * (q.w * q.y - q.z * q.x);

    T pitch = std::asin(
        std::clamp(sin_pitch, T(-1), T(1))
    );

    T yaw = std::atan2(
        T(2) * (q.w * q.z + q.x * q.y),
        T(1) - T(2) * (q.y * q.y + q.z * q.z)
    );

    return {roll, pitch, yaw};
}

template <typename T>
Quat<T> from_euler_zyx(T roll, T pitch, T yaw) {
    T cr = std::cos(roll * T(0.5));
    T sr = std::sin(roll * T(0.5));

    T cp = std::cos(pitch * T(0.5));
    T sp = std::sin(pitch * T(0.5));

    T cy = std::cos(yaw * T(0.5));
    T sy = std::sin(yaw * T(0.5));

    return normalized(Quat<T>{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    });
}

}