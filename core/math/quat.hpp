#pragma once
#include "math/vec3.hpp"
#include <cmath>

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

} 
