
#pragma once

#include <cmath>

namespace core {

template <typename T>
struct Vec3 {
    T x{}, y{}, z{};
};

using Vec3d = Vec3<double>;
using Vec3f = Vec3<float>;

template <typename T>
constexpr Vec3<T> operator+(Vec3<T> a, Vec3<T> b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

template <typename T>
constexpr Vec3<T> operator-(Vec3<T> a, Vec3<T> b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

template <typename T>
constexpr Vec3<T> operator-(Vec3<T> a) {
    return {-a.x, -a.y, -a.z};
}

template <typename T>
constexpr Vec3<T> operator*(Vec3<T> a, T s) {
    return {a.x * s, a.y * s, a.z * s};
}

template <typename T>
constexpr Vec3<T> operator*(T s, Vec3<T> a) {
    return a * s;
}

template <typename T>
constexpr Vec3<T> operator/(Vec3<T> a, T s) {
    return {a.x / s, a.y / s, a.z / s};
}

template <typename T>
constexpr Vec3<T>& operator+=(Vec3<T>& a, Vec3<T> b) {
    a = a + b;
    return a;
}

template <typename T>
constexpr Vec3<T>& operator-=(Vec3<T>& a, Vec3<T> b) {
    a = a - b;
    return a;
}

template <typename T>
constexpr Vec3<T>& operator*=(Vec3<T>& a, T s) {
    a = a * s;
    return a;
}

template <typename T>
constexpr Vec3<T>& operator/=(Vec3<T>& a, T s) {
    a = a / s;
    return a;
}

template <typename T>
constexpr T dot(Vec3<T> a, Vec3<T> b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename T>
constexpr Vec3<T> cross(Vec3<T> a, Vec3<T> b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

template <typename T>
constexpr T norm_sq(Vec3<T> a) {
    return dot(a, a);
}

template <typename T>
T norm(Vec3<T> a) {
    return static_cast<T>(std::sqrt(norm_sq(a)));
}

template <typename T>
Vec3<T> normalized(Vec3<T> a) {
    const T n = norm(a);

    if (n < T(1e-9)) {
        return {};
    }

    return a / n;
}

}
