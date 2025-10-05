//
// Created by avsom on 10/4/2025.
//

#pragma once

#include <iostream>
#include <sstream>
#include <format>

namespace maid
{
    template<typename T>
    struct vector3 {

        constexpr vector3(T x, T y, T z) : x(x), y(y), z(z)
        {}

        constexpr vector3() = default;

        constexpr vector3 operator^(const vector3 &other) const
        {
            return vector3{y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x};
        };

        //TODO since the '^' operator has lower precedence than '<<' we get an error without the parenthesis when we try to print vec ^ vec
        //TODO I could change the operator symbol to % or just use the .cross() method
        constexpr vector3 cross(const vector3 &other) const
        {
            return *this ^ other;
        };

        constexpr T operator*(const vector3 &other) const
        {
            return x*other.x + y*other.y + z*other.z;
        };

        constexpr T dot(const vector3 &other) const
        {
            return *this * other;
        };

        constexpr vector3 operator*(T scalar) const
        {
            vector3<T> product = *this;
            product *= scalar;
            return product;
        };

        constexpr vector3 operator/=(T scalar)
        {
            x/= scalar;
            y/= scalar;
            z/= scalar;
            return *this;
        }
        constexpr vector3 operator/(T scalar) const
        {
            vector3<T> product = *this;
            product /= scalar;
            return product;
        }

        constexpr vector3 &operator*=(T scalar)
        {
            x*= scalar;
            y*= scalar;
            z*= scalar;
            return *this;
        };

        constexpr vector3 operator+(const vector3 &other) const
        {
            vector3<T> sum = *this;
            sum += other;
            return sum;
        };

        constexpr vector3 &operator+=(const vector3 &other)
        {
            x+=other.x;
            y+=other.y;
            z+=other.z;
            return *this;
        };

        constexpr vector3 operator-(const vector3 &other) const
        {
            vector3<T> difference = *this;
            difference -= other;
            return difference;
        };

        constexpr vector3 &operator-=(const vector3 &other)
        {
            x-=other.x;
            y-=other.y;
            z-=other.z;
            return *this;
        };

        constexpr bool operator==(const vector3 &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        };

        constexpr bool operator!=(const vector3 &other) const
        {
            return !(*this==other);
        };

        constexpr T length() const
        {
            return sqrt(x*x + y*y + z*z);
        };

        constexpr vector3& normalize()
        {
            *this /= length();
            return *this;
        };

        //TODO this should also have a tolerance/epsilon parameter
        constexpr vector3& normalize_safe()
        {
            return zero() ? *this : normalize();
        };

        constexpr vector3& normalize_safe(bool& normalized)
        {
            return (normalized = !zero()) ? normalize() : *this;
        };

        //TODO maybe we should add a tolerance/epsilon parameter
        constexpr bool zero() const
        {
            return x+y+z == 0;
        };

        friend std::ostream &operator<<(std::ostream &os, const vector3<T> rhs)
        {
            os << std::format("({}, {}, {})", rhs.x, rhs.y, rhs.z);
            return os;
        }

        T x;
        T y;
        T z;
    };

    using vector3f = vector3<float>;
    using vector3d = vector3<double>;
    using vector3i = vector3<int>;
}

