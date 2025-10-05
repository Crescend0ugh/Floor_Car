//
// Created by avsom on 10/4/2025.
//

#pragma once

namespace maid
{
    template <typename T>
    struct vector3
    {

        constexpr vector3 operator^ (const vector3& other){};
        constexpr vector3 cross(const vector3& other){};
        constexpr T operator* (const vector3& other){};
        constexpr T dot(const vector3& other);
        constexpr vector3 operator* (T scalar){};
        constexpr vector3& operator*=(T scalar){};
        constexpr vector3 operator+ (const vector3& other){};
        constexpr vector3& operator+=(const vector3& other){};
        constexpr vector3 operator- (const vector3& other){};
        constexpr vector3& operator-=(const vector3& other){};
        constexpr vector3 operator==(const vector3& other){};
        constexpr vector3 operator!=(const vector3& other){};
        constexpr T       length();
        constexpr void    normalize();

        T x;
        T y;
        T z;
    };
}