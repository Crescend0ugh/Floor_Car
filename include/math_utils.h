//
// Created by Adithya Somashekhar on 10/5/25.
//

#pragma once
#include "vector.h"
#include <numbers>

namespace maid
{


    template<typename T>
    static constexpr vector3<T> spherical_to_cartesian(T p, T theta, T phi)
    {
        return vector3<T>{p*sin(phi)*cos(theta), p*sin(phi)*sin(theta), p*cos(phi)};
    }


    template<typename T>
    static constexpr T degrees_to_radians(T theta)
    {
        return {theta * std::numbers::pi / static_cast<T>(180)};
    }

    template<typename T>
    static constexpr T radians_to_degrees(T rads)
    {
        return {rads * static_cast<T>(180) * std::numbers::inv_pi_v<T>};
    }

}