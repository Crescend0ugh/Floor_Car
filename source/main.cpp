//
// Created by avsom on 10/4/2025.
//

#include "vector.h"
#include "math_utils.h"


int main() {
    maid::vector3<double> a(0,1,1);
    maid::vector3<double> b(1,0,1);


    std::cout << "a = " << a << "\n";
    std::cout << "b = " << b << "\n";
    std::cout << "a x b = " << (a ^ b) << "\n";
    std::cout << "a * b = " << (a * b) << "\n";
    std::cout << "|a| = " << a.length() << "\n";
    std::cout << "|b| = " << b.length() << "\n";
    std::cout << "a - b = " << a - b << "\n";
    std::cout << "a + b = " << a + b << "\n";
    std::cout << "a * 2 = " << a * 2 << "\n";
    std::cout << "a / 2 = " << a / 2 << "\n";
    std::cout << "a == b = " << std::boolalpha << (a == b) << "\n";
    std::cout << "a != b = " << std::boolalpha << (a != b) << "\n";
    std::cout << "a == a = " << std::boolalpha << (a == a) << "\n";

    maid::vector3<double> c(0, 0, 0);
    std::cout << c.normalize_safe();

    maid::spherical_to_cartesian(1.f, 1.f, 1.f);


    return 0;
}