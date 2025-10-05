//
// Created by avsom on 10/4/2025.
//

#include "navmesh.h"
#include "vector.h"

#include <iostream>


int main() {
    maid::vector3<double> a(1,1,1);
    maid::vector3<double> b(1,1,1);

    std::cout << "ff" << std::endl;

    agent_params params;
    params.cell_height = 0.2f;
    params.cell_size = 0.3f;
    params.height = 2.0f;
    params.radius = 0.6f;
    params.max_slope = 20.0f;

    return 0;
}