//
// Created by avsom on 10/6/2025.
//

#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include "vector.h"
#include <sstream>



std::vector<robo::vector3d> load_points(const std::string& path)
{

    std::vector<robo::vector3d> positions;
    std::stringstream contents;
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            std::cerr << "Couldn't find file at path: " << path << "\n";
            std::exit(3);
        }
        contents << ifs.rdbuf();
    }

    double rgb;
    robo::vector3d vec;
    while (contents >> vec.x >> vec.y >> vec.z >> rgb)
    {
        positions.push_back(vec);
    }
    return positions;
}