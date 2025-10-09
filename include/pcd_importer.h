//
// Created by avsom on 10/6/2025.
//

#pragma once

//enum class data_type
//{
//    ascii,
//    binary,
//    binary_compressed
//};
//
//
//struct pcd_header
//{
//    version
//    fields
//    size
//    type
//    count
//    width
//    height
//    viewpoint
//    points
//    data
//};

void load_pcd_file()
{
    file_data         outData;
    std::stringstream contents;
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            std::cerr << "Couldn't find file at path: " << path << "\n";
            std::exit(3);
        }
        contents << ifs.rdbuf();
    }

    std::string lineBuffer;

    std::string token;

    while (std::getline(contents, lineBuffer)) {
        std::istringstream lineStream(lineBuffer);
        lineStream >> token;

        if (token == "v") {

            outData.positions.push_back({});
            auto &back = outData.positions.back();
            lineStream >> back.x >> back.y >> back.z >> back.w;
        }

        if (token == "vt") {
            outData.texture_coordinates.push_back({});
            auto &back = outData.texture_coordinates.back();
            lineStream >> back.u >> back.v >> back.w;
        }

        if (token == "vn") {
            outData.normals.push_back({});
            auto &back = outData.normals.back();
            lineStream >> back.x >> back.y >> back.z;
        }

        if (token == "f") {
            outData.faces.push_back({});
            auto &back = outData.faces.back();
            while (lineStream >> token) {
                auto &vert = back.vertices[back.vertex_count];


                std::istringstream splitter(token);

                std::getline(splitter, token, '/');
                vert.position_index = std::stoul(token);

                if (std::getline(splitter, token, '/')) {
                    if (!token.empty()) {
                        vert.texture_coordinates_index = std::stoul(token);
                    }
                }
                if (std::getline(splitter, token, '/')) {
                    vert.normals_index = std::stoul(token);
                }

                back.vertex_count += 1;
            }
        }
    }
}