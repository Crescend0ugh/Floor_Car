//
// Created by avsom on 10/4/2025.
//

#include "navmesh.h"

#include <iostream>

int main() {
    std::cout << "hello world!" << std::endl;

    agent_params params;
    params.cell_height = 0.2f;
    params.cell_size = 0.3f;
    params.height = 2.0f;
    params.radius = 0.6f;
    params.max_slope = 20.0f;

    navmesh* mesh = new navmesh(&params);
    navcontext* context = mesh->get_context();
    InputGeom* geometry = new InputGeom();

    geometry->load(context, "tests/meshes/dungeon.obj");

    context->log(RC_LOG_PROGRESS, "Hi\n");
    context->dump_log("Geometry log:");

    mesh->on_mesh_changed(geometry);

    if (mesh->build()) {
        std::cout << "Successfully built navmesh (lie)" << std::endl;
    }
    context->dump_log("Build log: ");

    return 0;
}