#include "navmesh.h"
#include "path.h"

#include <iostream>

int main() {
    agent_params params;
    params.cell_height = 0.2f;
    params.cell_size = 0.3f;
    params.height = 1.0f;
    params.radius = 0.5f;
    params.max_slope = 20.0f;

    navmesh* mesh = new navmesh(&params);
    navcontext* context = mesh->get_context();
    InputGeom* geometry = new InputGeom();

    geometry->load(context, "../../../../../tests/meshes/dungeon.obj");
    context->dump_log("Geometry log:");

    mesh->on_mesh_changed(geometry);
    mesh->build();

    context->dump_log("Build log: ");

    ////////////////////////////////////////

    path* p = new path();
    p->init(mesh);

    float start[3] = { 20.230976f, 9.998184f, 1.481956f };
    p->set_start(start);

    float end[3] = { -1.292282f, 9.998180f, -5.186989f };
    p->set_end(end);

    fprintf(stdout, "path points: %d", p->get_waypoint_count());
    for (int i = 0; i < p->get_waypoint_count(); ++i) {
        const float* next_position = p->get_next_waypoint();
        fprintf(stdout, "position %d: [%f, %f, %f]", i, next_position[0], next_position[1], next_position[2]);
        p->increment_waypoint();
    }

	return 0;
}