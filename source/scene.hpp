#pragma once

#include <anton/array.hpp>
#include <scene_objects.hpp>
#include <transform.hpp>

namespace raytracing {
    struct Scene {
        Array<Transform> sphere_transforms;
        Array<Sphere> spheres;
        Array<Transform> mesh_transforms;
        Array<Mesh> meshes;
    };
} // namespace raytracing
