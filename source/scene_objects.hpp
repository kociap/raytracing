#pragma once

#include <anton/array.hpp>
#include <build_config.hpp>
#include <handle.hpp>
#include <materials.hpp>

namespace raytracing {
    struct Sphere {
        f32 radius;
        Handle<Material> material;
    };

    struct Triangle {
        Vec3 v1;
        Vec3 v2;
        Vec3 v3;
        Handle<Material> material;
    };

    struct Mesh {
        Array<Triangle> triangles;
    };
} // namespace raytracing