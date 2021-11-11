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

    struct Mesh {
        Array<Vec3> vertices;
        Handle<Material> material;
    };
} // namespace raytracing