#pragma once

#include <anton/array.hpp>
#include <primitives.hpp>

namespace raytracing {
    struct Scene {
        Array<Sphere> spheres;
        Array<Triangle> triangles;
    };
} // namespace raytracing
