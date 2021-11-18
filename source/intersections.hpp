#pragma once

#include <build_config.hpp>
#include <handle.hpp>
#include <materials.hpp>
#include <primitives.hpp>

namespace raytracing {
    struct Surface_Interaction {
        Vec3 normal;
        f32 distance = math::infinity;
        Handle<Material> material;
    };

    [[nodiscard]] Optional<Surface_Interaction> intersect_sphere(Ray ray, Sphere sphere);
    [[nodiscard]] Optional<Surface_Interaction> intersect_triangle(Ray ray, Triangle triangle);
} // namespace raytracing
