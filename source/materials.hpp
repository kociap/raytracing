#pragma once

#include <anton/math/primitives.hpp>
#include <anton/math/vec3.hpp>
#include <anton/optional.hpp>
#include <build_config.hpp>
#include <handle.hpp>
#include <random_engine.hpp>

namespace raytracing {
    struct Material {
        Vec3 albedo;
        bool metallic = false;
        // The roughness of a metallic surface
        f32 roughness = 0.0f;
    };

    [[nodiscard]] Handle<Material> create_material(Material const& material);
    [[nodiscard]] Material const& get_material(Handle<Material> const& handle);

    struct Scatter_Result {
        // Scattered ray
        Ray ray;
        // Attenuation
        Vec3 attenuation;
    };

    [[nodiscard]] Optional<Scatter_Result> scatter(Random_Engine* random_engine, Ray incident_ray, f32 distance, Vec3 normal, Handle<Material> const& material);
} // namespace raytracing
