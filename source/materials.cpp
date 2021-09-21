#include <anton/array.hpp>
#include <anton/assert.hpp>
#include <materials.hpp>

namespace raytracing {
    static Array<Material> materials;

    Handle<Material> create_material(Material const& material) {
        i64 const index = materials.size();
        materials.push_back(material);
        return {index};
    }

    Material const& get_material(Handle<Material> const& handle) {
        ANTON_ASSERT(handle.value < materials.size(), "invalid material handle");
        return materials[handle.value];
    }

    static Vec3 reflect(Vec3 const incident, Vec3 const normal) {
        return incident - 2.0f * math::dot(normal, incident) * normal;
    }

    Optional<Scatter_Result> scatter(Random_Engine* const random_engine, Ray incident_ray, f32 distance, Vec3 normal, Handle<Material> const& handle) {
        Material const& material = get_material(handle);
        Vec3 const incident_point = incident_ray.origin + incident_ray.direction * distance;
        if(material.metallic) {
            Vec3 const reflected = reflect(incident_ray.direction, normal);
            Vec3 const roughness = material.roughness * random_unit_vec3(random_engine);
            if(math::dot(reflected + roughness, normal) > 0) {
                Vec3 const rough_reflected = math::normalize(reflected + roughness);
                return Scatter_Result{Ray{incident_point, rough_reflected}, material.albedo};
            } else {
                Vec3 const rough_reflected = math::normalize(reflected - roughness);
                return Scatter_Result{Ray{incident_point, rough_reflected}, material.albedo};
            }
        } else {
            // Lambertian scatter
            Vec3 const unit_vec = random_unit_vec3(random_engine);
            Vec3 scatter_direction = math::normalize(unit_vec + normal);
            if(math::is_almost_zero(scatter_direction)) {
                scatter_direction = normal;
            }

            return Scatter_Result{Ray{incident_point, scatter_direction}, material.albedo};
        }
    }
} // namespace raytracing
