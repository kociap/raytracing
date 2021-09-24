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

    // refract
    //
    // Parameters:
    // eta - ratio of indices of refraction.
    //
    static Vec3 refract(Vec3 const incident, Vec3 const normal, f32 const eta) {
        f32 const cos_theta = math::dot(incident, normal);
        f32 const k = 1.0f - eta * eta * (1.0f - cos_theta * cos_theta);
        if(k >= 0.0f) {
            return eta * incident - (eta * cos_theta + math::sqrt(k)) * normal;
        } else {
            return Vec3{0.0f};
        }
    }

    Optional<Scatter_Result> scatter(Random_Engine* const random_engine, Ray incident_ray, f32 distance, Vec3 normal, Handle<Material> const& handle) {
        Material const& material = get_material(handle);
        Vec3 const incident_point = incident_ray.origin + incident_ray.direction * distance;
        if(material.transmissive) {
            // Transmissive
            f32 const cos_theta_incident = math::dot(incident_ray.direction, normal);
            bool const front_facing = cos_theta_incident < 0.0f;
            f32 const ior_ratio = front_facing ? 1.0f / material.ior : material.ior;
            f32 const sin_theta_incident = math::sqrt(1.0f - cos_theta_incident * cos_theta_incident);
            if(ior_ratio * sin_theta_incident > 1.0f) {
                // Total Internal Reflection
                Vec3 const reflected = reflect(incident_ray.direction, normal);
                return Scatter_Result{Ray{incident_point, reflected}, material.albedo};
            } else {
                Vec3 const refracted = refract(incident_ray.direction, normal, ior_ratio);
                return Scatter_Result{Ray{incident_point, refracted}, material.albedo};
            }
        } else if(material.metallic) {
            // Metallic reflection
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
