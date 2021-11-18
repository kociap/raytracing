#include <intersections.hpp>

namespace raytracing {
    Optional<Surface_Interaction> intersect_sphere(Ray const ray, Sphere const sphere) {
        Vec3 const ray_origin = ray.origin - sphere.position;
        // a = dot(ray.direction, ray.direction) which is always 1
        f32 const b = 2.0f * dot(ray_origin, ray.direction);
        f32 const c = dot(ray_origin, ray_origin) - sphere.radius * sphere.radius;
        f32 const delta = b * b - 4.0f * c;
        if(delta < 0.0f) {
            return null_optional;
        }
        f32 const sqrt_delta = 0.5 * math::sqrt(delta);
        f32 const half_b = -0.5 * b;
        f32 distance = half_b - sqrt_delta;
        if(distance < 0.001f) {
            distance = half_b + sqrt_delta;
            if(distance < 0.001f) {
                return null_optional;
            }
        }

        Vec3 normal = ray.origin + ray.direction * distance - sphere.position;
        return Surface_Interaction{normal, distance, sphere.material};
    }

    [[nodiscard]] static Optional<f32> intersect_plane(Ray const ray, Vec3 const plane_normal, f32 const plane_distance) {
        f32 const angle_cos = dot(ray.direction, plane_normal);
        f32 const coeff = (plane_distance - dot(ray.origin, plane_normal)) / angle_cos;
        // TODO: Shift the distance >= 0.001f check here and remove it in the routines higher up.
        if(math::abs(angle_cos) > math::epsilon && coeff >= 0.001f) {
            return coeff;
        } else {
            return null_optional;
        }
    }

    [[nodiscard]] Optional<Surface_Interaction> intersect_triangle(Ray const ray, Triangle const triangle) {
        Vec3 const u_vec = triangle.v1 - triangle.v2;
        Vec3 const v_vec = triangle.v3 - triangle.v2;
        Vec3 const plane_normal_unnormalized = math::cross(v_vec, u_vec);
        Vec3 const plane_normal = math::normalize(plane_normal_unnormalized);
        f32 const plane_distance = math::dot(triangle.v2, plane_normal);
        Optional<f32> const distance = intersect_plane(ray, plane_normal, plane_distance);
        if(!distance) {
            return null_optional;
        }

        Vec3 const pr = distance.value() * ray.direction;
        // dot(pr, cross(bc, bc))
        // The cross product is the plane normal in CCW. PR points the opposite way.
        // det is negative when ABC is CCW, positive when ABC is CW.
        f32 const det = math::dot(pr, plane_normal_unnormalized);
        Vec3 const pa = triangle.v1 - ray.origin;
        Vec3 const pb = triangle.v2 - ray.origin;
        Vec3 const pc = triangle.v3 - ray.origin;
        // When ABC is CCW, u and v are positive for R in ABC, negative for R outside ABC.
        // When ABC is CW, u and v are negative for R in ABC, positive for R outside ABC.
        // We divide by -det to normalize them and ensure they are always positive when R is inside ABC.
        f32 const u = math::dot(pr, math::cross(pa, pc)) / -det;
        f32 const v = math::dot(pr, math::cross(pc, pb)) / -det;
        if(u >= 0.0f & v >= 0.0f & u + v <= 1.0f) {
            return Surface_Interaction{plane_normal, distance.value(), triangle.material};
        } else {
            return null_optional;
        }
    }
} // namespace raytracing
