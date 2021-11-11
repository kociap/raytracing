#include <anton/array.hpp>
#include <anton/console.hpp>
#include <anton/filesystem.hpp>
#include <anton/format.hpp>
#include <anton/import.hpp>
#include <anton/intrinsics.hpp>
#include <anton/iterators/range.hpp>
#include <anton/iterators/zip.hpp>
#include <anton/math/math.hpp>
#include <anton/optional.hpp>
#include <anton/slice.hpp>
#include <build_config.hpp>
#include <camera.hpp>
#include <filesystem.hpp>
#include <materials.hpp>
#include <random_engine.hpp>
#include <scene.hpp>

namespace raytracing {
    struct Context {
        Random_Engine* random_engine = nullptr;
        i64 bounces = 0;
        i64 samples = 0;
    };

    struct Raycast_Result {
        Vec3 normal;
        f32 distance;
        Handle<Material> material;
    };

    [[nodiscard]] static f32 intersect_sphere(Sphere const& sphere, Transform const& transform, Ray const ray) {
        Vec3 const ray_origin = ray.origin - transform.position;
        // a = dot(ray.direction, ray.direction) which is always 1
        f32 const b = 2.0f * dot(ray_origin, ray.direction);
        f32 const c = dot(ray_origin, ray_origin) - sphere.radius * sphere.radius;
        f32 const delta = b * b - 4.0f * c;
        if(delta < 0.0f) {
            return -1.0f;
        } else {
            return 0.5f * (-b - math::sqrt(delta));
        }
    }

    [[nodiscard]] static Optional<f32> intersect_plane(Ray const ray, Vec3 const plane_normal, f32 const plane_distance) {
        f32 const angle_cos = dot(ray.direction, plane_normal);
        f32 const coeff = (plane_distance - dot(ray.origin, plane_normal)) / angle_cos;
        // TODO: Shift the distance > 0.001f check here and remove it in the routines higher up.
        if(math::abs(angle_cos) > math::epsilon && coeff >= 0.0f) {
            return coeff;
        } else {
            return null_optional;
        }
    }

    [[nodiscard]] static Optional<Raycast_Result> intersect_triangle(Vec3 const a, Vec3 const b, Vec3 const c, Ray const ray) {
        Vec3 const u_vec = a - b;
        Vec3 const v_vec = c - b;
        Vec3 const plane_normal_unnormalized = math::cross(v_vec, u_vec);
        Vec3 const plane_normal = math::normalize(plane_normal_unnormalized);
        f32 const plane_distance = math::dot(b, plane_normal);
        Optional<f32> const distance = intersect_plane(ray, plane_normal, plane_distance);
        if(!distance) {
            return null_optional;
        }

        Vec3 const pr = distance.value() * ray.direction;
        // dot(pr, cross(bc, bc))
        // The cross product is the plane normal in CCW. PR points the opposite way.
        // det is negative when ABC is CCW, positive when ABC is CW.
        f32 const det = math::dot(pr, plane_normal_unnormalized);
        Vec3 const pa = a - ray.origin;
        Vec3 const pb = b - ray.origin;
        Vec3 const pc = c - ray.origin;
        // When ABC is CCW, u and v are positive for R in ABC, negative for R outside ABC.
        // When ABC is CW, u and v are negative for R in ABC, positive for R outside ABC.
        // We divide by -det to normalize them and ensure they are always positive when R is inside ABC.
        f32 const u = math::dot(pr, math::cross(pa, pc)) / -det;
        f32 const v = math::dot(pr, math::cross(pc, pb)) / -det;
        if(u >= 0.0f & v >= 0.0f & u + v <= 1.0f) {
            return Raycast_Result{plane_normal, distance.value(), Handle<Material>{}};
        } else {
            return null_optional;
        }
    }

    [[nodiscard]] static Optional<Raycast_Result> intersect_mesh(Mesh const& mesh, Transform const& transform, Ray const ray) {
        // Translate ray to local space instead of the entire mesh to world space.
        Ray const translated_ray{ray.origin - transform.position, ray.direction};
        Raycast_Result result{Vec3{0.0f}, math::infinity, {}};
        bool hit;
        for(Triangle const& triangle: mesh.triangles) {
            // Optional<Raycast_Result> intersection_result = intersect_triangle(v1, v2, v3, translated_ray);
            Optional<Raycast_Result> intersection_result = intersect_triangle(triangle.v1, triangle.v2, triangle.v3, ray);
            if(intersection_result && intersection_result->distance > 0.001f && intersection_result->distance < result.distance) {
                result = intersection_result.value();
                result.material = triangle.material;
                hit = true;
            }
        }

        if(hit) {
            return result;
        } else {
            return null_optional;
        }
    }

    [[nodiscard]] static Optional<Raycast_Result> intersect_scene(Scene const& scene, Ray const ray) {
        bool hit = false;
        Raycast_Result result{Vec3{0.0f}, math::infinity, {}};
        // Intersect spheres in the scene.
        {
            Zip_Iterator begin{scene.spheres.begin(), scene.sphere_transforms.begin()};
            Zip_Iterator end{scene.spheres.end(), scene.sphere_transforms.end()};
            for(auto [sphere, transform]: Range(ANTON_MOV(begin), ANTON_MOV(end))) {
                f32 const intersection_result = intersect_sphere(sphere, transform, ray);
                if(intersection_result > 0.001f && intersection_result < result.distance) {
                    result.distance = intersection_result;
                    result.normal = ray.origin + ray.direction * intersection_result - transform.position;
                    result.material = sphere.material;
                    hit = true;
                }
            }
        }
        // Instersect meshes in the scene.
        {
            Zip_Iterator begin{scene.meshes.begin(), scene.mesh_transforms.begin()};
            Zip_Iterator end{scene.meshes.end(), scene.mesh_transforms.end()};
            for(auto [mesh, transform]: Range(ANTON_MOV(begin), ANTON_MOV(end))) {
                Optional<Raycast_Result> const intersection_result = intersect_mesh(mesh, transform, ray);
                if(intersection_result && intersection_result->distance > 0.001f && intersection_result->distance < result.distance) {
                    result = intersection_result.value();
                    hit = true;
                }
            }
        }

        if(hit) {
            return result;
        } else {
            return anton::null_optional;
        }
    }

    static Vec3 cast_ray(Context const& ctx, Scene const& scene, Ray const ray, i64 const bounce) {
        if(bounce >= ctx.bounces) {
            return Vec3{0.0f};
        }

        Optional<Raycast_Result> const result = intersect_scene(scene, ray);
        if(result) {
            Optional<Scatter_Result> scatter_result = scatter(ctx.random_engine, ray, result->distance, result->normal, result->material);
            if(scatter_result) {
                Vec3 const color = cast_ray(ctx, scene, scatter_result->ray, bounce + 1);
                return scatter_result->attenuation * color;
            } else {
                return Vec3{0.0f};
            }
        }

        // Sky gradient
        f32 const t = 0.5f * (ray.direction.y + 1.0f);
        return (1.0f - t) * Vec3{1.0f} + t * Vec3{0.5f, 0.7f, 1.0f};
    }

    [[nodiscard]] static Array<Vec3> render_scene(Context const& ctx, Scene const& scene, Camera const& camera, Transform const& camera_transform,
                                                  Camera_Target const& target) {
        // TODO: The lookat code does not correctly handle camera target being positioned exactly above the camera.
        Vec3 const camera_view = math::normalize(target.position - camera_transform.position);
        Vec3 const camera_right = math::normalize(math::cross(camera_view, Vec3{0.0f, 1.0f, 0.0f}));
        Vec3 const camera_up = math::cross(camera_right, camera_view);
        Mat3 const viewport_rotation{camera_right, camera_up, camera_view};
        Vec3 const viewport_top_left = viewport_rotation * Vec3{-0.5f * camera.viewport_width, 0.5f * camera.viewport_height, camera.focal_length};

        Array<Vec3> pixels{reserve, camera.image_width * camera.image_height};
        i64 const samples_root = math::sqrt(ctx.samples);
        for(i64 y = 0; y < camera.image_height; ++y) {
            for(i64 x = 0; x < camera.image_width; ++x) {
                Vec3& pixel = pixels.push_back(Vec3{0.0f});
                for(i64 sample = 0; sample < samples_root * samples_root; ++sample) {
                    f32 const u = (static_cast<f32>(x) + static_cast<f32>(sample % samples_root) / samples_root) / (camera.image_width - 1);
                    f32 const v = (static_cast<f32>(y) + static_cast<f32>(sample / samples_root) / samples_root) / (camera.image_height - 1);
                    Ray const ray{camera_transform.position,
                                  math::normalize(viewport_top_left + u * camera.viewport_width * camera_right - v * camera.viewport_height * camera_up)};
                    Vec3 const color = cast_ray(ctx, scene, ray, 0);
                    pixel += color;
                }
                pixel /= samples_root * samples_root;
                pixel.x = math::sqrt(pixel.x);
                pixel.y = math::sqrt(pixel.y);
                pixel.z = math::sqrt(pixel.z);
            }
        }
        return pixels;
    }

    static int entry() {
        Context ctx;
        ctx.random_engine = create_random_engine(7849034);
        ctx.bounces = 8;
        ctx.samples = 16;

        Camera camera{90.0f, 16.0f / 9.0f, 720};
        Transform camera_transform{Vec3{2.0f, 2.0f, 5.0f}};
        Camera_Target target{Vec3{0.0f, 0.0f, 0.0f}};

        Material green_diffuse{Vec3{0.8f, 0.8f, 0.0f}};
        Handle<Material> green_diffuse_handle = create_material(green_diffuse);
        Material glass{Vec3{1.0f, 1.0f, 1.0f}, false, 0.0f, true, 1.4f};
        Handle<Material> glass_handle = create_material(glass);
        Material red_metallic{Vec3{0.8f, 0.0f, 0.0f}, true, 0.0f};
        Handle<Material> red_metallic_handle = create_material(red_metallic);
        Material green_metallic{Vec3{0.8f, 0.8f, 0.0f}, true, 0.5f};
        Handle<Material> green_metallic_handle = create_material(green_metallic);
        Material grey_diffuse{Vec3{0.4f, 0.4f, 0.4f}};
        Handle<Material> grey_diffuse_handle = create_material(grey_diffuse);

        // Import cube.
        Console_Output cout;
        Expected<Array<u8>, String> file_read_result = read_file("./assets/cube.obj");
        if(!file_read_result) {
            cout.write(file_read_result.error());
            return -1;
        }
        Expected<Array<anton::Mesh>, String> import_result = import_obj(file_read_result.value(), {});
        if(!import_result) {
            cout.write(import_result.error());
            return -1;
        }

        Random_Engine* rnd = create_random_engine(100478823);
        Scene scene;
        for(anton::Mesh const& mesh: import_result.value()) {
            cout.write(format("Adding mesh {} (indices: {})\n"_sv, mesh.name, mesh.indices.size()));
            Mesh& scene_mesh = scene.meshes.emplace_back();
            for(i64 i = 0; i < mesh.indices.size(); i += 3) {
                Vec3 const v1 = mesh.vertices[mesh.indices[i]];
                Vec3 const v2 = mesh.vertices[mesh.indices[i + 1]];
                Vec3 const v3 = mesh.vertices[mesh.indices[i + 2]];
                Triangle triangle{v1, v2, v3, grey_diffuse_handle};
                scene_mesh.triangles.push_back(ANTON_MOV(triangle));
            }
            // f32 const x = random_f32(rnd, -2.0f, 2.0f);
            // f32 const y = random_f32(rnd, -2.0f, 2.0f);
            // f32 const z = random_f32(rnd, -2.0f, 2.0f);
            // scene.mesh_transforms.push_back(Transform{Vec3{x, y, z}});
            scene.mesh_transforms.push_back(Transform{Vec3{0.0f}});
        }
        // Add spheres.
        // scene.spheres.push_back(Sphere{1.0f, red_metallic_handle});
        // scene.sphere_transforms.push_back(Transform{Vec3{-2.0f, 0.0f, -5.0f}});
        // scene.spheres.push_back(Sphere{1.0f, green_metallic_handle});
        // scene.sphere_transforms.push_back(Transform{Vec3{2.0f, 0.0f, -5.0f}});
        // scene.spheres.push_back(Sphere{1.0f, green_diffuse_handle});
        // scene.sphere_transforms.push_back(Transform{Vec3{0.0f, 0.0f, -5.0f}});
        // scene.spheres.push_back(Sphere{0.5f, glass_handle});
        // scene.sphere_transforms.push_back(Transform{Vec3{-1.0f, -0.5f, -3.0f}});
        scene.spheres.push_back(Sphere{200.0f, green_diffuse_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{0.0f, -201.0f, -3.0f}});

        Array<Vec3> const pixels = render_scene(ctx, scene, camera, camera_transform, target);

        fs::Output_File_Stream stream("img.ppm"_s);
        write_ppm_file(stream, pixels, camera.image_width, camera.image_height);
        return 0;
    }
} // namespace raytracing

int main() {
    return raytracing::entry();
}
