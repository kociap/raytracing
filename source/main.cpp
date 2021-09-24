#include <anton/array.hpp>
#include <anton/filesystem.hpp>
#include <anton/format.hpp>
#include <anton/iterators/range.hpp>
#include <anton/iterators/zip.hpp>
#include <anton/math/math.hpp>
#include <anton/optional.hpp>
#include <anton/slice.hpp>
#include <build_config.hpp>
#include <camera.hpp>
#include <materials.hpp>
#include <random_engine.hpp>

namespace raytracing {
    struct Context {
        Random_Engine* random_engine = nullptr;
        i64 bounces = 0;
        i64 samples = 0;
    };

    struct Transform {
        Vec3 position;
    };

    struct Sphere {
        f32 radius;
        Handle<Material> material;
    };

    static void write_ppm_file(Output_Stream& stream, Slice<Vec3 const> const pixels, i64 const width, i64 const height) {
        String header = format("P3\n{} {}\n255\n"_sv, width, height);
        stream.write(header);
        for(Vec3 const pixel: pixels) {
            i64 const r = static_cast<i64>(255.999f * pixel.r);
            i64 const g = static_cast<i64>(255.999f * pixel.g);
            i64 const b = static_cast<i64>(255.999f * pixel.b);
            String value = format("{} {} {}\n"_sv, r, g, b);
            stream.write(value);
        }
    }

    struct Scene {
        Array<Transform> sphere_transforms;
        Array<Sphere> spheres;
    };

    static f32 intersect_sphere(Sphere const& sphere, Transform const& transform, Ray const ray) {
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

    struct Raycast_Result {
        Vec3 normal;
        f32 distance;
        Handle<Material> material;
    };

    static Optional<Raycast_Result> intersect_scene(Scene const& scene, Ray const ray) {
        f32 closest = math::infinity;
        Vec3 normal;
        Handle<Material> material;
        bool hit = false;
        Zip_Iterator begin{scene.spheres.begin(), scene.sphere_transforms.begin()};
        Zip_Iterator end{scene.spheres.end(), scene.sphere_transforms.end()};
        for(auto [sphere, transform]: Range(ANTON_MOV(begin), ANTON_MOV(end))) {
            f32 const result = intersect_sphere(sphere, transform, ray);
            if(result > 0.001f && result < closest) {
                closest = result;
                normal = ray.origin + ray.direction * result - transform.position;
                material = sphere.material;
                hit = true;
            }
        }

        if(hit) {
            Raycast_Result raycast_result;
            raycast_result.distance = closest;
            raycast_result.normal = math::normalize(normal);
            raycast_result.material = material;
            return raycast_result;
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

    static int entry() {
        Context ctx;
        ctx.random_engine = create_random_engine(7849034);
        ctx.bounces = 8;
        ctx.samples = 16;

        Camera camera{90.0f, 16.0f / 9.0f, 720};
        Transform camera_transform{Vec3{0.0f, 0.0f, 0.0f}};

        Material green_diffuse{Vec3{0.8f, 0.8f, 0.0f}};
        Handle<Material> green_diffuse_handle = create_material(green_diffuse);
        Material glass{Vec3{1.0f, 1.0f, 1.0f}, false, 0.0f, true, 1.4f};
        Handle<Material> glass_handle = create_material(glass);
        Material red_metallic{Vec3{0.8f, 0.0f, 0.0f}, true, 0.0f};
        Handle<Material> red_metallic_handle = create_material(red_metallic);
        Material green_metallic{Vec3{0.8f, 0.8f, 0.0f}, true, 0.5f};
        Handle<Material> green_metallic_handle = create_material(green_metallic);

        Scene scene;
        scene.spheres.push_back(Sphere{1.0f, red_metallic_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{-2.0f, 0.0f, -5.0f}});
        scene.spheres.push_back(Sphere{1.0f, green_metallic_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{2.0f, 0.0f, -5.0f}});
        scene.spheres.push_back(Sphere{1.0f, green_diffuse_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{0.0f, 0.0f, -5.0f}});
        scene.spheres.push_back(Sphere{0.5f, glass_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{1.0f, -0.5f, -3.0f}});
        scene.spheres.push_back(Sphere{200.0f, green_diffuse_handle});
        scene.sphere_transforms.push_back(Transform{Vec3{0.0f, -201.0f, -3.0f}});

        Array<Vec3> pixels{reserve, camera.image_width * camera.image_height};
        Vec3 const viewport_top_left = camera_transform.position + Vec3{-0.5f * camera.viewport_width, 0.5f * camera.viewport_height, -camera.focal_length};
        i64 const samples_root = math::sqrt(ctx.samples);
        for(i64 y = 0; y < camera.image_height; ++y) {
            for(i64 x = 0; x < camera.image_width; ++x) {
                Vec3& pixel = pixels.push_back(Vec3{0.0f});
                for(i64 sample = 0; sample < samples_root * samples_root; ++sample) {
                    f32 const u = (static_cast<f32>(x) + static_cast<f32>(sample % samples_root) / samples_root) / (camera.image_width - 1);
                    f32 const v = (static_cast<f32>(y) + static_cast<f32>(sample / samples_root) / samples_root) / (camera.image_height - 1);
                    Ray const ray{camera_transform.position,
                                  math::normalize(viewport_top_left + Vec3{u, v, 0.0f} * Vec3{camera.viewport_width, -camera.viewport_height, 0.0f} -
                                                  camera_transform.position)};
                    Vec3 const color = cast_ray(ctx, scene, ray, 0);
                    pixel += color;
                }
                pixel /= samples_root * samples_root;
                pixel.x = math::sqrt(pixel.x);
                pixel.y = math::sqrt(pixel.y);
                pixel.z = math::sqrt(pixel.z);
            }
        }

        fs::Output_File_Stream stream("img.ppm"_s);
        write_ppm_file(stream, pixels, camera.image_width, camera.image_height);
        return 0;
    }
} // namespace raytracing

int main() {
    return raytracing::entry();
}
