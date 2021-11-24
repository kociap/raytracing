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
#include <intersections.hpp>
#include <kd_tree.hpp>
#include <materials.hpp>
#include <random_engine.hpp>
#include <scene.hpp>

namespace raytracing {
    struct Context {
        Random_Engine* random_engine = nullptr;
        i64 bounces = 0;
        i64 samples = 0;
    };

    [[nodiscard]] static Optional<Surface_Interaction> intersect_scene(Scene const& scene, Ray const ray) {
        bool hit = false;
        Surface_Interaction result;
        // Intersect spheres in the scene.
        {
            for(Sphere const& sphere: scene.spheres) {
                Optional<Surface_Interaction> intersection_result = intersect_sphere(ray, sphere);
                if(intersection_result && intersection_result < result.distance) {
                    result = intersection_result.value();
                    hit = true;
                }
            }
        }
        // Instersect triangles in the scene.
        {
            for(Triangle const& triangle: scene.triangles) {
                Optional<Surface_Interaction> intersection_result = intersect_triangle(ray, triangle);
                if(intersection_result && intersection_result->distance < result.distance) {
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

    static Vec3 cast_ray(Context const& ctx, Scene const& scene, KD_Tree& tree, Ray const ray, i64 const bounce) {
        if(bounce >= ctx.bounces) {
            return Vec3{0.0f};
        }

        Optional<Surface_Interaction> const result = tree.intersect(scene, ray);
        if(result) {
            Optional<Scatter_Result> scatter_result = scatter(ctx.random_engine, ray, result->distance, result->normal, result->material);
            if(scatter_result) {
                Vec3 const color = cast_ray(ctx, scene, tree, scatter_result->ray, bounce + 1);
                return scatter_result->attenuation * color;
            } else {
                return Vec3{0.0f};
            }
        }

        // Sky gradient
        f32 const t = 0.5f * (ray.direction.y + 1.0f);
        return (1.0f - t) * Vec3{1.0f} + t * Vec3{0.5f, 0.7f, 1.0f};
    }

    [[nodiscard]] static Array<Vec3> render_scene(Context const& ctx, Scene const& scene, Camera const& camera, Camera_Target const& target) {
        // TODO: The lookat code does not correctly handle camera target being positioned exactly above the camera.
        Vec3 const camera_view = math::normalize(target.position - camera.position);
        Vec3 const camera_right = math::normalize(math::cross(camera_view, Vec3{0.0f, 1.0f, 0.0f}));
        Vec3 const camera_up = math::cross(camera_right, camera_view);
        Mat3 const viewport_rotation{camera_right, camera_up, camera_view};
        Vec3 const viewport_top_left = viewport_rotation * Vec3{-0.5f * camera.viewport_width, 0.5f * camera.viewport_height, camera.focal_length};

        KD_Tree tree;
        tree.build(scene, KD_Tree::Build_Options{.max_primitives = 16, .empty_bonus = 0.2f});

        Console_Output cout;
        Array<Vec3> pixels{reserve, camera.image_width * camera.image_height};
        i64 const samples_root = math::sqrt(ctx.samples);
        for(i64 y = 0; y < camera.image_height; ++y) {
            cout.write(format("processing row {}\n", y));
            for(i64 x = 0; x < camera.image_width; ++x) {
                Vec3& pixel = pixels.push_back(Vec3{0.0f});
                for(i64 sample = 0; sample < samples_root * samples_root; ++sample) {
                    f32 const u = (static_cast<f32>(x) + static_cast<f32>(sample % samples_root) / samples_root) / (camera.image_width - 1);
                    f32 const v = (static_cast<f32>(y) + static_cast<f32>(sample / samples_root) / samples_root) / (camera.image_height - 1);
                    Ray const ray{camera.position,
                                  math::normalize(viewport_top_left + u * camera.viewport_width * camera_right - v * camera.viewport_height * camera_up)};
                    Vec3 const color = cast_ray(ctx, scene, tree, ray, 0);
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

        Camera camera{Vec3{2.0f, 2.0f, 5.0f}, 90.0f, 16.0f / 9.0f, 720};
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
        Expected<Array<u8>, String> file_read_result = read_file("./assets/skull.obj");
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
            for(i64 i = 0; i < mesh.indices.size(); i += 3) {
                Vec3 const v1 = mesh.vertices[mesh.indices[i]];
                Vec3 const v2 = mesh.vertices[mesh.indices[i + 1]];
                Vec3 const v3 = mesh.vertices[mesh.indices[i + 2]];
                // TODO: Apply object-world transform here.
                Triangle triangle{v1, v2, v3, grey_diffuse_handle};
                scene.triangles.push_back(ANTON_MOV(triangle));
            }
            // f32 const x = random_f32(rnd, -2.0f, 2.0f);
            // f32 const y = random_f32(rnd, -2.0f, 2.0f);
            // f32 const z = random_f32(rnd, -2.0f, 2.0f);
            // scene.mesh_transforms.push_back(Transform{Vec3{x, y, z}});
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
        scene.spheres.push_back(Sphere{Vec3{0.0f, -201.0f, -3.0f}, 200.0f, green_diffuse_handle});

        Array<Vec3> const pixels = render_scene(ctx, scene, camera, target);

        fs::Output_File_Stream stream("img.ppm"_s);
        write_ppm_file(stream, pixels, camera.image_width, camera.image_height);
        return 0;
    }
} // namespace raytracing

int main() {
    return raytracing::entry();
}
