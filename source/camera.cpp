#include <camera.hpp>

#include <anton/math/math.hpp>

namespace raytracing {
    Camera::Camera(f32 const vfov, f32 const aspect_ratio, i64 const image_height): vfov(vfov), aspect_ratio(aspect_ratio), image_height(image_height) {
        f32 const fov_rad = math::radians(vfov);
        f32 const fov_tan = math::tan(0.5f * fov_rad);
        viewport_height = 2.0f * fov_tan;
        viewport_width = viewport_height * aspect_ratio;
        image_width = image_height * aspect_ratio;

        focal_length = 1.0f;
    }
} // namespace raytracing
