#pragma once

#include <build_config.hpp>

namespace raytracing {
    struct Camera {
        // width / height
        f32 aspect_ratio;
        f32 vfov;
        f32 viewport_width;
        f32 viewport_height;
        f32 focal_length;
        // Width of the generated image in pixels.
        i64 image_width;
        // Height of the generated image in pixels.
        i64 image_height;

        Camera(f32 vfov, f32 aspect_ratio, i64 image_height);
    };

    struct Camera_Target {
        Vec3 position;
    };
} // namespace raytracing
