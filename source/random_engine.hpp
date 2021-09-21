#pragma once

#include <build_config.hpp>

namespace raytracing {
    struct Random_Engine;

    [[nodiscard]] Random_Engine* create_random_engine(i64 seed);
    void destroy_random_engine(Random_Engine* engine);

    [[nodiscard]] f32 random_f32(Random_Engine* engine, f32 min, f32 max);
    [[nodiscard]] Vec3 random_unit_vec3(Random_Engine* engine);
} // namespace raytracing
