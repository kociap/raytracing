#include <random_engine.hpp>

#include <random>

namespace raytracing {
    struct Random_Engine {
        std::mt19937_64 engine;

        Random_Engine(i64 seed): engine(seed) {}
    };

    Random_Engine* create_random_engine(i64 const seed) {
        return new Random_Engine{seed};
    }

    void destroy_random_engine(Random_Engine* engine) {
        delete engine;
    }

    f32 random_f32(Random_Engine* const engine, f32 const min, f32 const max) {
        u64 const random = engine->engine();
        bool const odd = random & 1;
        i64 halved = random / 2;
        halved -= 4611686018427387904;
        halved *= 2;
        halved += odd;
        f64 const random_double = halved / f64(~0ULL >> 1);
        f64 const range_half = 0.5 * (max - min);
        f64 const remapped = random_double * range_half + range_half + min;
        return remapped;
    }

    Vec3 random_unit_vec3(Random_Engine* const engine) {
        f32 const x = random_f32(engine, -1.0f, 1.0f);
        f32 const y = random_f32(engine, -1.0f, 1.0f);
        f32 const z = random_f32(engine, -1.0f, 1.0f);
        return math::normalize(Vec3{x, y, z});
    }
} // namespace raytracing
