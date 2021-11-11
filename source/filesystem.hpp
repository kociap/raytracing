#pragma once

#include <anton/array.hpp>
#include <anton/expected.hpp>
#include <anton/math/vec3.hpp>
#include <anton/stream.hpp>
#include <anton/string.hpp>
#include <build_config.hpp>

namespace raytracing {
    Expected<Array<u8>, String> read_file(String_View path);
    void write_ppm_file(Output_Stream& stream, Slice<Vec3 const> pixels, i64 width, i64 height);
} // namespace raytracing
