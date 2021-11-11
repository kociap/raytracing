#include <filesystem.hpp>

#include <anton/filesystem.hpp>
#include <anton/format.hpp>

namespace raytracing {
    Expected<Array<u8>, String> read_file(String_View const path) {
        String path_str{path};
        fs::Input_File_Stream stream(path_str);
        if(!stream) {
            return {expected_error, format("could not open file \"{}\" for reading", path)};
        }

        stream.seek(Seek_Dir::end, 0);
        i64 const size = stream.tell();
        stream.seek(Seek_Dir::beg, 0);
        Array<u8> result{reserve, size};
        result.force_size(size);
        stream.read(result);
        return {expected_value, ANTON_MOV(result)};
    }

    void write_ppm_file(Output_Stream& stream, Slice<Vec3 const> const pixels, i64 const width, i64 const height) {
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
} // namespace raytracing
