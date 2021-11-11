cmake_minimum_required(VERSION 3.15)
include(FetchContent)

project(RayTracing)

string(FIND "${CMAKE_CXX_COMPILER}" "clang++" RT_COMPILER_CLANGPP)
if(RT_COMPILER_CLANGPP GREATER -1)
    set(RT_COMPILER_CLANGPP ON)
else()
    set(RT_COMPILER_CLANGPP OFF)
endif()

if(RT_COMPILER_CLANGPP)
    set(RT_COMPILE_FLAGS
        # -march=native
        -fno-math-errno
        -fno-char8_t # Yea, just no.
        -fno-rtti 
        -fno-exceptions
        -Wall -Wextra -pedantic
        -ferror-limit=1
        
        -Werror=return-type
        -Werror=uninitialized
        -Werror=reorder-init-list

        -Wnon-virtual-dtor
        -Wnewline-eof
    )
elseif(MSVC)
    set(RT_COMPILE_FLAGS
        /GF # Kill duplicated strings
    )
endif()

# Add anton_core
FetchContent_Declare(
    anton_core
    GIT_REPOSITORY https://github.com/kociap/anton_core.git
)
FetchContent_MakeAvailable(anton_core)
# Add anton_import
FetchContent_Declare(
    anton_import
    GIT_REPOSITORY https://github.com/kociap/anton_import.git
)
FetchContent_MakeAvailable(anton_import)

add_executable(raytracing
    "${CMAKE_CURRENT_SOURCE_DIR}/source/build_config.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/camera.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/camera.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/filesystem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/filesystem.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/handle.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/materials.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/materials.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/random_engine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/source/random_engine.hpp"
)
set_target_properties(raytracing PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
target_compile_options(raytracing PRIVATE ${RT_COMPILE_FLAGS})
target_link_libraries(raytracing PUBLIC anton_core anton_import)
target_include_directories(raytracing PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/source")
