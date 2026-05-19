
cmake_minimum_required(VERSION 3.25)

project ("CS Algorithms" VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Match the static MSVC runtime (/MT) used by the prebuilt CppModel libs
# and the x64-windows-static vcpkg triplet.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

enable_testing()

add_subdirectory ("simulations")

