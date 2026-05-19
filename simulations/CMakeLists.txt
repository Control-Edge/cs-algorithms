
add_definitions(-DCPPHTTPLIB_OPENSSL_SUPPORT)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/first-order-c
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/pid-controller-c
)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lib)

# ---- OS-specific configuration ----
# Each branch sets:
#   PLATFORM_LIBS     - libraries appended to every simulation target
#   PLATFORM_INCLUDES - include dirs applied per-target (empty if none)
if (WIN32)
    if (MINGW AND CMAKE_C_COMPILER MATCHES "(ucrt64|clang64)")
        message("using MSYS2 (UCRT64 or CLANG64)")
        set(PLATFORM_LIBS ssl crypto z crypt32 ws2_32)
    elseif (MINGW)
        message("MINGW not supported")
    elseif (MSVC)
        message("Compiling with Visual Studio C++")
        find_package(OpenSSL REQUIRED)
        find_package(ZLIB REQUIRED)
        set(PLATFORM_LIBS OpenSSL::SSL OpenSSL::Crypto ZLIB::ZLIB ws2_32)
        set(PLATFORM_INCLUDES ${OPENSSL_INCLUDE_DIR})
    endif()
elseif (APPLE)
    message("Compiling for macOS")
    find_package(OpenSSL REQUIRED)
    find_package(ZLIB REQUIRED)
    set(PLATFORM_LIBS OpenSSL::SSL OpenSSL::Crypto ZLIB::ZLIB)
    set(PLATFORM_INCLUDES ${OPENSSL_INCLUDE_DIR})
elseif (UNIX)
    message("Compiling for UNIX")
    set(PLATFORM_LIBS crypto ssl z)
endif()

set(SIMULATION_LIBS CppModelBase CppModelLib ixwebsocket ${PLATFORM_LIBS})

# add_simulation_test(<name> <sources...>)
# Creates an executable from the given sources, links the shared simulation
# libs, applies platform include dirs, and registers it as a ctest.
function(add_simulation_test name)
    add_executable(${name} ${ARGN})
    # CppModelBase is C++; link with the C++ driver so libstdc++ is pulled in
    # even when the simulation sources are pure C.
    set_target_properties(${name} PROPERTIES LINKER_LANGUAGE CXX)
    if (PLATFORM_INCLUDES)
        target_include_directories(${name} PRIVATE ${PLATFORM_INCLUDES})
    endif()
    target_link_libraries(${name} ${SIMULATION_LIBS})
    add_test(NAME Simulation${name} COMMAND ${name})
endfunction()

# ---- Tests ----

add_simulation_test(PidSystemSim
    PidSystemSim.c
    ../algorithms/first-order-c/CFirstOrder.c
    ../algorithms/pid-controller-c/CPid.c
)
