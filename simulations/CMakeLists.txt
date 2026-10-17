
# CppModel 0.8+ ships a shared library with a CMake package; it carries its own
# OpenSSL/zlib/websocket dependencies, so nothing platform-specific is linked here.
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies)
find_package(cppmodel 0.8 REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/first-order
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/pid-controller
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/line-cruise-control
)

# add_simulation_test(<name> <sources...>)
# Creates an executable from the given sources, links CppModel, copies its
# shared library next to the executable, and registers it as a ctest.
function(add_simulation_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE cppmodel::cppmodel)
    if (UNIX AND NOT APPLE)
        target_link_libraries(${name} PRIVATE m)
    endif()
    cppmodel_deploy_runtime(${name})
    add_test(NAME Simulation${name} COMMAND ${name})
endfunction()

# ---- Tests ----

add_simulation_test(CPidSystemSim
    CPidSystemSim.c
    ../algorithms/first-order/CFirstOrder.c
    ../algorithms/pid-controller/CPid.c
)

add_simulation_test(CppPidSystemSim
    CppPidSystemSim.cpp
    ../algorithms/first-order/FirstOrder.cpp
    ../algorithms/pid-controller/PIDController.cpp
)

add_simulation_test(CLineCruiseSim
    CLineCruiseSim.c
    models/CLineCarModel.c
    ../algorithms/line-cruise-control/CLineCruise.c
    ../algorithms/pid-controller/CPid.c
)
