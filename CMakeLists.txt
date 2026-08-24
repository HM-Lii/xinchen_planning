cmake_minimum_required(VERSION 3.5)

project(Path_Planning LANGUAGES CXX)

# Use one conventional build directory and make it an optimized build unless
# the caller explicitly selects another configuration.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Boost 1.66 REQUIRED COMPONENTS system)
set(CMAKE_PREFIX_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/.deps/osqp;${CMAKE_PREFIX_PATH}"
)
find_package(osqp REQUIRED)
find_package(Threads REQUIRED)

add_library(planning_core
  src/longitudinal_qp.cpp
  src/map.cpp
  src/path_stitcher.cpp
  src/planner.cpp
  src/planner_monitor.cpp
  src/qp_solver.cpp
  src/simulator_protocol.cpp
  src/speed_reference.cpp
  src/traffic_predictor.cpp
  src/trajectory_sampler.cpp
)
target_include_directories(planning_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(planning_core SYSTEM PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Eigen-3.3
)
target_link_libraries(planning_core PUBLIC osqp::osqp)

add_executable(path_planning src/main.cpp)
target_include_directories(path_planning PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(path_planning PRIVATE
  planning_core
  ${Boost_LIBRARIES}
  Threads::Threads
)

function(configure_planning_target target)
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

configure_planning_target(planning_core)
configure_planning_target(path_planning)

enable_testing()
add_executable(planning_tests tests/planning_tests.cpp)
target_link_libraries(planning_tests PRIVATE planning_core)
configure_planning_target(planning_tests)
add_test(NAME planning_tests COMMAND planning_tests)

add_executable(longitudinal_tests tests/longitudinal_tests.cpp)
target_link_libraries(longitudinal_tests PRIVATE planning_core)
configure_planning_target(longitudinal_tests)
add_test(NAME longitudinal_tests COMMAND longitudinal_tests)

# Keep runtime data next to the binary so the executable can be launched
# directly from the build directory.
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
