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

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
if(NOT CLANG_FORMAT_EXECUTABLE)
  set(CLANG_FORMAT_EXECUTABLE clang-format)
endif()
set(PLANNING_FORMAT_BASE "HEAD" CACHE STRING
  "Git base revision used by the changed-lines format check"
)
add_custom_target(
  format-check
  COMMAND ${CMAKE_COMMAND} -E env
    CLANG_FORMAT=${CLANG_FORMAT_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/check-format.sh
    ${PLANNING_FORMAT_BASE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Checking changed C++ lines against .clang-format"
)

add_library(planning_core
  src/active_behavior_planner.cpp
  src/behavior_planner.cpp
  src/longitudinal_qp.cpp
  src/map.cpp
  src/path_stitcher.cpp
  src/planner.cpp
  src/planner_monitor.cpp
  src/qp_solver.cpp
  src/simulator_protocol.cpp
  src/spatial_path.cpp
  src/st_corridor.cpp
  src/speed_reference.cpp
  src/traffic_prediction.cpp
  src/traffic_predictor.cpp
  src/traffic_tracker.cpp
  src/trajectory_assembler.cpp
  src/trajectory_sampler.cpp
  src/trajectory_validator.cpp
)
target_include_directories(planning_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(planning_core SYSTEM PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Eigen-3.3
)
target_link_libraries(planning_core PUBLIC osqp::osqp Threads::Threads)

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

add_executable(phase1_safety_tests tests/phase1_safety_tests.cpp)
target_link_libraries(phase1_safety_tests PRIVATE planning_core)
configure_planning_target(phase1_safety_tests)
add_test(NAME phase1_safety_tests COMMAND phase1_safety_tests)

add_executable(phase2_traffic_tests
  tests/phase2_traffic_tests.cpp
)
target_link_libraries(phase2_traffic_tests PRIVATE planning_core)
configure_planning_target(phase2_traffic_tests)
add_test(NAME phase2_traffic_tests COMMAND phase2_traffic_tests)

add_executable(phase2_behavior_tests
  tests/phase2_behavior_tests.cpp
)
target_link_libraries(phase2_behavior_tests PRIVATE planning_core)
configure_planning_target(phase2_behavior_tests)
add_test(NAME phase2_behavior_tests COMMAND phase2_behavior_tests)

add_executable(phase2_spatial_path_tests
  tests/phase2_spatial_path_tests.cpp
)
target_link_libraries(phase2_spatial_path_tests PRIVATE planning_core)
configure_planning_target(phase2_spatial_path_tests)
add_test(NAME phase2_spatial_path_tests COMMAND phase2_spatial_path_tests)

add_executable(phase2_st_corridor_tests
  tests/phase2_st_corridor_tests.cpp
)
target_link_libraries(phase2_st_corridor_tests PRIVATE planning_core)
configure_planning_target(phase2_st_corridor_tests)
add_test(NAME phase2_st_corridor_tests COMMAND phase2_st_corridor_tests)

add_executable(active_behavior_tests tests/active_behavior_tests.cpp)
target_link_libraries(active_behavior_tests PRIVATE planning_core)
configure_planning_target(active_behavior_tests)
add_test(NAME active_behavior_tests COMMAND active_behavior_tests)
add_test(
  NAME active_behavior_long_closed_loop_tests
  COMMAND active_behavior_tests --long-only
)
set_tests_properties(active_behavior_long_closed_loop_tests PROPERTIES
  LABELS "closed-loop;long;active-highway-navigation"
  TIMEOUT 180
)

# Keep runtime data next to the binary so the executable can be launched
# directly from the build directory.
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
