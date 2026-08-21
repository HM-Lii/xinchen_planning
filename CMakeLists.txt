cmake_minimum_required(VERSION 3.5)

project(Path_Planning LANGUAGES CXX)

find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

add_library(planning_core
  src/map.cpp
  src/planner.cpp
  src/simulator_protocol.cpp
)
target_include_directories(planning_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

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

# Keep runtime data next to the binary so the executable can be launched
# directly from the build directory.
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
