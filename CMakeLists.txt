cmake_minimum_required(VERSION 3.5)

project(Path_Planning LANGUAGES CXX)

find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

add_executable(path_planning src/main.cpp)

set_target_properties(path_planning PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

target_include_directories(path_planning PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_options(path_planning PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(path_planning PRIVATE ${Boost_LIBRARIES} Threads::Threads)

# Keep runtime data next to the binary so the executable can be launched
# directly from the build directory.
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
