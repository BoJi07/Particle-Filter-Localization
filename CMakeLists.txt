cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 11)

project(particle_filter)

add_executable(particle_filter src/main.cpp src/Tools.cpp src src/ParticleFilter.cpp)

find_package(PythonLibs 2.7)

target_include_directories(particle_filter PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(particle_filter ${PYTHON_LIBRARIES})