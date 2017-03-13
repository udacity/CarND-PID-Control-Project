project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++1y)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)


add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)
