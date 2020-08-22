cmake_minimum_required(VERSION 3.16)

project(regolith)
find_package(Bullet REQUIRED)
message(STATUS "${BULLET_INCLUDE_DIRS}")

include_directories("${BULLET_INCLUDE_DIRS}")
add_subdirectory(src)
add_subdirectory(simulations)
