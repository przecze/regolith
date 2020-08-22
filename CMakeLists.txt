cmake_minimum_required(VERSION 3.16)

project(regolith)
find_package(Bullet REQUIRED)
message(STATUS "${BULLET_INCLUDE_DIRS}")

include_directories("${BULLET_INCLUDE_DIRS}")
add_executable(regolith main.cpp utils.cpp Regolith.cpp ConePenetrationTest.cpp)
target_link_libraries(regolith "${BULLET_LIBRARIES}")
