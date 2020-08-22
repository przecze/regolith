add_executable(cone_penetration_test main.cpp ConePenetrationTest.cpp)
target_link_libraries(cone_penetration_test regolith utils "${BULLET_LIBRARIES}")
