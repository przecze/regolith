#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
add_definitions(-DB3_USE_STANDALONE_EXAMPLE)

set(BULLET_PHYSICS_SOURCE_DIR "/home/przemek/repos/bullet3")

set(sim_srcs
	ConePenetrationTest.cpp
)

add_executable(cone_penetration_test main.cpp "${sim_srcs}")
add_executable(cone_penetration_test_gui main_gui.cpp "${sim_srcs}")

target_link_libraries(cone_penetration_test regolith utils "${BULLET_LIBRARIES}")
target_link_libraries(cone_penetration_test_gui regolith utils "${BULLET_LIBRARIES}" OpenGLWindow BulletExampleBrowserLib Bullet3OpenCL_clew Bullet3Common dl pthread BulletCollision)
