add_library(regolith Regolith.cpp utils.cpp)
target_link_libraries(regolith yaml-cpp "${BULLET_LIBRARIES}")
target_include_directories (regolith PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(simprof simprof.cpp)
target_link_libraries(simprof nlohmann_json)
target_include_directories (simprof PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
