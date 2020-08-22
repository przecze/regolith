add_library(regolith Regolith.cpp)
target_link_libraries(regolith "${BULLET_LIBRARIES}")
target_include_directories (regolith PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(utils utils.cpp)
target_link_libraries(utils "${BULLET_LIBRARIES}")
target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
