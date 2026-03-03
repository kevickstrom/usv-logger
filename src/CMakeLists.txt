# src/CMakeLists.txt
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."         # includes src/ headers if any
    REQUIRES gps_ublox ping-cpp   # <-- declare dependency on components
)