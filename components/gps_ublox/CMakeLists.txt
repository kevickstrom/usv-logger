# components/gps_ublox/CMakeLists.txt

file(GLOB GPS_UBLOX_SRCS "src/*.cpp")

idf_component_register(
    SRCS ${GPS_UBLOX_SRCS}
    INCLUDE_DIRS "src"
)