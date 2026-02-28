idf_component_register(
    SRCS
        "src/nmea/nmea.c"
        "src/nmea/parser_static.c"
        "src/parsers/parse.c"
        "src/parsers/gpgga.c"
        "src/parsers/gpgll.c"
        "src/parsers/gpgsa.c"
        "src/parsers/gpgsv.c"
        "src/parsers/gprmc.c"
        "src/parsers/gptxt.c"
        "src/parsers/gpvtg.c"
    INCLUDE_DIRS
        "src/nmea"
        "src/parsers"
)

# 7 parsers enabled (adjust if you remove some)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    PARSER_COUNT=7
    ENABLE_GPGGA
    ENABLE_GPGLL
    ENABLE_GPGSA
    ENABLE_GPGSV
    ENABLE_GPRMC
    ENABLE_GPTXT
    ENABLE_GPVTG
)