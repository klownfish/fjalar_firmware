
zephyr_library_named(protocol)
zephyr_library_sources(
    schema.pb.c
    pb_common.c
    pb_decode.c
    pb_encode.c
    protocol.c
)
zephyr_include_directories(.)
