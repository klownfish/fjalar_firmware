cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
list(APPEND EXTRA_ZEPHYR_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/dummysensor
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/ms5611
  )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fjalar)


target_sources(app PRIVATE
    src/flight_state.c
    src/sensors.c
    src/communication.c
    src/commands.c
    src/actuation.c
    src/main.c
    src/filter.c
)

add_subdirectory(../common ../build/common)

add_subdirectory(lib)
target_link_libraries(app PRIVATE protocol)
