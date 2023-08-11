cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(OPENOCD_NRF5_SUBFAMILY "nrf52")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tracker)
target_sources(app PRIVATE
    src/main.c
)

# add_subdirectory(lib)
# target_link_libraries(app PRIVATE protocol)
