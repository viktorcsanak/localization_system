# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Localization_service)

target_sources(app PRIVATE src/main.c)
add_compile_options(-Os)

##################################################################
add_subdirectory(src/dw1000_driver)
target_link_libraries(app PRIVATE dw1000_driver)
target_link_libraries(dw1000_driver INTERFACE zephyr_interface)
target_link_libraries(dw1000_driver PRIVATE kernel)
target_include_directories(dw1000_driver PUBLIC
    ${PROJECT_BINARY_DIR}
    ${ZEPHYR_BASE}/kernel/include
    ${ZEPHYR_BASE}/arch/${ARCH}/include
)

##################################################################

target_include_directories(app PUBLIC
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src/dw1000_driver/include
)