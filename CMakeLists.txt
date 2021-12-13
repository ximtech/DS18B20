cmake_minimum_required(VERSION 3.20)
project(DS18B20 C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

CPMAddPackage(
        NAME DWT_Delay
        GITHUB_REPOSITORY ximtech/DWT_Delay
        GIT_TAG origin/main)

CPMAddPackage(
        NAME Vector
        GITHUB_REPOSITORY ximtech/Vector
        GIT_TAG origin/main)

CPMAddPackage(
        NAME CRC
        GITHUB_REPOSITORY ximtech/CRC
        GIT_TAG origin/main)

set(DS18B20_DIRECTORY
        ${DWT_DELAY_DIRECTORY}
        ${DS18B20_SOURCE_DIR}
        ${DS18B20_SOURCE_DIR}/include
        CACHE STRING "DS18B20 directories include to the main project" FORCE)

set(DS18B20_SOURCES
        ${DWT_DELAY_SOURCES}
        ${DS18B20_SOURCE_DIR}/${PROJECT_NAME}.c
        ${DS18B20_SOURCE_DIR}/include/${PROJECT_NAME}.h
        CACHE STRING "DS18B20 source files include to the main project" FORCE)

link_libraries(Vector CRC)