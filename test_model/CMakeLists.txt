cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fall_detection_inference)

# Notice we changed this to main.cpp!
target_sources(app PRIVATE src/main.cpp)

# 1. Include Edge Impulse Header directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk
    ${CMAKE_CURRENT_SOURCE_DIR}/model-parameters
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite-model
)

# 2. Automatically find and compile all Edge Impulse C/C++ files
file(GLOB_RECURSE ei_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite-model/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tflite-model/*.c
)
target_sources(app PRIVATE ${ei_sources})