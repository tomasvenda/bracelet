# =====================================================================
# Fall inference app -- CMakeLists.txt
#
# Expected project layout (adjust paths below if yours differs):
#   CMakeLists.txt
#   prj.conf
#   app.overlay
#   pm_static.yml           (NOT needed here -- no NVS in this app)
#   edge-impulse-sdk/       (from the downloaded EI C++ library)
#   model-parameters/       (from the downloaded EI C++ library)
#   tflite-model/           (from the downloaded EI C++ library)
#   bmi270_legacy_api/      (your existing Bosch driver folder)
#   src/
#     main.c
#     bmi2_zephyr_platform.c
#     classifier.h
#     classifier.cpp
# =====================================================================
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_model)

# --- Edge Impulse SDK ------------------------------------------------
# The SDK ships its own Zephyr integration: utils.cmake provides the
# RECURSIVE_FIND_FILE helper, and the cmake/zephyr subdirectory adds
# all SDK sources (DSP, TFLite Micro, Zephyr porting layer) to 'app'.
include(edge-impulse-sdk/cmake/utils.cmake)
add_subdirectory(edge-impulse-sdk/cmake/zephyr)

# --- Include paths ---------------------------------------------------
target_include_directories(app PRIVATE
    .
    src
    model-parameters
    tflite-model
    bmi270_legacy_api
)

# --- Sources ---------------------------------------------------------
# Generated model sources (compiled TFLite model + resolver)
RECURSIVE_FIND_FILE(MODEL_SOURCES tflite-model "*.cpp")

# Bosch BMI270 legacy driver + your I2C/delay port functions
FILE(GLOB BMI_SOURCES bmi270_legacy_api/*.c)

target_sources(app PRIVATE
    src/main.c
    src/classifier.cpp
    src/bmi2_zephyr_platform.c
    ${MODEL_SOURCES}
    ${BMI_SOURCES}
)