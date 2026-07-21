# =====================================================================
# Bracelet final project -- CMakeLists.txt
# =====================================================================
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(final_project)

# --- Edge Impulse SDK (adds DSP, TFLite Micro, Zephyr porting) -------
include(edge_impulse/edge-impulse-sdk/cmake/utils.cmake)
add_subdirectory(edge_impulse/edge-impulse-sdk/cmake/zephyr)

# --- Include paths ----------------------------------------------------
target_include_directories(app PRIVATE
    include
    bmi270_legacy_api
    edge_impulse
    edge_impulse/model-parameters
    edge_impulse/tflite-model
)

# --- Sources ----------------------------------------------------------
# Manually glob the model sources so the Zephyr app target can see them
RECURSIVE_FIND_FILE(MODEL_SOURCES edge_impulse/tflite-model "*.cpp")

FILE(GLOB APP_C_SOURCES src/*.c)
FILE(GLOB BMI_SOURCES bmi270_legacy_api/*.c)

target_sources(app PRIVATE
    ${APP_C_SOURCES}
    src/ml_wrapper.cpp
    ${MODEL_SOURCES}
    ${BMI_SOURCES}
)