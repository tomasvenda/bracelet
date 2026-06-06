cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bracelet_app)

# Add your include directory so the compiler can find your headers
zephyr_include_directories(include)

# Add all your source files
target_sources(app PRIVATE 
    src/main.c
    src/fsm.c
    src/sensors.c
    src/comms.c
    src/display.c
    src/ml_wrapper.cpp
)