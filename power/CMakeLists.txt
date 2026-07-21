# Require a minimum CMake version (Standard for NCS)
cmake_minimum_required(VERSION 3.20.0)

# Find the Zephyr package. This is the magic line that loads the whole RTOS.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Name your specific project
project(power)

target_sources(app PRIVATE src/main.c)
