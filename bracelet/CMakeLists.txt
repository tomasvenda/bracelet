cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bracelet_tracker)

# From your original: Include Zephyr net/ip headers
target_include_directories(app PUBLIC ${ZEPHYR_BASE}/subsys/net/ip)

# Our custom include directory for shared headers
zephyr_include_directories(src/include)


target_sources(app PRIVATE 
    src/main.c
    src/localization_wifi.c
    src/low_power_acl.c
    src/data_manager.c
    src/lte_service.c
)