# FMS Node - factory monitoring demo node (target: esp32s3, ESP-IDF v5.2+/v6)
cmake_minimum_required(VERSION 3.16)

# Minimal build: only main + its dependency tree gets compiled. Keeps builds
# fast and avoids compiling unused IDF components (esp_lcd, camera, ...).
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(fms_node)
