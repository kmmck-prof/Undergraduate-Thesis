# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(SYSTEM_BLE_GW)
target_add_binary_data(SYSTEM_BLE_GW.elf "main/mqtt_eclipseprojects_io.pem" TEXT)
