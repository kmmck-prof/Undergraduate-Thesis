# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common
                         $ENV{IDF_PATH}/examples/common_components/iperf)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_ot_br)
target_add_binary_data(esp_ot_br.elf "main/mqtt_eclipseprojects_io.pem" TEXT)