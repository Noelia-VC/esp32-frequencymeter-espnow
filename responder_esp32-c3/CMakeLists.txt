cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(responder_esp32-c3)
idf_build_set_property(MINIMAL_BUILD ON)