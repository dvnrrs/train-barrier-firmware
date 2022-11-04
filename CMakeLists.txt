#
# train-barrier-firmware
# ESP32 firmware for train barrier servo motor and light controller
# Copyright (C) 2022 David Norris & Alexander Norris
#

cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "1.0.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(train-barrier-firmware)
