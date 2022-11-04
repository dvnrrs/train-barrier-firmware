#
# train-barrier-firmware
# ESP32 firmware for train barrier servo motor and light controller
# Copyright (C) 2022 David Norris & Alexander Norris
#

idf_component_register(
    SRCS            "main.cpp"
    INCLUDE_DIRS    "")

component_compile_options(-std=gnu++17)
component_compile_options(-Werror)
