cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ElectrochemicalWorkstationNRF52832)

FILE(GLOB app_sources src/*.c include/*.h drivers/*.c drivers/*.h elec_methods/*.h elec_methods/*.c)
target_sources(app PRIVATE ${app_sources} src/main.c)

