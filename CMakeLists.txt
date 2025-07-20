cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ElectrochemicalWorkstationNRF52832)

set(app_sources
    src/main.c
    src/ble_faca.c
    src/ble_protocal.c
    drivers/ad5940.c                //后来补充的 ad5940.c文件来自源文件ad5940克隆下载
    drivers/nrf52_peripheral.c
    drivers/freistat_ramp.c
    drivers/freistat_ca.c
    drivers/cyclic_voltammetry.c
    drivers/eis_impedance.c
    drivers/custom_cv.c
    drivers/chrono_amperometric.c
    drivers/chrono_amp_simple.c
    drivers/bat_impedance.c
    elec_methods/ca.c
    elec_methods/ca2.c
    elec_methods/ca3.c
    elec_methods/common.c
    elec_methods/cv.c
    elec_methods/cv2.c
    elec_methods/eis.c
    elec_methods/ocp.c
)

include_directories(include elec_methods drivers)

target_sources(app PRIVATE ${app_sources})

