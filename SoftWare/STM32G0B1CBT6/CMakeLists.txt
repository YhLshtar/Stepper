cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32G0B1CBT6)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Oihex ${CMAKE_PROJECT_NAME}.hex
        COMMENT "Generating hex file for STM32"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/m_task.c
    Common/Common.c
    Include/Flash/m_flash.c
    Include/SerialPort/serialport.c
    Include/Timers/m_timers.c
    Include/FDCAN/m_fdcan.c
    Include/TMC2209/TMC2209.c
    Include/MT6701/MT6701.c
    Include/StepMotor/Stepper_control.c
    Include/PIDControl/PIDControl.c
    Libraries/FreeModbus/port.c
    Libraries/FreeModbus/port/portevent.c
    Libraries/FreeModbus/port/portserial.c
    Libraries/FreeModbus/modbus/mb.c
    Libraries/FreeModbus/modbus/rtu/mbcrc.c
    Libraries/FreeModbus/modbus/rtu/mbrtu.c
    Libraries/FreeModbus/modbus/functions/mbfunccoils.c
    Libraries/FreeModbus/modbus/functions/mbfuncdiag.c
    Libraries/FreeModbus/modbus/functions/mbfuncdisc.c
    Libraries/FreeModbus/modbus/functions/mbfuncholding.c
    Libraries/FreeModbus/modbus/functions/mbfuncinput.c
    Libraries/FreeModbus/modbus/functions/mbfuncother.c
    Libraries/FreeModbus/modbus/functions/mbutils.c
    Libraries/FreeModbus/modbus/include/mbfunuserf.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Common/
    Include/Flash/
    Include/SerialPort/
    Include/FDCAN/
    Include/Timers/
    Include/TMC2209/
    Include/MT6701/
    Include/StepMotor/
    Include/PIDControl/
    Libraries/FreeModbus/
    Libraries/FreeModbus/modbus/
    Libraries/FreeModbus/modbus/include/
    Libraries/FreeModbus/modbus/rtu/
    Libraries/FreeModbus/port/
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O1 -g3")
#慎重启用LTO，没有具体测试过有无问题。除非真的要极限减小固件大小。慎用！！！
#但是KEIL里是自动启用LTO。不过KEIL用的ARMCLANG,没有BUG是应该的😊
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")# 启用 LTO
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -gdwarf-4")
#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-u_printf_float")
#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--strip-debug")
