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
set(CMAKE_PROJECT_NAME Hilab_PID)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/vl53l1x.c
    # Dual-channel print module (serial_print / oled_print)
    Core/Src/print.c
    # OLED application layer (home page / printf print / progress bar)
    Core/Src/oled.c
    # Stepper motor basic driver (D36A: EN/DIR GPIO + TIM2_CH4 STEP pulse)
    Core/Src/stepper.c
    # Standard single-loop positional PID controller (reusable, no external deps)
    Core/Src/pid.c
    # Basic Kalman filter observer (2D CV model: encoder + ranger fusion)
    Core/Src/kalman.c
    # OLED (SSD1306) driver
    Middleware/OLED/ssd1306.c
    Middleware/OLED/ssd1306_fonts.c
    # VL53L1X middleware platform
    Middleware/VL53L1X/VL53L1X_Platform/vl53l1_platform_stm32.c
    # VL53L1X middleware core
    Middleware/VL53L1X/core/src/vl53l1_api.c
    Middleware/VL53L1X/core/src/vl53l1_api_calibration.c
    Middleware/VL53L1X/core/src/vl53l1_api_core.c
    Middleware/VL53L1X/core/src/vl53l1_api_debug.c
    Middleware/VL53L1X/core/src/vl53l1_api_preset_modes.c
    Middleware/VL53L1X/core/src/vl53l1_api_strings.c
    Middleware/VL53L1X/core/src/vl53l1_core.c
    Middleware/VL53L1X/core/src/vl53l1_core_support.c
    Middleware/VL53L1X/core/src/vl53l1_error_strings.c
    Middleware/VL53L1X/core/src/vl53l1_register_funcs.c
    Middleware/VL53L1X/core/src/vl53l1_silicon_core.c
    Middleware/VL53L1X/core/src/vl53l1_wait.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Middleware/OLED
    Middleware/VL53L1X/VL53L1X_Platform
    Middleware/VL53L1X/core/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
