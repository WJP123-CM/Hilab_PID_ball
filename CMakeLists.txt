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
    # 硬件模块（一个硬件一个文件：驱动 + 该硬件的基本软件处理）
    Core/Src/tof.c        # VL53L1X 驱动 + 逐帧滤波调用 + 有效帧统计
    Core/Src/motor.c      # 步进电机：STEP/DIR/EN + 限幅/死区/斜率/换向决策
    Core/Src/encoder.c    # 编码器计数/Z 校准/软限幅区间/Flash 掉电保存
    Core/Src/oled.c       # OLED 屏幕原语
    # 纯算法模块（无硬件依赖，可主机侧验证）
    Core/Src/tof_filter.c # ToF 滤波（中值-5 + EMA，整数运算）
    Core/Src/pid.c        # 单环位置式 PID
    # 应用层模块（无硬件）
    Core/Src/key.c        # PA0 按键消抖 + 事件队列
    Core/Src/ctrl.c       # 级联闭环：内外环 PID + 模式 + 掉电保存 + VOFA + 急停
    Core/Src/cmd.c        # 上位机行协议：A/Z/P/I/D/R 命令解析
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

# VL53L1X ULD 头文件自带的 PALDevDataGet/PALDevDataSet 改名提示以 -Wcpp 形式在每次构建输出，
# 并常驻 IDE Problems 面板（对使用方没有行动价值），故仅对 VL53 中间件源文件关闭 -Wcpp；
# 用户代码仍保留该警告（例如自建 #warning 提醒不会因此被吞掉）。
get_target_property(HILAB_TARGET_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)
foreach(src IN LISTS HILAB_TARGET_SOURCES)
    if(src MATCHES "Middleware/VL53L1X")
        set_source_files_properties(${src} PROPERTIES COMPILE_OPTIONS "-Wno-cpp")
    endif()
endforeach()

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
