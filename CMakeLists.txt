cmake_minimum_required(VERSION 3.22)

#
# 本文件只生成一次，
# 后续多次调用转换器时不会重新生成。
#
# 用户可按项目需要修改本文件。
#

# 设置编译器。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# 定义构建类型。
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 设置项目名称。
set(CMAKE_PROJECT_NAME FOC_ZT)

# 导出编译命令，便于clangd等工具建立索引。
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 项目核心设置。
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用汇编和C语言支持。
enable_language(C ASM)

# 创建可执行目标。
add_executable(${CMAKE_PROJECT_NAME}
        Lib/Src/FOC_Math.c
        Lib/Src/FOC_Control.c
        Lib/Src/PID_Control.c
        Lib/Src/ssi.c
        Lib/Inc/ssi.h
        Lib/Src/pos_process.c
        Lib/Inc/pos_process.h
        Lib/Src/turntable_comm.c
        Lib/Inc/turntable_comm.h
        Experiments/Src/Experiment_Control.c
        Experiments/Src/Experiment_Baseline.c
        Experiments/Src/Experiment_ESO.c
        Experiments/Src/Experiment_FixedFeedforward.c
        Experiments/Src/Experiment_FixedFF_Sweep.c
        Experiments/Src/Experiment_LearningFeedforward.c
)

# 添加STM32CubeMX生成的源码。
add_subdirectory(cmake/stm32cubemx)

# 设置链接目录。
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户库搜索路径。
)

# 向可执行目标添加源码。
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户源码。
)

# 添加头文件搜索路径。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户头文件路径。
        Lib/Inc
        Experiments/Inc
)

# 添加项目宏定义。
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 在此添加用户宏定义。
)

# 使用C++文件时移除错误的libob.a依赖。
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 添加链接库。
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # 在此添加用户库。
)
