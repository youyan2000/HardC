# =====================================================================
#  gcc-arm-none-eabi.cmake — ARM GCC (arm-none-eabi) 工具链文件
#
#  对标 LibXR/bsp-dev-c 的 cmake/gcc-arm-none-eabi.cmake (工具链放库内
#  分发), 按 HardC 库定位适配: 支持静态库 (scaffold 骨架) 与可执行
#  (外部 CubeMX 工程) 两种目标; 链接脚本不硬编码芯片, 由工程用
#  -DHARDC_LINK_SCRIPT=<.ld 路径> 提供 (缺省指向 ${CMAKE_SOURCE_DIR})。
#
#  用法:
#     cmake -S build/gen/<project> -B build/out/<project> \
#       -DCMAKE_TOOLCHAIN_FILE=<HardC>/cmake/gcc-arm-none-eabi.cmake
#
#  本工具链可产静态库 (scaffold 骨架 add_library STATIC) 或可执行
#  (外部工程加 -DHARDC_LINK_SCRIPT 指向工程 .ld)。
# =====================================================================

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# ---- 工具链根目录 (arm-none-eabi- 需在 PATH) ----
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}ranlib)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

# 交叉编译器无 try-run; 静态库/链接脚本由工程接 (同 starm-clang 工具链)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---- 架构/浮点 (Cortex-M4F, 覆盖 F3/G4; FPv4-SP-D16) ----
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_STANDARD_DEFAULT 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- 链接脚本 (工程提供, 不硬编码芯片; HardC 只产静态库时不使用) ----
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
if(NOT DEFINED HARDC_LINK_SCRIPT)
  set(HARDC_LINK_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F334XX_FLASH.ld")
endif()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${HARDC_LINK_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")