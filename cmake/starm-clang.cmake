# =====================================================================
#  starm-clang.cmake — ST ARM Clang (starm-clang) 工具链文件
#
#  对标 LibXR/bsp-dev-c 的 cmake/starm-clang.cmake (工具链放库内分发),
#  按 HardC 库定位适配: 支持静态库 (scaffold 骨架) 与可执行 (外部
#  CubeMX 工程) 两种目标; 链接脚本不硬编码芯片, 由工程用
#  -DHARDC_LINK_SCRIPT=<.ld 路径> 提供 (缺省指向 ${CMAKE_SOURCE_DIR})。
#
#  用法:
#     cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<HardC>/cmake/starm-clang.cmake \
#       -DSTARM_TOOLCHAIN_CONFIG=STARM_PICOLIBC   # 见下
#
#  STARM_TOOLCHAIN_CONFIG 三种配置 (继承 ST 官方/bsd-dev-c 约定):
#    STARM_HYBRID   : starm-clang 汇编/编译 + GNU 链接
#    STARM_NEWLIB   : NEWLIB C 库
#    STARM_PICOLIBC : PICOLIBC C 库 (默认)
# =====================================================================

set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID Clang)
set(CMAKE_CXX_COMPILER_ID Clang)

# ---- 工具链根目录 (starm- 前缀, ST ARM Compass 安装到 PATH) ----
set(TOOLCHAIN_PREFIX starm-)
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}clang)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}clang++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}clang)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

# 交叉编译器无 try-run; 静态库/链接脚本由工程接 (同 HardC gcc 工具链)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---- STARM_TOOLCHAIN_CONFIG (学 bsp-dev-c 多配置) ----
if(NOT DEFINED STARM_TOOLCHAIN_CONFIG)
  set(STARM_TOOLCHAIN_CONFIG "STARM_PICOLIBC")
endif()

if(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_HYBRID")
  set(TOOLCHAIN_MULTILIBS "--multi-lib-config=\"$ENV{CLANG_GCC_CMSIS_COMPILER}/multilib.gnu_tools_for_stm32.yaml\" --gcc-toolchain=\"$ENV{GCC_TOOLCHAIN_ROOT}/..\"")
elseif(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_NEWLIB")
  set(TOOLCHAIN_MULTILIBS "--config=newlib.cfg")
endif()

# ---- 架构/浮点 (Cortex-M4F, FPv4-SP-D16) ----
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ${TOOLCHAIN_MULTILIBS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MP")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Oz -g0")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Oz -g0")

set(CMAKE_C_STANDARD_DEFAULT 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- 链接脚本 (工程提供, 不硬编码芯片; HardC 只产静态库时不使用) ----
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
if(NOT DEFINED HARDC_LINK_SCRIPT)
  set(HARDC_LINK_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F334XX_FLASH.ld")
endif()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${HARDC_LINK_SCRIPT}\"")

if(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_HYBRID")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --gcc-specs=nano.specs")
  set(TOOLCHAIN_LINK_LIBRARIES "m")
elseif(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_NEWLIB")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lcrt0-nosys")
elseif(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_PICOLIBC")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lcrt0-hosted -z norelro")
endif()

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
