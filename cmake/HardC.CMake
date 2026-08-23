# =====================================================================
#  HardC.CMake — HardC 库接入文件 (对标 LibXR.CMake)
#
#  外部工程在 CMakeLists.txt 中:
#     set(HARDC_DIR  Middlewares/Third_Party/HardC)   # git submodule 路径
#     set(HARDC_DRIVER st)        # st | c2000 | none
#     set(HARDC_DEVICES "Devices/adc/adc_dc_sampler.c;Devices/pwm/pwm_buckboost.c")
#                                 # 拓扑闭包选出的 Device 源 (相对 HARDC_DIR)
#     set(C2000_SDK_DIR "...")    # c2000 时: C2000Ware / DigitalPower SDK 根
#     include(${HARDC_DIR}/cmake/HardC.CMake)
#     target_link_libraries(${PROJECT_NAME} PRIVATE hardc)
#
#  职责 (逐条对标 LibXR.CMake):
#    1. 平台分发: HARDC_DRIVER → 平台库链接 + BSP 后端选择
#    2. SDK 路径校验: 不存在 → FATAL_ERROR
#    3. SysConfig 产物发现: c2000 → RELEASE/syscfg/board.c + c2000ware_libraries.c
#    4. HardC 源聚合: 核心 (Components+Module+App/pid_tune) + BSP 后端 + Devices 闭包
#    5. include 目录: 各层 + 平台 SDK 头
# =====================================================================

if(NOT DEFINED HARDC_DIR)
  # 独立构建 (根 CMakeLists.txt): cmake/ 的上一级
  get_filename_component(HARDC_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

if(NOT DEFINED HARDC_DRIVER)
  set(HARDC_DRIVER none)
endif()

# ---- 0. 平台 SDK 路径校验 (对标 LibXR 的 SDK 检查) ----
if(HARDC_DRIVER STREQUAL "c2000")
  if(NOT DEFINED C2000_SDK_DIR)
    message(FATAL_ERROR "HardC.CMake: HARDC_DRIVER=c2000 需要 set(C2000_SDK_DIR <C2000Ware/DigitalPower SDK 根>)")
  endif()
  if(NOT EXISTS "${C2000_SDK_DIR}")
    message(FATAL_ERROR "HardC.CMake: C2000_SDK_DIR 不存在: ${C2000_SDK_DIR}")
  endif()
elseif(NOT HARDC_DRIVER STREQUAL "st" AND NOT HARDC_DRIVER STREQUAL "none")
  message(FATAL_ERROR "HardC.CMake: HARDC_DRIVER 取值非法: ${HARDC_DRIVER} (支持 st/c2000/none)")
endif()

# ---- 1. 源聚合 ----
# 核心层 (HAL-free): Components + Module + App/pid_tune (App 的 app_main 由外部工程生成)
file(GLOB_RECURSE HARDC_CORE_SOURCES
  "${HARDC_DIR}/Components/**/*.c"
  "${HARDC_DIR}/Module/**/*.c"
  "${HARDC_DIR}/App/pid_tune.c")

# BSP 后端选择 (平台加速, 按 HARDC_DRIVER 编译对应实现)
set(HARDC_BSP_SOURCES "")
if(HARDC_DRIVER STREQUAL "st")
  # bsp_watchdog_stm32.c 内部以 HAL_IWDG_MODULE_ENABLED 自门控 (LibXR 同款):
  # 未启用 IWDG 的工程编译为空体, 不引入依赖
  foreach(_bsp_f bsp_delay.c bsp_hrtim.c bsp_adc_stm32.c bsp_gpio_stm32.c bsp_watchdog_stm32.c bsp_uart_stm32.c bsp_i2c_stm32.c bsp_spi_stm32.c bsp_can_stm32.c bsp_flash_stm32.c bsp_jump_stm32.c)
    if(EXISTS "${HARDC_DIR}/BSP/${_bsp_f}")
      list(APPEND HARDC_BSP_SOURCES "${HARDC_DIR}/BSP/${_bsp_f}")
    endif()
  endforeach()
elseif(HARDC_DRIVER STREQUAL "c2000")
  # bsp_watchdog_c2000.c 直接 include driverlib.h (F28004x 无 WdtRegs 全局),
  # 由下方 c2000 include 目录 + --preinclude=driverlib.h 提供 — 与 epwm/adc 后端同约定
  foreach(_bsp_f bsp_c2000_epwm.c bsp_c2000_adc.c bsp_watchdog_c2000.c bsp_uart_c2000.c bsp_spi_c2000.c bsp_can_c2000.c bsp_flash_c2000_fixed.c bsp_jump_c2000.c)
    if(EXISTS "${HARDC_DIR}/BSP/${_bsp_f}")
      list(APPEND HARDC_BSP_SOURCES "${HARDC_DIR}/BSP/${_bsp_f}")
    endif()
  endforeach()
endif()

# Devices 闭包 (拓扑选择, 由 gen_app/cmake_integrate 计算后传入)
set(HARDC_DEVICE_SOURCES "")
foreach(_dev IN LISTS HARDC_DEVICES)
  list(APPEND HARDC_DEVICE_SOURCES "${HARDC_DIR}/${_dev}")
endforeach()

# SysConfig 产物发现 (c2000 → RELEASE/syscfg/; 外部工程勿再单独编译)
set(HARDC_SYSCFG_SOURCES "")
if(HARDC_DRIVER STREQUAL "c2000")
  foreach(_cfg_c board.c c2000ware_libraries.c)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE/syscfg/${_cfg_c}")
      list(APPEND HARDC_SYSCFG_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE/syscfg/${_cfg_c}")
    endif()
  endforeach()
endif()

add_library(hardc STATIC
  ${HARDC_CORE_SOURCES}
  ${HARDC_BSP_SOURCES}
  ${HARDC_DEVICE_SOURCES}
  ${HARDC_SYSCFG_SOURCES})

# ---- 2. include 目录 (PUBLIC, 供外部 app_main.c 使用) ----
file(GLOB HARDC_COMPONENT_DIRS "${HARDC_DIR}/Components/*")
file(GLOB HARDC_DEVICE_DIRS     "${HARDC_DIR}/Devices/*")
file(GLOB HARDC_MODULE_DIRS     "${HARDC_DIR}/Module/*")
target_include_directories(hardc PUBLIC
  "${HARDC_DIR}/BSP"
  ${HARDC_COMPONENT_DIRS}
  ${HARDC_DEVICE_DIRS}
  ${HARDC_MODULE_DIRS}
  "${HARDC_DIR}/App")

# ---- 3. 平台库链接 + 编译选项 ----
if(HARDC_DRIVER STREQUAL "st")
  # STM32: 链 CubeMX 平台目标 (同 LibXR target_link_libraries(xr PUBLIC stm32cubemx))
  if(NOT TARGET stm32cubemx)
    message(FATAL_ERROR "HardC.CMake: HARDC_DRIVER=st 需要先定义 stm32cubemx 目标 (CubeMX add_subdirectory)")
  endif()
  target_link_libraries(hardc PUBLIC stm32cubemx)

  # STM32 系列选择: 驱动 BSP/bsp_stm32_hal.h 的系列 shim
  if(NOT DEFINED HARDC_STM32_SERIES)
    set(HARDC_STM32_SERIES F3)
  endif()
  target_compile_definitions(hardc PUBLIC HARDC_STM32_${HARDC_STM32_SERIES})

elseif(HARDC_DRIVER STREQUAL "c2000")
  # C2000: 链 driverlib + 数字电源库 (参考 RELEASE/makefile 的 -l 列表)
  # 路径约定: <SDK>/driverlib/f28004x/driverlib + <SDK>/libraries/{CLAmath,DCL,FPUfastRTS,SFO,sfra}
  # (device_support 头目录也列入了 include, 兼容两种 SDK 布局; 不存在时 CMake 自动忽略)
  set(C2000_DEVICE_DIR "${C2000_SDK_DIR}/driverlib/f28004x/driverlib")
  target_include_directories(hardc PUBLIC
    "${C2000_SDK_DIR}/driverlib/f28004x/driverlib"
    "${C2000_SDK_DIR}/device_support/f28004x/common/include"
    "${C2000_SDK_DIR}/device_support/f28004x/headers/include"
    "${C2000_SDK_DIR}/libraries/flash_api/f28004x/include/FlashAPI")
  # 预编译库实际位置 (C2000Ware 26.x 布局):
  #   driverlib: driverlib/f28004x/driverlib/ccs/Release/driverlib_eabi.lib
  #   CLAmath/FPUfastRTS: libraries/math/<名>/c28/lib/*.lib
  foreach(_lib
      "${C2000_DEVICE_DIR}/ccs/Release/driverlib_eabi.lib"
      "${C2000_SDK_DIR}/libraries/math/CLAmath/c28/lib/cla2_math_library_fpu32_eabi.lib"
      "${C2000_SDK_DIR}/libraries/math/FPUfastRTS/c28/lib/rts2800_fpu32_fast_supplement_eabi.lib")
    if(EXISTS "${_lib}")
      target_link_libraries(hardc PRIVATE "${_lib}")
    endif()
  endforeach()

  # F021 Flash API (bsp_flash_c2000_fixed.c 依赖). 只链 eabi+fpu32 变体 (匹配上方
  # --abi=eabi --float_support=fpu32); lib/ 下还含 FPU32(非 eabi) / ROM API 等互斥变体,
  # 不能 GLOB 全链. 头在 include/FlashAPI/ 下, 见上方 include 目录.
  set(C2000_F021_LIB
    "${C2000_SDK_DIR}/libraries/flash_api/f28004x/lib/F021_API_F28004x_FPU32_EABI.lib")
  if(EXISTS "${C2000_F021_LIB}")
    target_link_libraries(hardc PRIVATE "${C2000_F021_LIB}")
    # 链接了 F021 库 → 定义 C2000_FLASH_HAS_F021=1, 使 bsp_flash_c2000_fixed.c
    # 的 F021 初始化路径真正编译进来 (未定义时宏默认 0, 初始化被 #if 跳过).
    target_compile_definitions(hardc PRIVATE C2000_FLASH_HAS_F021=1)
  endif()

  # cl2000 编译选项 (学自 CCS gmake: -v28 -ml -mt --abi=eabi --float_support=fpu32 ...)
  # --preinclude=driverlib.h 为库 TU 注入 uint8_t (C28x 字寻址, CGT <stdint.h> 不定义 uint8_t,
  #   仅 driverlib hw_types.h 提供 `typedef uint16_t uint8_t`; 库头不 include 平台库 → 构建期注入)
  target_compile_options(hardc PRIVATE
    -v28 -ml -mt
    --abi=eabi
    --float_support=fpu32
    --preinclude=driverlib.h
    --tmu_support=tmu0
    --vcu_support=vcu0
    --cla_support=cla2
    --float_operations_allowed=32
    --diag_suppress=10063
    --diag_warning=225
    --diag_wrap=off
    --display_error_number)
  target_compile_definitions(hardc PUBLIC
    _TI_EABI_ F28x_DEVICE CPU1 LARGE_MODEL _FLASH)

endif()

# ---- 4. 标准 ----
set_target_properties(hardc PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON)
