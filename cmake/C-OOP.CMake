# =====================================================================
#  C-OOP.CMake — C-OOP 库接入文件 (对标 LibXR.CMake)
#
#  外部工程在 CMakeLists.txt 中:
#     set(C_OOP_DIR  Middlewares/Third_Party/C-OOP)   # git submodule 路径
#     set(C_OOP_DRIVER st)        # st | c2000 | none
#     set(C_OOP_DEVICES "Devices/adc/adc_dc_sampler.c;Devices/pwm/pwm_buckboost.c")
#                                 # 拓扑闭包选出的 Device 源 (相对 C_OOP_DIR)
#     set(C2000_SDK_DIR "...")    # c2000 时: C2000Ware / DigitalPower SDK 根
#     include(${C_OOP_DIR}/cmake/C-OOP.CMake)
#     target_link_libraries(${PROJECT_NAME} PRIVATE coop)
#
#  职责 (逐条对标 LibXR.CMake):
#    1. 平台分发: C_OOP_DRIVER → 平台库链接 + BSP 后端选择
#    2. SDK 路径校验: 不存在 → FATAL_ERROR
#    3. SysConfig 产物发现: c2000 → RELEASE/syscfg/board.c + c2000ware_libraries.c
#    4. C-OOP 源聚合: 核心 (Components+Module+App/pid_tune) + BSP 后端 + Devices 闭包
#    5. include 目录: 各层 + 平台 SDK 头
# =====================================================================

if(NOT DEFINED C_OOP_DIR)
  # 独立构建 (根 CMakeLists.txt): cmake/ 的上一级
  get_filename_component(C_OOP_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

if(NOT DEFINED C_OOP_DRIVER)
  set(C_OOP_DRIVER none)
endif()

# ---- 0. 平台 SDK 路径校验 (对标 LibXR 的 SDK 检查) ----
if(C_OOP_DRIVER STREQUAL "c2000")
  if(NOT DEFINED C2000_SDK_DIR)
    message(FATAL_ERROR "C-OOP.CMake: C_OOP_DRIVER=c2000 需要 set(C2000_SDK_DIR <C2000Ware/DigitalPower SDK 根>)")
  endif()
  if(NOT EXISTS "${C2000_SDK_DIR}")
    message(FATAL_ERROR "C-OOP.CMake: C2000_SDK_DIR 不存在: ${C2000_SDK_DIR}")
  endif()
elseif(NOT C_OOP_DRIVER STREQUAL "st" AND NOT C_OOP_DRIVER STREQUAL "none")
  message(FATAL_ERROR "C-OOP.CMake: C_OOP_DRIVER 取值非法: ${C_OOP_DRIVER} (支持 st/c2000/none)")
endif()

# ---- 1. 源聚合 ----
# 核心层 (HAL-free): Components + Module + App/pid_tune (App 的 app_main 由外部工程生成)
file(GLOB_RECURSE C_OOP_CORE_SOURCES
  "${C_OOP_DIR}/Components/**/*.c"
  "${C_OOP_DIR}/Module/**/*.c"
  "${C_OOP_DIR}/App/pid_tune.c")

# BSP 后端选择 (平台加速, 按 C_OOP_DRIVER 编译对应实现)
set(C_OOP_BSP_SOURCES "")
if(C_OOP_DRIVER STREQUAL "st")
  # bsp_watchdog_stm32.c 内部以 HAL_IWDG_MODULE_ENABLED 自门控 (LibXR 同款):
  # 未启用 IWDG 的工程编译为空体, 不引入依赖
  foreach(_bsp_f bsp_delay.c bsp_hrtim.c bsp_adc_stm32.c bsp_watchdog_stm32.c)
    if(EXISTS "${C_OOP_DIR}/BSP/${_bsp_f}")
      list(APPEND C_OOP_BSP_SOURCES "${C_OOP_DIR}/BSP/${_bsp_f}")
    endif()
  endforeach()
elseif(C_OOP_DRIVER STREQUAL "c2000")
  # bsp_watchdog_c2000.c 直接 include driverlib.h (F28004x 无 WdtRegs 全局),
  # 由下方 c2000 include 目录 + --preinclude=driverlib.h 提供 — 与 epwm/adc 后端同约定
  foreach(_bsp_f bsp_c2000_epwm.c bsp_c2000_adc.c bsp_watchdog_c2000.c)
    if(EXISTS "${C_OOP_DIR}/BSP/${_bsp_f}")
      list(APPEND C_OOP_BSP_SOURCES "${C_OOP_DIR}/BSP/${_bsp_f}")
    endif()
  endforeach()
endif()

# Devices 闭包 (拓扑选择, 由 gen_app/cmake_integrate 计算后传入)
set(C_OOP_DEVICE_SOURCES "")
foreach(_dev IN LISTS C_OOP_DEVICES)
  list(APPEND C_OOP_DEVICE_SOURCES "${C_OOP_DIR}/${_dev}")
endforeach()

# SysConfig 产物发现 (c2000 → RELEASE/syscfg/; 外部工程勿再单独编译)
set(C_OOP_SYSCFG_SOURCES "")
if(C_OOP_DRIVER STREQUAL "c2000")
  foreach(_cfg_c board.c c2000ware_libraries.c)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE/syscfg/${_cfg_c}")
      list(APPEND C_OOP_SYSCFG_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE/syscfg/${_cfg_c}")
    endif()
  endforeach()
endif()

add_library(coop STATIC
  ${C_OOP_CORE_SOURCES}
  ${C_OOP_BSP_SOURCES}
  ${C_OOP_DEVICE_SOURCES}
  ${C_OOP_SYSCFG_SOURCES})

# ---- 2. include 目录 (PUBLIC, 供外部 app_main.c 使用) ----
file(GLOB C_OOP_COMPONENT_DIRS "${C_OOP_DIR}/Components/*")
file(GLOB C_OOP_DEVICE_DIRS     "${C_OOP_DIR}/Devices/*")
file(GLOB C_OOP_MODULE_DIRS     "${C_OOP_DIR}/Module/*")
target_include_directories(coop PUBLIC
  "${C_OOP_DIR}/BSP"
  ${C_OOP_COMPONENT_DIRS}
  ${C_OOP_DEVICE_DIRS}
  ${C_OOP_MODULE_DIRS}
  "${C_OOP_DIR}/App")

# ---- 3. 平台库链接 + 编译选项 ----
if(C_OOP_DRIVER STREQUAL "st")
  # STM32: 链 CubeMX 平台目标 (同 LibXR target_link_libraries(xr PUBLIC stm32cubemx))
  if(NOT TARGET stm32cubemx)
    message(FATAL_ERROR "C-OOP.CMake: C_OOP_DRIVER=st 需要先定义 stm32cubemx 目标 (CubeMX add_subdirectory)")
  endif()
  target_link_libraries(coop PUBLIC stm32cubemx)

  # STM32 系列选择: 驱动 BSP/bsp_stm32_hal.h 的系列 shim
  if(NOT DEFINED C_OOP_STM32_SERIES)
    set(C_OOP_STM32_SERIES F3)
  endif()
  target_compile_definitions(coop PUBLIC C_OOP_STM32_${C_OOP_STM32_SERIES})

elseif(C_OOP_DRIVER STREQUAL "c2000")
  # C2000: 链 driverlib + 数字电源库 (参考 RELEASE/makefile 的 -l 列表)
  # 路径约定: <SDK>/driverlib/f28004x/driverlib + <SDK>/libraries/{CLAmath,DCL,FPUfastRTS,SFO,sfra}
  # (device_support 头目录也列入了 include, 兼容两种 SDK 布局; 不存在时 CMake 自动忽略)
  set(C2000_DEVICE_DIR "${C2000_SDK_DIR}/driverlib/f28004x/driverlib")
  target_include_directories(coop PUBLIC
    "${C2000_SDK_DIR}/driverlib/f28004x/driverlib"
    "${C2000_SDK_DIR}/device_support/f28004x/common/include"
    "${C2000_SDK_DIR}/device_support/f28004x/headers/include")
  # 预编译库实际位置 (C2000Ware 26.x 布局):
  #   driverlib: driverlib/f28004x/driverlib/ccs/Release/driverlib_eabi.lib
  #   CLAmath/FPUfastRTS: libraries/math/<名>/c28/lib/*.lib
  foreach(_lib
      "${C2000_DEVICE_DIR}/ccs/Release/driverlib_eabi.lib"
      "${C2000_SDK_DIR}/libraries/math/CLAmath/c28/lib/cla2_math_library_fpu32_eabi.lib"
      "${C2000_SDK_DIR}/libraries/math/FPUfastRTS/c28/lib/rts2800_fpu32_fast_supplement_eabi.lib")
    if(EXISTS "${_lib}")
      target_link_libraries(coop PRIVATE "${_lib}")
    endif()
  endforeach()

  # cl2000 编译选项 (学自 CCS gmake: -v28 -ml -mt --abi=eabi --float_support=fpu32 ...)
  # --preinclude=driverlib.h 为库 TU 注入 uint8_t (C28x 字寻址, CGT <stdint.h> 不定义 uint8_t,
  #   仅 driverlib hw_types.h 提供 `typedef uint16_t uint8_t`; 库头不 include 平台库 → 构建期注入)
  target_compile_options(coop PRIVATE
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
  target_compile_definitions(coop PUBLIC
    _TI_EABI_ F28x_DEVICE CPU1 LARGE_MODEL _FLASH)

endif()

# ---- 4. 标准 ----
set_target_properties(coop PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON)
