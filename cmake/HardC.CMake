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
#
# 裁剪 (对标 precise-select): 库默认全量 glob 编译 (Components+Module 通编, 保目录全集
#   供未来任意拓扑选用). 需要按拓扑裁剪时可显式给出允许子目录白名单:
#     set(HARDC_MODULES     "motor;power"   )  # Module/ 下子目录, ; 或空格分隔
#     set(HARDC_COMPONENTS  "pid;adc;pwm"   )  # Components/ 下子目录
#   未定义 → 保持全量 (后向兼容). 二者独立, 只裁设置了的那一层.
#   Bootloader: 默认不编，是独立固件/Opt-in (Buck 等 App 工程不需要). 需要时
#   外部工程 set(HARDC_ENABLE_BOOTLOADER ON) 显式打开.
file(GLOB_RECURSE HARDC_CORE_SOURCES
  "${HARDC_DIR}/Components/**/*.c"
  "${HARDC_DIR}/Module/**/*.c"
  "${HARDC_DIR}/App/pid_tune.c")

# Module/ + Components/ 子目录白名单裁剪.
#   HARDC_MODULES="motor;power"    → 只编 Module/motor + Module/power
#   HARDC_COMPONENTS="pid;adc;pwm" → 只编 Components/pid + Components/adc + Components/pwm
#   未定义某一层 → 该层保持全量; App/pid_tune.c 恒保留.
if(DEFINED HARDC_MODULES OR DEFINED HARDC_COMPONENTS)
  set(_hardc_core_keep "(.*/App/pid_tune\\.c)")
  if(DEFINED HARDC_MODULES)
    foreach(_m IN LISTS HARDC_MODULES)
      list(APPEND _hardc_core_keep "^${HARDC_DIR}/[Mm]odule/${_m}/")
    endforeach()
  endif()
  if(DEFINED HARDC_COMPONENTS)
    foreach(_c IN LISTS HARDC_COMPONENTS)
      list(APPEND _hardc_core_keep "^${HARDC_DIR}/[Cc]omponents/${_c}/")
    endforeach()
  endif()
  # 拼接成正则轮询 (用 |) 后 INCLUDE(任一匹配即保留)
  string(JOIN "|" _hardc_keep_regex ${_hardc_core_keep})
  list(FILTER HARDC_CORE_SOURCES INCLUDE REGEX "${_hardc_keep_regex}")
endif()

# Bootloader 独立固件: 默认裁剪 (需 HARDC_ENABLE_BOOTLOADER=ON 才编)
if(NOT HARDC_ENABLE_BOOTLOADER)
  list(FILTER HARDC_CORE_SOURCES EXCLUDE REGEX ".*/[Mm]odule/[Bb]ootloader/.*")
endif()

# BSP 后端选择 (平台加速, 按 HARDC_DRIVER + HARDC_BSP 编译对应实现)
#
# 裁剪原则 (对标 LibXR 精确选真): 不把整个 BSP 后端全量编进库, 只编拓扑/外设实际
# 需要的那几个后端, 避免撞上 CubeMX 未启用外设 (CAN/SPI/UART/I2C 未在 .ioc 使能时,
# 其 HAL 头如 CAN_HandleTypeDef 不存在 → 对应 bsp_*_stm32.c 编译失败)。
# 外部工程通过 YmaC cmake_integrate 注入 `set(HARDC_BSP "...")` 指定要编的后端文件
# (空格分隔, 相对 HARDC_DIR/BSP), 缺省回退到平台默认最小集 (保持后向兼容)。
set(HARDC_BSP_SOURCES "")
if(HARDC_DRIVER STREQUAL "st")
  if(NOT DEFINED HARDC_BSP)
    # 缺省最小集: 通用 + 内建常用 (delay/irq/watchdog), 不含 comm/bootloader 后端 (按需由 HARDC_BSP 显式加)
    set(HARDC_BSP "bsp_delay.c bsp_hrtim.c bsp_adc_stm32.c bsp_gpio_stm32.c bsp_irq_stm32.c bsp_watchdog_stm32.c")
  endif()
  foreach(_bsp_f ${HARDC_BSP})
    if(EXISTS "${HARDC_DIR}/BSP/${_bsp_f}")
      list(APPEND HARDC_BSP_SOURCES "${HARDC_DIR}/BSP/${_bsp_f}")
    endif()
  endforeach()
elseif(HARDC_DRIVER STREQUAL "c2000")
  if(NOT DEFINED HARDC_BSP)
    set(HARDC_BSP "bsp_c2000_epwm.c bsp_c2000_adc.c bsp_watchdog_c2000.c")
  endif()
  foreach(_bsp_f ${HARDC_BSP})
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
# 与 HARDC_COMPONENTS / HARDC_MODULES 白名单保持一致 (避免被裁剪模块的头泄漏到 include 搜索路径)
if(DEFINED HARDC_COMPONENTS)
  foreach(_c IN LISTS HARDC_COMPONENTS)
    list(APPEND _hardc_comp_keep_dir "${HARDC_DIR}/Components/${_c}")
  endforeach()
  if(_hardc_comp_keep_dir)
    # 只保留白名单列出的子目录; 未列出的全部移除
    set(_hardc_comp_all "${HARDC_COMPONENT_DIRS}")
    set(HARDC_COMPONENT_DIRS "")
    foreach(_dir IN LISTS _hardc_comp_all)
      list(FIND _hardc_comp_keep_dir "${_dir}" _idx)
      if(NOT _idx EQUAL -1)
        list(APPEND HARDC_COMPONENT_DIRS "${_dir}")
      endif()
    endforeach()
  endif()
endif()
if(DEFINED HARDC_MODULES)
  foreach(_m IN LISTS HARDC_MODULES)
    list(APPEND _hardc_mod_keep_dir "${HARDC_DIR}/Module/${_m}")
  endforeach()
  if(_hardc_mod_keep_dir)
    set(_hardc_mod_all "${HARDC_MODULE_DIRS}")
    set(HARDC_MODULE_DIRS "")
    foreach(_dir IN LISTS _hardc_mod_all)
      list(FIND _hardc_mod_keep_dir "${_dir}" _idx)
      if(NOT _idx EQUAL -1)
        list(APPEND HARDC_MODULE_DIRS "${_dir}")
      endif()
    endforeach()
  endif()
endif()
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
