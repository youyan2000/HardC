# =====================================================================
#  c2000-ti-cgt.cmake — TI cl2000 (C2000 CGT) 工具链文件
#
#  C2000 生态无官方 CMake 支持 (CCS 用 gmake). 本文件由 C-OOP 自建,
#  标志学自 CCS gmake 的编译行 (RELEASE/subdir_rules.mk):
#     cl2000 -v28 -ml -mt --cla_support=cla2 --float_support=fpu32
#            --tmu_support=tmu0 --vcu_support=vcu0 -O3 --abi=eabi
#            --float_operations_allowed=32 ...
#
#  用法:
#     cmake -S . -B build \
#       -DCMAKE_TOOLCHAIN_FILE=<C-OOP>/cmake/c2000-ti-cgt.cmake \
#       -DC2000_CGT_ROOT="<TI CCS 安装>/ccs/tools/compiler/ti-cgt-c2000_<ver>"
#
#  若 cl2000 的 CMake 探测/try-compile 失败 (STATIC_LIBRARY 目标类型不可用),
#  C-OOP 会退化为生成 coop.makefile 片段交由 CCS gmake 纳入 (见 ymac_cfg 文档).
# =====================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR TMS320F28xxx)

# C2000 无 try-run; 只链接静态库, 规避交叉编译器探测失败
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---- 工具链根目录 ----
if(NOT DEFINED C2000_CGT_ROOT)
  if(DEFINED ENV{TI_CGT_C2000_ROOT})
    set(C2000_CGT_ROOT "$ENV{TI_CGT_C2000_ROOT}")
  elseif(EXISTS "E:/TIIDE/CCS 21.0.0/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS")
    set(C2000_CGT_ROOT "E:/TIIDE/CCS 21.0.0/ccs/tools/compiler/ti-cgt-c2000_25.11.1.LTS")
  elseif(EXISTS "E:/TIIDE/CCS 20.0.1/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS")
    set(C2000_CGT_ROOT "E:/TIIDE/CCS 20.0.1/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS")
  else()
    message(FATAL_ERROR "c2000-ti-cgt.cmake: 未找到 cl2000, 请 -DC2000_CGT_ROOT=<TI CCS 工具链根>")
  endif()
endif()

set(C2000_CGT_BIN "${C2000_CGT_ROOT}/bin")
# 显式带 .exe: CMake 的编译器存在性检查不做扩展名补全, 裸 cl2000 会被判不存在
# (TI 也出 Linux 版, 需要时改回 cl2000; 本工具链面向 Windows CCS 环境)
set(CMAKE_C_COMPILER   "${C2000_CGT_BIN}/cl2000.exe")
set(CMAKE_AR           "${C2000_CGT_BIN}/ar2000.exe")

# cl2000 在给了 -I 后不自带搜索自身 include 目录 (实测 driverlib adc.h 的
# <stdbool.h> 找不到) → 显式加 CGT include/. 必须走 include_directories:
# 路径含空格 (E:/TIIDE/CCS 21.0.0/...), 塞进 -I 旗标会被 Ninja 按空格拆开 → 编译器
# 误当源文件. CMake 的 include 机制自带空格路径转义. (CCS gmake 由 makefile 隐式带)
include_directories("${C2000_CGT_ROOT}/include")

# cl2000 是"编译"驱动 (加 -z 才链接); 这里只做编译, 链接由 coop 目标/外部工程负责
set(CMAKE_C_OUTPUT_EXTENSION .obj)

# 统一架构标志 (C28x FPU32 + TMU0 + VCU0 + CLA2). --c99 为 cl2000 的 C99 开关
# (CMake 的 C_STANDARD=99 对 TI 编译器不注入 -std=, 必须显式给 --c99)
# CGT include/ 目录由上方 include_directories 提供 (空格路径不能塞进 -I 旗标)
set(C2000_BASE_FLAGS
  --c99
  -v28 -ml -mt
  --cla_support=cla2
  --float_support=fpu32
  --tmu_support=tmu0
  --vcu_support=vcu0
  --float_operations_allowed=32
  --abi=eabi
  --diag_suppress=10063
  --diag_warning=225
  --diag_wrap=off
  --display_error_number)

string(REPLACE ";" " " CMAKE_C_FLAGS_INIT "${C2000_BASE_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O3 --opt_for_speed=5 --fp_mode=relaxed")

set(CMAKE_C_STANDARD_DEFAULT 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 编译行 (自定义规则下 CMake 只替换 <SOURCE>, 不再追加 -c)
set(CMAKE_C_COMPILE_OBJECT
  "<CMAKE_C_COMPILER> <FLAGS> <DEFINES> <INCLUDES> --compile_only <SOURCE>")

# 静态库打包 (cl2000 不直接出 .a; 用 ar2000)
set(CMAKE_C_CREATE_STATIC_LIBRARY
  "<CMAKE_AR> -r <TARGET> <OBJECTS>")
