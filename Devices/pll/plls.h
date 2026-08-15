// PLL 全局句柄 —— 应用层通过此文件访问所有锁相环实例
// 遵循分层架构: Application → Module → Devices → Components → BSP
//
// 用法:
//   #include "plls.h"
//   PllInput in = { .v = grid_v };         // 单相
//   float fo = pll_run(g_pll_grid, &in);   // 锁定频率 (Hz)
//   float theta = pll_get_theta(g_pll_grid);
//
// 三相 (SRF): in.v_alpha / in.v_beta 由外部 Clarke 得到
// 三相不平衡 (DDSRF): in.d_p/d_n/q_p/q_n 由外部正负序 Park 得到

#ifndef PLLS_H
#define PLLS_H

#include "comp_pll_base.h"

// 全局 PLL 句柄 (由 board_init.c 绑定到具体子类实例)
extern PllBase *g_pll_grid;      // 并网锁相 (SRF-PLL / SOGI-PLL)
extern PllBase *g_pll_single;    // 单相电网锁相 (SOGI-PLL / Notch-PLL / SOGI-FLL)
extern PllBase *g_pll_unbal;     // 三相不平衡锁相 (DDSRF-PLL)

// 用户扩展句柄
extern PllBase *g_pll_user0;
extern PllBase *g_pll_user1;

#endif
