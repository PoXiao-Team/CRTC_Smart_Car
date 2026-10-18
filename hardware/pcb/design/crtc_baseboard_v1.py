#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CRTC2026 · 破晓战队 · 智能车模块化底板 V1
================================================================================
本文件是底板设计的**唯一数据源**：元件表（BOM）、引脚级网表（NETLIST）、
板框与布局坐标、电流预算与走线宽度计算、ERC 电气规则检查、装配核对清单，
全部由这里生成。改设计只改这里，然后重新运行：

    python crtc_baseboard_v1.py            # 生成全部产物 + 打印 ERC 报告
    python crtc_baseboard_v1.py --quiet    # 只生成产物，不打印明细

退出码：0 = ERC 全部通过（可进入布局）；1 = 存在错误（必须先修）。

产物（写到本文件同级的 out/ 目录）：
    BOM.csv              采购 + 嘉立创 EDA 器件搜索清单（含必选参数约束）
    NETLIST.csv          引脚级网表（EDA 连线与 ERC 的唯一依据）
    接线表.md            按模块组织的接线表，画原理图时照着连，打印带去工位
    装配核对表.md        板子回来后逐项蜂鸣档核对清单（装配前必做）
    布局坐标.csv         每个器件的坐标/朝向，EDA 里可直接填入，避免随手乱摆
    线宽与电流预算.md    各电源网络的电流预算与最小线宽（IPC-2221 计算）
    施工单.html          可打印施工单（接线表 + 装配核对，A4 打印）
    ERC报告.txt          电气规则检查报告

设计前提（已固化为常量，改动必须同步改这里）：
    电池 2×18650 串联（2S）→ VBAT 标称 7.4V，满电 8.4V，截止 7.0V
    mini#1 输出 +5V → 只给 STM32 核心板（≤150mA）
    mini#2 输出 VM  → 给两块 DRV8833 的 VM 与舵机，设定 5.0V
    PS2 接收器 3.3V 由 STM32 核心板板载 LDO 提供
"""

from __future__ import annotations

import csv
import math
import os
import sys
from collections import defaultdict

# =============================================================================
# 0. 板级参数
# =============================================================================

BOARD = {
    "name": "CRTC2026_SmartCar_BaseBoard",
    "rev": "V1",
    "author": "破晓战队 / 谭翔珂",
    "outline_mm": (92.0, 72.0),      # 板框（宽 × 高），≤100×100 走免费打样档
    "corner_radius_mm": 3.0,
    "layers": 2,
    "thickness_mm": 1.6,
    "copper_oz": 1.0,
    "mount_hole_d_mm": 3.2,          # M3 螺丝
    "mount_holes": [(4.0, 4.0), (88.0, 4.0), (4.0, 68.0), (88.0, 68.0)],
    "min_trace_mil": 10.0,
    "min_via_mil": 24.0,
    "min_via_drill_mil": 12.0,
    "silk_line_mil": 6.0,
    "silk_height_mil": 45.0,
    "clearance_mil": 8.0,
}

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# 电源轨的电流预算（用于线宽计算与风险判定）
POWER_BUDGET = [
    {
        "net": "VBAT",
        "desc": "电池 → 两块 mini 的输入（全车总电流入口）",
        "normal_a": 2.0,
        "peak_a": 6.5,
        "note": "2S 电池，7.0～8.4V；此网电流 = +5V 路 + VM 路之和",
    },
    {
        "net": "+5V",
        "desc": "mini#1 输出 → STM32 核心板 5V 脚（核心板 LDO 再出 3.3V 给 PS2）",
        "normal_a": 0.15,
        "peak_a": 0.30,
        "note": "核心板 ≈50mA + PS2 ≈10mA + 余量；不得再挂马达/舵机",
    },
    {
        "net": "+3V3",
        "desc": "STM32 核心板板载 LDO 输出 → PS2 接收器 VCC",
        "normal_a": 0.05,
        "peak_a": 0.10,
        "note": "核心板 LDO 能力有限（≈300mA），只带 PS2",
    },
    {
        "net": "VM",
        "desc": "mini#2 输出 → DRV#1/#2 的 VM + 舵机 V + 储能电容",
        "normal_a": 2.0,
        "peak_a": 5.5,
        "note": "4×TT 马达常态约 1.5A、堵转约 5A，SG90 峰值约 0.5A；"
                "mini 模块额定按 1.8A 计 → 常态已贴近上限，见风险 R3",
    },
    {
        "net": "MOTOR_x",
        "desc": "单路马达输出（DRV8833 AO/BO → 马达端子）",
        "normal_a": 0.6,
        "peak_a": 1.5,
        "note": "单通道持续 1.0A 以内是安全的；堵转由软件限流保护",
    },
]

# 供电模块额定值（用于风险判定；型号按采购链接实测值回填到 BOM 的“关键参数”列）
MINI_MODULE_RATING_A = 1.8     # 偏保守的"假设额定"（实物型号待核对）；与 BOM"优选 ≥2A"（采购口径）、对策 A"≥3A"（决策门槛）性质不同，勿混用

# =============================================================================
# 1. 元件表（BOM + 符号引脚定义）
#    pins : 参与连接的引脚名（顺序即符号引脚顺序）
#    nc   : 存在但不连接、且**允许**悬空的引脚（ERC 不报错）
# =============================================================================

COMPONENTS = [
    {
        "ref": "J1",
        "name": "电池/继电器输入端子",
        "model": "KF301-2P 螺钉端子（5.08mm 间距，弯插）",
        "package": "KF301-2P",
        "mount": "板边，接线口朝板外",
        "search": "KF301-2P",
        "spec": "间距 5.08mm；引脚可过 ≥3A；弯插（接线口朝板外）",
        "qty": 1,
        "place": (6.0, 60.0, 0),
        "pins": ["BAT+", "BAT-"],
        "nc": [],
        "note": "官方无线继电器串在电池与此处之间，继电器本体不装在板上",
    },
    {
        "ref": "U1",
        "name": "mini 降压模块 #1（+5V 路）",
        "model": "mini DC-DC 降压模块（可调，输出设为 5.0V）",
        "package": "1x4 排母 2.54mm",
        "mount": "排母插座；模块为四角焊盘（2×2，无插针），用 4 根短线焊到排母上进行连接",
        "search": "排母 1x4 2.54",
        "spec": "1×4 2.54mm 排母；模块本体另购（丝印 OUT+/OUT−/IN+/IN−，型号 V142E 级）",
        "qty": 1,
        "place": (24.0, 62.0, 0),
        "pins": ["IN+", "IN-", "OUT+", "OUT-"],
        "nc": [],
        "note": "上电前必须把输出调到 5.00V（空载调、带载复测）；"
                "实物四角焊盘顺序按丝印核对后焊线（勿凭位置猜正负）",
    },
    {
        "ref": "U2",
        "name": "mini 降压模块 #2（VM 路）",
        "model": "mini DC-DC 降压模块（可调，输出设为 5.0V）",
        "package": "1x4 排母 2.54mm",
        "mount": "排母插座；模块为四角焊盘（2×2，无插针），用 4 根短线焊到排母上进行连接",
        "search": "排母 1x4 2.54",
        "spec": "1×4 2.54mm 排母；模块本体另购（丝印 OUT+/OUT−/IN+/IN−，优选持续 ≥2A）",
        "qty": 1,
        "place": (24.0, 48.0, 0),
        "pins": ["IN+", "IN-", "OUT+", "OUT-"],
        "nc": [],
        "note": "输出设 5.0V（不是 6V）：2S 电池后半程压差不足会导致不稳压",
    },
    {
        "ref": "U3A",
        "name": "STM32 核心板排母 · 左列（20 脚，自上而下）",
        "model": "1×20 排母 2.54mm（核心板实物丝印顺序）",
        "package": "1x20 排母 2.54mm",
        "mount": "排母插座，核心板直插；1 号脚靠 USB 端",
        "search": "排母 1x20 2.54",
        "spec": "1×20 排母；与 U3B 平行，两排中心距卡尺实测（估值 15.24mm）",
        "qty": 1,
        "place": (44.0, 38.0, 0),
        "silk": ["B12", "B13", "B14", "B15", "A8", "A9", "A10", "A11", "A12", "A15",
                  "B3", "B4", "B5", "B6", "B7", "B8", "B9", "5V", "G", "3V3"],
        "pins": ["B12", "A8", "A9", "A10", "A11", "B3", "B4", "B5", "B6", "B7",
                 "5V", "G", "3V3"],
        "nc": ["B13", "B14", "B15", "A12", "A15", "B8", "B9"],
        "note": "实物左列自上而下：B12、B13、B14、B15、A8、A9、A10、A11、A12、A15、"
                "B3、B4、B5、B6、B7、B8、B9、5V、G、3.3 —— "
                "丝印缩略名 B12=PB12、A8=PA8；B3/B4 是 JTAG 复用脚（固件须关 JTAG 保 SWD）",
    },
    {
        "ref": "U3B",
        "name": "STM32 核心板排母 · 右列（20 脚，自上而下）",
        "model": "1×20 排母 2.54mm（核心板实物丝印顺序）",
        "package": "1x20 排母 2.54mm",
        "mount": "排母插座，核心板直插；1 号脚靠 USB 端",
        "search": "排母 1x20 2.54",
        "spec": "1×20 排母；与 U3A 平行，两排中心距卡尺实测（估值 15.24mm）",
        "qty": 1,
        "place": (60.0, 38.0, 0),
        "silk": ["G1", "G2", "3V3", "RES", "B11", "B10", "B1", "B0", "A7", "A6",
                  "A5", "A4", "A3", "A2", "A1", "A0", "C15", "C14", "C13", "VB"],
        "pins": ["G1", "G2", "3V3", "B1", "B0", "A7", "A6", "A3", "A2"],
        "nc": ["RES", "B11", "B10", "A5", "A4", "A1", "A0", "C15", "C14", "C13", "VB"],
        "note": "实物右列自上而下：G、G、3.3、RES、B11、B10、B1、B0、A7、A6、A5、A4、"
                "A3、A2、A1、A0、C15、C14、C13、VB —— "
                "RES 为复位脚、VB 为 VBAT 脚，本设计均不接；两处 G 都要接 GND",
    },
    {
        "ref": "U4A",
        "name": "DRV8833 #1 模块座 · 左列（控制/使能侧）",
        "model": "DRV8833 双 H 桥模块（16 脚，2×8，实测丝印）",
        "package": "1x8 排母 2.54mm",
        "mount": "排母插座，模块直插；与 U4B 平行，列距按实测（默认 10.16mm）",
        "search": "排母 1x8 2.54",
        "spec": "1×8 2.54mm 排母；模块本体另购（红板 2×8 丝印版）",
        "qty": 1,
        "place": (62.0, 56.0, 0),
        "silk": ["NC1", "AIN2", "AIN1", "STBY", "BIN1", "BIN2", "NC2", "GND1"],
        "pins": ["AIN2", "AIN1", "STBY", "BIN1", "BIN2", "GND1"],
        "nc": ["NC1", "NC2"],
        "note": "实物左列自上而下：NC、AIN2、AIN1、STBY、BIN1、BIN2、NC、GND —— "
                "两个 NC 脚（第 1、7 位）悬空不接；第 8 位 GND 必须接 GND 网",
    },
    {
        "ref": "U4B",
        "name": "DRV8833 #1 模块座 · 右列（电源/输出侧）",
        "model": "DRV8833 双 H 桥模块（16 脚，2×8，实测丝印）",
        "package": "1x8 排母 2.54mm",
        "mount": "排母插座，模块直插；VM 与两处 GND 都要接（大电流）",
        "search": "排母 1x8 2.54",
        "spec": "1×8 2.54mm 排母；模块本体另购",
        "qty": 1,
        "place": (72.0, 56.0, 0),
        "silk": ["VM", "NC3", "GND2", "AO1", "AO2", "BO2", "BO1", "GND3"],
        "pins": ["VM", "GND2", "AO1", "AO2", "BO2", "BO1", "GND3"],
        "nc": ["NC3"],
        "note": "实物右列自上而下：VM、NC、GND、AO1、AO2、BO2、BO1、GND —— "
                "注意 BO2 在 BO1 **上方**；VM 单脚要并线加粗，两处 GND 都接到 GND 网",
    },
    {
        "ref": "U5A",
        "name": "DRV8833 #2 模块座 · 左列（控制/使能侧）",
        "model": "DRV8833 双 H 桥模块（16 脚，2×8，实测丝印）",
        "package": "1x8 排母 2.54mm",
        "mount": "排母插座，模块直插；与 U5B 平行，列距按实测（默认 10.16mm）",
        "search": "排母 1x8 2.54",
        "spec": "1×8 2.54mm 排母；模块本体另购",
        "qty": 1,
        "place": (62.0, 38.0, 0),
        "silk": ["NC1", "AIN2", "AIN1", "STBY", "BIN1", "BIN2", "NC2", "GND1"],
        "pins": ["AIN2", "AIN1", "STBY", "BIN1", "BIN2", "GND1"],
        "nc": ["NC1", "NC2"],
        "note": "同 U4A；模块丝印可能是 STBY 或 SLP，都接 DRV_STBY",
    },
    {
        "ref": "U5B",
        "name": "DRV8833 #2 模块座 · 右列（电源/输出侧）",
        "model": "DRV8833 双 H 桥模块（16 脚，2×8，实测丝印）",
        "package": "1x8 排母 2.54mm",
        "mount": "排母插座，模块直插；VM 与两处 GND 都要接",
        "search": "排母 1x8 2.54",
        "spec": "1×8 2.54mm 排母；模块本体另购",
        "qty": 1,
        "place": (72.0, 38.0, 0),
        "silk": ["VM", "NC3", "GND2", "AO1", "AO2", "BO2", "BO1", "GND3"],
        "pins": ["VM", "GND2", "AO1", "AO2", "BO2", "BO1", "GND3"],
        "nc": ["NC3"],
        "note": "同 U4B；BO1/BO2 到端子的接法见 MOTOR_RR_1/_2（镜像）",
    },
    {
        "ref": "C1",
        "name": "VM 储能电容 #1（紧贴 U4）",
        "model": "铝电解 470µF / 16V",
        "package": "CAP-TH_D6.3xH11.0_P2.50",
        "mount": "直插，紧贴 U4 的 VM-GND，丝印画 +",
        "search": "铝电解电容 470uF 16V 直插",
        "spec": "容量 ≥470µF，耐压 ≥16V，直插，脚距 2.5mm",
        "qty": 1,
        "place": (80.0, 56.0, 0),
        "pins": ["+", "-"],
        "nc": [],
        "note": "吸收马达启停的电流尖峰，距 U4 的 VM 脚 ≤5mm",
    },
    {
        "ref": "C2",
        "name": "VM 储能电容 #2（紧贴 U5）",
        "model": "铝电解 470µF / 16V",
        "package": "CAP-TH_D6.3xH11.0_P2.50",
        "mount": "直插，紧贴 U5 的 VM-GND，丝印画 +",
        "search": "铝电解电容 470uF 16V 直插",
        "spec": "容量 ≥470µF，耐压 ≥16V，直插，脚距 2.5mm",
        "qty": 1,
        "place": (80.0, 38.0, 0),
        "pins": ["+", "-"],
        "nc": [],
        "note": "同上；两块驱动各配一颗，不要共用",
    },
    {
        "ref": "C3",
        "name": "+5V 输出滤波电容",
        "model": "铝电解 100µF / 10V",
        "package": "CAP-TH_D5.0xH11.0_P2.00",
        "mount": "直插，靠近 U1 的 OUT+ / OUT-",
        "search": "铝电解电容 100uF 10V 直插",
        "spec": "容量 ≥100µF，耐压 ≥10V，直插",
        "qty": 1,
        "place": (36.0, 64.0, 0),
        "pins": ["+", "-"],
        "nc": [],
        "note": "抑制降压模块纹波，避免核心板复位",
    },
    {
        "ref": "C4",
        "name": "VM 高频去耦电容",
        "model": "瓷片 0.1µF（104）/ 50V",
        "package": "CAP-TH_D4.3xH11.0_P5.08",
        "mount": "直插，跨在 VM-GND 上（与 C1/C2 并联）",
        "search": "瓷片电容 104 0.1uF 直插 5.08",
        "spec": "0.1µF，耐压 ≥25V，直插，脚距 5.08mm",
        "qty": 1,
        "place": (80.0, 48.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "无极性，两脚可互换；负责高频噪声",
    },
    {
        "ref": "C5",
        "name": "+5V 高频去耦电容",
        "model": "瓷片 0.1µF（104）/ 50V",
        "package": "CAP-TH_D4.3xH11.0_P5.08",
        "mount": "直插，跨在 +5V-GND 上",
        "search": "瓷片电容 104 0.1uF 直插 5.08",
        "spec": "0.1µF，耐压 ≥25V，直插，脚距 5.08mm",
        "qty": 1,
        "place": (36.0, 56.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "无极性",
    },
    {
        "ref": "C6",
        "name": "VBAT 输入储能电容",
        "model": "铝电解 100~220µF / 16V（推荐 220µF）",
        "package": "CAP-TH_D6.3xH11.0_P2.50",
        "mount": "直插，靠近 J1 与两块 mini 的 IN",
        "search": "铝电解电容 220uF 16V 直插",
        "spec": "容量 ≥100µF（推荐 220µF），耐压 ≥16V，直插，脚距 2.5mm",
        "qty": 1,
        "place": (14.0, 50.0, 0),
        "pins": ["+", "-"],
        "nc": [],
        "note": "电池端电压会随大电流下拉，这颗负责兜底",
    },
    {
        "ref": "J2",
        "name": "左前马达端子 FL",
        "model": "KF301-2P 螺钉端子（5.08mm，弯插）",
        "package": "KF301-2P",
        "mount": "板下边缘，接线口朝板外",
        "search": "KF301-2P",
        "spec": "间距 5.08mm；可过 ≥2A；必须可插拔（换向靠拔线对调）",
        "qty": 1,
        "place": (10.0, 6.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "丝印标 FL；_1/_2 只是两个脚，正负靠实际转向对调",
    },
    {
        "ref": "J3",
        "name": "右前马达端子 FR",
        "model": "KF301-2P 螺钉端子（5.08mm，弯插）",
        "package": "KF301-2P",
        "mount": "板下边缘，接线口朝板外",
        "search": "KF301-2P",
        "spec": "间距 5.08mm；可过 ≥2A",
        "qty": 1,
        "place": (27.0, 6.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "丝印标 FR",
    },
    {
        "ref": "J4",
        "name": "左后马达端子 RL",
        "model": "KF301-2P 螺钉端子（5.08mm，弯插）",
        "package": "KF301-2P",
        "mount": "板下边缘，接线口朝板外",
        "search": "KF301-2P",
        "spec": "间距 5.08mm；可过 ≥2A",
        "qty": 1,
        "place": (44.0, 6.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "丝印标 RL",
    },
    {
        "ref": "J5",
        "name": "右后马达端子 RR",
        "model": "KF301-2P 螺钉端子（5.08mm，弯插）",
        "package": "KF301-2P",
        "mount": "板下边缘，接线口朝板外",
        "search": "KF301-2P",
        "spec": "间距 5.08mm；可过 ≥2A",
        "qty": 1,
        "place": (61.0, 6.0, 0),
        "pins": ["1", "2"],
        "nc": [],
        "note": "丝印标 RR；本设计用**镜像接法**（BO2→脚1、BO1→脚2），"
                "与你面包板上跑通的方向一致，装车后若方向反了就整组对调",
    },
    {
        "ref": "J6",
        "name": "舵机接口 SG90",
        "model": "1×3 直插排针 2.54mm",
        "package": "HDR-TH_1x3_P2.54",
        "mount": "板右边缘，针朝上；杜邦线直插",
        "search": "排针 1x3 2.54 直插",
        "spec": "1×3 2.54mm 直插排针",
        "qty": 1,
        "place": (84.0, 16.0, 90),
        "silk": ["S", "V", "G"],
        "pins": ["S", "V", "G"],
        "nc": [],
        "note": "丝印标 S/V/G；舵机线序 黄(信号) 红(V) 棕(G)，插反必烧舵机，装配前核对",
    },
    {
        "ref": "J7",
        "name": "PS2 接收器接口",
        "model": "1×6 直插排针 2.54mm",
        "package": "HDR-TH_1x6_P2.54",
        "mount": "板右边缘，针朝上",
        "search": "排针 1x6 2.54 直插",
        "spec": "1×6 2.54mm 直插排针",
        "qty": 1,
        "place": (84.0, 26.0, 90),
        "silk": ["CS", "CMD", "DAT", "CLK", "VCC", "GND"],
        "pins": ["DAT", "CMD", "CS", "CLK", "VCC", "GND"],
        "nc": [],
        "note": "VCC 接 +3V3（不是 +5V）；若转接板只有 5 脚（VCC 与 GND 合并引出）"
                "则改按 5 脚座画，GND 仍接 GND",
    },
    {
        "ref": "J8",
        "name": "调试扩展口（USART2）",
        "model": "1×4 直插排针 2.54mm",
        "package": "HDR-TH_1x4_P2.54",
        "mount": "板右边缘，针朝上",
        "search": "排针 1x4 2.54 直插",
        "spec": "1×4 2.54mm 直插排针",
        "qty": 1,
        "place": (84.0, 36.0, 90),
        "silk": ["TX", "RX", "+5V", "GND"],
        "pins": ["TX", "RX", "+5V", "GND"],
        "nc": [],
        "note": "PA9/PA10 已被电机 PWM 占用，串口调试只能走 USART2(PA2/PA3)；"
                "接 USB-TTL 时 TX↔RX 交叉",
    },
]

# =============================================================================
# 2. 网表（引脚级）—— EDA 连线与 ERC 的唯一依据
# =============================================================================

NETS = [
    # ---------------- 电源 ----------------
    {
        "net": "VBAT", "cat": "电源", "current_a": 2.0, "peak_a": 6.5,
        "endpoints": [("J1", "BAT+"), ("U1", "IN+"), ("U2", "IN+"), ("C6", "+")],
        "note": "电池正（经官方无线继电器）→ 两块 mini 输入；7.0～8.4V",
    },
    {
        "net": "GND", "cat": "电源", "current_a": 2.0, "peak_a": 6.5,
        "endpoints": [
            ("J1", "BAT-"), ("U1", "IN-"), ("U1", "OUT-"), ("U2", "IN-"), ("U2", "OUT-"),
            ("U3A", "G"), ("U3B", "G1"), ("U3B", "G2"),
            ("U4A", "GND1"), ("U4B", "GND2"), ("U4B", "GND3"),
            ("U5A", "GND1"), ("U5B", "GND2"), ("U5B", "GND3"),
            ("C1", "-"), ("C2", "-"), ("C3", "-"), ("C4", "2"), ("C5", "2"), ("C6", "-"),
            ("J6", "G"), ("J7", "GND"), ("J8", "GND"),
        ],
        "note": "全板唯一地网（不得出现 VSS / GNDA / AGND 等别名）；铺铜网络",
    },
    {
        "net": "+5V", "cat": "电源", "current_a": 0.15, "peak_a": 0.30,
        "endpoints": [("U1", "OUT+"), ("U3A", "5V"), ("C3", "+"), ("C5", "1"), ("J8", "+5V")],
        "note": "只给核心板与调试口，禁止挂马达/舵机",
    },
    {
        "net": "+3V3", "cat": "电源", "current_a": 0.05, "peak_a": 0.10,
        "endpoints": [("U3A", "3V3"), ("U3B", "3V3"), ("J7", "VCC")],
        "note": "由核心板板载 LDO 提供，只带 PS2 接收器",
    },
    {
        "net": "VM", "cat": "电源", "current_a": 2.0, "peak_a": 5.5,
        "endpoints": [
            ("U2", "OUT+"), ("U4B", "VM"), ("U5B", "VM"), ("J6", "V"),
            ("C1", "+"), ("C2", "+"), ("C4", "1"),
        ],
        "note": "mini#2 输出 5.0V；严禁与 VBAT 有任何连接；走线按最宽处理",
    },
    # ---------------- 驱动 PWM（前驱 U4） ----------------
    {"net": "DRV_F_AIN1", "cat": "信号", "endpoints": [("U3B", "A6"), ("U4A", "AIN1")],
     "note": "TIM3_CH1；PA6"},
    {"net": "DRV_F_BIN1", "cat": "信号", "endpoints": [("U3B", "A7"), ("U4A", "BIN1")],
     "note": "TIM3_CH2；PA7"},
    {"net": "DRV_F_AIN2", "cat": "信号", "endpoints": [("U3A", "A8"), ("U4A", "AIN2")],
     "note": "TIM1_CH1；PA8（TIM1 必须开 MOE 才有输出）"},
    {"net": "DRV_F_BIN2", "cat": "信号", "endpoints": [("U3A", "A9"), ("U4A", "BIN2")],
     "note": "TIM1_CH2；PA9"},
    # ---------------- 驱动 PWM（后驱 U5） ----------------
    {"net": "DRV_R_AIN1", "cat": "信号", "endpoints": [("U3B", "B0"), ("U5A", "AIN1")],
     "note": "TIM3_CH3；PB0"},
    {"net": "DRV_R_BIN1", "cat": "信号", "endpoints": [("U3B", "B1"), ("U5A", "BIN1")],
     "note": "TIM3_CH4；PB1"},
    {"net": "DRV_R_AIN2", "cat": "信号", "endpoints": [("U3A", "A10"), ("U5A", "AIN2")],
     "note": "TIM1_CH3；PA10"},
    {"net": "DRV_R_BIN2", "cat": "信号", "endpoints": [("U3A", "A11"), ("U5A", "BIN2")],
     "note": "TIM1_CH4；PA11"},
    # ---------------- 使能 ----------------
    {"net": "DRV_STBY", "cat": "信号", "endpoints": [("U3A", "B12"), ("U4A", "STBY"), ("U5A", "STBY")],
     "note": "三点同网；高电平使能，内部 500kΩ 下拉 → MCU 未初始化时电机不转（安全）"},
    # ---------------- PS2 ----------------
    {"net": "PS2_DAT", "cat": "信号", "endpoints": [("U3A", "B6"), ("J7", "DAT")], "note": "软件 SPI，主入"},
    {"net": "PS2_CMD", "cat": "信号", "endpoints": [("U3A", "B5"), ("J7", "CMD")], "note": "软件 SPI，主出"},
    {"net": "PS2_CS", "cat": "信号", "endpoints": [("U3A", "B3"), ("J7", "CS")],
     "note": "PB3 默认 JTAG(JTDO)，必须关 JTAG 保 SWD 才能当普通 IO"},
    {"net": "PS2_CLK", "cat": "信号", "endpoints": [("U3A", "B4"), ("J7", "CLK")],
     "note": "PB4 默认 JTAG(NJTRST)，同上；且必须禁用 TIM3 部分重映射"},
    # ---------------- 舵机 / 调试 ----------------
    {"net": "SRV_PWM", "cat": "信号", "endpoints": [("U3A", "B7"), ("J6", "S")],
     "note": "TIM4_CH2，50Hz（周期 20ms，脉宽 0.5～2.5ms）"},
    {"net": "DBG_TX", "cat": "信号", "endpoints": [("U3B", "A2"), ("J8", "TX")], "note": "USART2_TX"},
    {"net": "DBG_RX", "cat": "信号", "endpoints": [("U3B", "A3"), ("J8", "RX")], "note": "USART2_RX"},
    # ---------------- 马达输出 ----------------
    {"net": "MOTOR_FL_1", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U4B", "AO1"), ("J2", "1")], "note": "左前马达脚 1"},
    {"net": "MOTOR_FL_2", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U4B", "AO2"), ("J2", "2")], "note": "左前马达脚 2"},
    {"net": "MOTOR_FR_1", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U4B", "BO1"), ("J3", "1")], "note": "右前马达脚 1"},
    {"net": "MOTOR_FR_2", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U4B", "BO2"), ("J3", "2")], "note": "右前马达脚 2"},
    {"net": "MOTOR_RL_1", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U5B", "AO1"), ("J4", "1")], "note": "左后马达脚 1"},
    {"net": "MOTOR_RL_2", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U5B", "AO2"), ("J4", "2")], "note": "左后马达脚 2"},
    {"net": "MOTOR_RR_1", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U5B", "BO2"), ("J5", "1")], "note": "右后马达脚 1（镜像接法：接 BO2）"},
    {"net": "MOTOR_RR_2", "cat": "输出", "current_a": 0.6, "peak_a": 1.5,
     "endpoints": [("U5B", "BO1"), ("J5", "2")], "note": "右后马达脚 2（镜像接法：接 BO1）"},
]

# =============================================================================
# 3. ERC 规则
# =============================================================================

FORBIDDEN_GND_ALIASES = {"VSS", "GNDA", "AGND", "DGND", "GROUND", "0V", "EARTH"}
REQUIRED_POWER_NETS = {"VBAT", "GND", "+5V", "+3V3", "VM"}
POLARIZED_CAPS = {"C1", "C2", "C3", "C6"}
POWER_NETS = REQUIRED_POWER_NETS


def ipc2221_width_mil(current_a: float, copper_oz: float = 1.0,
                      rise_c: float = 10.0, external: bool = True) -> tuple:
    """IPC-2221 载流能力反算：返回 (最小线宽 mil, 所需截面积 mil^2)。"""
    k = 0.048 if external else 0.024
    area = (abs(current_a) / (k * rise_c ** 0.44)) ** (1.0 / 0.725)
    thickness_mil = 1.377 * copper_oz
    return area / thickness_mil, area


WIDTH_CAP_MIL = 100.0   # 单根走线的实用上限；超出部分用铺铜/双面并走/开窗加锡补强


def ceil10(v: float) -> int:
    return int(math.ceil(v / 10.0) * 10)


def width_plan(normal_a: float, peak_a: float, copper_oz: float = 1.0) -> dict:
    """
    走线宽度方案：
      - 常态/峰值分别按 IPC-2221 计算；
      - 采用值取两者较大者，向上取整到 10mil，不低于板级最小线宽，封顶 WIDTH_CAP_MIL；
      - 若峰值需求超过封顶值，必须补强（铺铜 / 双面并走 / 开窗加锡），
        并给出需要的等效截面积，供补强时核算。
    """
    w_normal, _ = ipc2221_width_mil(normal_a, copper_oz)
    w_peak, area_peak = ipc2221_width_mil(peak_a, copper_oz)
    used = max(ceil10(w_normal), ceil10(w_peak), int(BOARD["min_trace_mil"]))
    need_reinforce = w_peak > WIDTH_CAP_MIL
    return {
        "normal_mil": w_normal,
        "peak_mil": w_peak,
        "used_mil": int(min(used, WIDTH_CAP_MIL)),
        "peak_area_mil2": area_peak,
        "reinforce": need_reinforce,
        "reinforce_note": (
            "峰值需 %.0f mil 单线（超出实用上限），改用：铺铜区域 + 双面并走"
            "（每层 %.0f mil）+ 大电流段开窗加锡，等效截面积 ≥ %.0f mil²"
            % (w_peak, WIDTH_CAP_MIL, area_peak)
        ) if need_reinforce else "",
    }


def run_erc(comp_index):
    """电气规则检查。返回 (errors, warnings, stats)。"""
    errors, warnings = [], []
    pin_owner = {}          # (ref, pin) -> net
    net_pins = defaultdict(list)

    # --- 元件级 ---
    refs = [c["ref"] for c in COMPONENTS]
    for ref, n in ((r, c) for r, c in sorted(((r, refs.count(r)) for r in set(refs))) if c > 1):
        errors.append("E001 位号重复：%s 出现 %d 次" % (ref, n))

    for c in COMPONENTS:
        if not c["package"]:
            errors.append("E002 元件 %s 缺少封装（转 PCB 会失败）" % c["ref"])
        pins = set(c["pins"]) | set(c["nc"])
        if len(pins) != len(c["pins"]) + len(c["nc"]):
            errors.append("E003 元件 %s 的 pins 与 nc 存在重名" % c["ref"])
        if not c["pins"]:
            errors.append("E004 元件 %s 没有任何引脚" % c["ref"])
        if c["ref"] in POLARIZED_CAPS and set(c["pins"]) != {"+", "-"}:
            errors.append("E005 极性电容 %s 的引脚必须是 + / -" % c["ref"])

    # --- 网络级 ---
    net_names = [n["net"] for n in NETS]
    for name, cnt in ((x, net_names.count(x)) for x in set(net_names)):
        if cnt > 1:
            errors.append("E006 网络 %s 被定义了 %d 次（一网一名）" % (name, cnt))

    for n in NETS:
        net = n["net"]
        # 命名规则
        if not all(ch.isupper() and (ch.isalnum() or ch in "_+") for ch in net):
            if not (net[0].isalpha() or net[0] == "+"):
                errors.append("E007 网络 %s 命名不合规（只允许 A-Z 0-9 _ +，且字母开头）" % net)
        if len(net) > 16:
            errors.append("E008 网络 %s 超长（>16 字符，丝印放不下）" % net)
        if net.upper() in FORBIDDEN_GND_ALIASES:
            errors.append("E009 出现地网别名 %s（全板只允许 GND 一个名字）" % net)

        if len(n["endpoints"]) < 2:
            errors.append("E010 网络 %s 只有 %d 个端点（单端网络 = 断网）"
                          % (net, len(n["endpoints"])))

        for ref, pin in n["endpoints"]:
            if ref not in comp_index:
                errors.append("E011 网络 %s 引用了不存在的元件 %s" % (net, ref))
                continue
            c = comp_index[ref]
            if pin not in c["pins"] and pin not in c["nc"]:
                errors.append("E012 网络 %s 引用了 %s 上不存在的引脚 %s" % (net, ref, pin))
                continue
            if pin in c["nc"]:
                errors.append("E013 网络 %s 连接了 %s.%s（该脚已声明为不连接 NC）"
                              % (net, ref, pin))
            key = (ref, pin)
            if key in pin_owner:
                errors.append("E014 引脚 %s.%s 同时属于 %s 与 %s（一个脚只能接一个网）"
                              % (ref, pin, pin_owner[key], net))
            else:
                pin_owner[key] = net
            net_pins[net].append(key)

    # --- 悬空引脚 ---
    for c in COMPONENTS:
        for pin in c["pins"]:
            if (c["ref"], pin) not in pin_owner:
                errors.append("E015 元件 %s 的引脚 %s 未接入任何网络（悬空）" % (c["ref"], pin))

    # --- 电源完整性 ---
    present = {n["net"] for n in NETS}
    for req in sorted(REQUIRED_POWER_NETS - present):
        errors.append("E016 缺少必需的电源网络 %s" % req)

    def endpoints_of(net):
        return set(net_pins.get(net, []))

    # VBAT 与 VM 严禁混接
    shared = endpoints_of("VBAT") & endpoints_of("VM")
    if shared:
        for ref, pin in sorted(shared):
            errors.append("E017 %s.%s 同时出现在 VBAT 与 VM（电池原压直接灌进驱动）"
                          % (ref, pin))

    # 每块 DRV8833 的 VM/GND/STBY 必须到位
    for drv, ctrl in (("U4B", "U4A"), ("U5B", "U5A")):
        if pin_owner.get((drv, "VM")) != "VM":
            errors.append("E018 %s.VM 未接到 VM 网络" % drv)
        for gnd_pin in ("GND2", "GND3"):
            if pin_owner.get((drv, gnd_pin)) != "GND":
                errors.append("E019 %s.%s 未接到 GND 网络（大电流回路必须接）" % (drv, gnd_pin))
        if pin_owner.get((ctrl, "GND1")) != "GND":
            errors.append("E019b %s.GND1 未接到 GND 网络" % ctrl)
        if pin_owner.get((ctrl, "STBY")) != "DRV_STBY":
            errors.append("E020 %s.STBY 未接到 DRV_STBY（两块驱动同网）" % ctrl)

    # 每个马达输出网 = 一个驱动输出脚 + 一个端子脚
    for n in NETS:
        if n["cat"] != "输出":
            continue
        refs_in = {r for r, _ in n["endpoints"]}
        if not (len(n["endpoints"]) == 2 and refs_in & {"U4B", "U5B"} and refs_in & {"J2", "J3", "J4", "J5"}):
            errors.append("E021 输出网络 %s 端点不是「驱动输出脚 + 马达端子脚」" % n["net"])

    # 极性电容方向
    for ref in sorted(POLARIZED_CAPS):
        pos_net = pin_owner.get((ref, "+"))
        neg_net = pin_owner.get((ref, "-"))
        if pos_net is not None and pos_net == "GND":
            errors.append("E022 极性电容 %s 正极接到了 GND（装反会爆）" % ref)
        if neg_net is not None and neg_net != "GND":
            errors.append("E023 极性电容 %s 负极未接 GND（当前接 %s）" % (ref, neg_net))

    # 电容耐压（只做提示性校验：数据里写明规格，人工复核）
    for ref in sorted(POLARIZED_CAPS):
        c = comp_index[ref]
        if "16V" not in c["model"] and "10V" not in c["model"]:
            warnings.append("W101 %s 规格未标注耐压，需人工确认 ≥16V" % ref)

    # --- 电流风险 ---
    for budget in POWER_BUDGET:
        if budget["net"] == "VM" and budget["peak_a"] > MINI_MODULE_RATING_A:
            warnings.append(
                "W102 VM 路峰值 %.1fA 超过 mini 模块额定 %.1fA："
                "必须选对策 A(换≥3A 模块) / B(舵机独立供电) / C(软件限流+电容) 之一"
                % (budget["peak_a"], MINI_MODULE_RATING_A))
        if budget["net"] == "VM" and budget["normal_a"] > MINI_MODULE_RATING_A * 0.8:
            warnings.append(
                "W103 VM 路常态 %.1fA 已达模块额定 %.1fA 的 %.0f%%，余量不足"
                % (budget["normal_a"], MINI_MODULE_RATING_A,
                   100.0 * budget["normal_a"] / MINI_MODULE_RATING_A))

    stats = {
        "元件数": len(COMPONENTS),
        "引脚总数": sum(len(c["pins"]) for c in COMPONENTS),
        "网络数": len(NETS),
        "已连接端点": len(pin_owner),
    }
    return errors, warnings, stats


# =============================================================================
# 4. 产物生成
# =============================================================================

def ensure_out_dir():
    os.makedirs(OUT_DIR, exist_ok=True)
    return OUT_DIR


def write_bom(path):
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["位号", "名称", "型号/规格", "PCB 封装", "板上做法", "数量",
                    "嘉立创EDA搜索关键词", "采购必选参数", "备注"])
        for c in COMPONENTS:
            w.writerow([c["ref"], c["name"], c["model"], c["package"], c["mount"],
                        c["qty"], c["search"], c["spec"], c["note"]])


def write_netlist(path, comp_index):
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["网络名", "类别", "端点(元件.引脚)", "端点数", "建议线宽(mil)", "说明"])
        for n in NETS:
            eps = ", ".join("%s.%s" % (r, p) for r, p in n["endpoints"])
            cur = n.get("current_a")
            if cur:
                plan = width_plan(cur, n.get("peak_a", cur), BOARD["copper_oz"])
                width = plan["used_mil"]
                extra = ("；" + plan["reinforce_note"]) if plan["reinforce"] else ""
            else:
                width = int(BOARD["min_trace_mil"])
                extra = ""
            w.writerow([n["net"], n["cat"], eps, len(n["endpoints"]),
                        int(width), (n["note"] + extra).strip("；")])


def write_wiring_md(path, comp_index, pin_owner):
    """按模块组织的接线表：画原理图时照着连，也可打印带去工位。"""
    L = []
    A = L.append
    A("# %s %s · 接线表（引脚级）" % (BOARD["name"], BOARD["rev"]))
    A("")
    A("> 本表由 `crtc_baseboard_v1.py` 自动生成，改动请改源文件后重跑，不要手改本表。")
    A("> 原理图连线与网络命名**全部照抄**本表；同名即同网，拼错一个字就是一条断线。")
    A("")
    A("## 0. 板级参数")
    A("")
    A("| 项 | 值 |")
    A("|---|---|")
    A("| 板框 | %.0f × %.0f mm（圆角 R%.0f） |" % (
        BOARD["outline_mm"][0], BOARD["outline_mm"][1], BOARD["corner_radius_mm"]))
    A("| 层数 / 板厚 / 铜厚 | %d 层 / %.1fmm / %.1f oz |" % (
        BOARD["layers"], BOARD["thickness_mm"], BOARD["copper_oz"]))
    A("| 最小线宽 / 过孔 | %.0f mil / %.0f-%.0f mil |" % (
        BOARD["min_trace_mil"], BOARD["min_via_mil"], BOARD["min_via_drill_mil"]))
    A("| 丝印 | 线宽 %.0f mil，字高 %.0f mil |" % (
        BOARD["silk_line_mil"], BOARD["silk_height_mil"]))
    A("| 安装孔 | M3，孔径 %.1fmm，四角 |" % BOARD["mount_hole_d_mm"])
    A("| 铺铜 | 顶层 + 底层全铺 GND，间距 %.0f mil |" % BOARD["clearance_mil"])
    A("")
    A("## 1. 按模块接线")
    A("")
    for c in COMPONENTS:
        A("### %s · %s" % (c["ref"], c["name"]))
        A("")
        A("- 封装：`%s`　做法：%s" % (c["package"], c["mount"]))
        A("")
        silk = c.get("silk")
        if silk:
            A("| 脚号 | 丝印 | 网络 | 说明 |")
            A("|---|---|---|---|")
            pins_iter = silk
        else:
            A("| 引脚 | 网络 | 说明 |")
            A("|---|---|---|")
            pins_iter = c["pins"]
        for idx, pin in enumerate(pins_iter):
            net = pin_owner.get((c["ref"], pin), "—")
            note = ""
            for n in NETS:
                if n["net"] == net and (c["ref"], pin) in n["endpoints"]:
                    note = n["note"]
                    break
            if net == "—":
                note = "不连接（NC，悬空）"
            if silk:
                A("| %d | %s | `%s` | %s |" % (idx + 1, pin, net, note))
            else:
                A("| %s | `%s` | %s |" % (pin, net, note))
        if c["nc"]:
            A("")
            A("- 不连接（NC）脚：%s —— %s" % (", ".join(c["nc"]), c["note"]))
        A("")
    A("## 2. 按网络反查")
    A("")
    A("| 网络 | 类别 | 端点 | 建议线宽 | 说明 |")
    A("|---|---|---|---|---|")
    for n in NETS:
        eps = "、".join("%s.%s" % (r, p) for r, p in n["endpoints"])
        cur = n.get("current_a")
        if cur:
            plan = width_plan(cur, n.get("peak_a", cur), BOARD["copper_oz"])
            width = plan["used_mil"]
            note = n["note"] + (("；" + plan["reinforce_note"]) if plan["reinforce"] else "")
        else:
            width = int(BOARD["min_trace_mil"])
            note = n["note"]
        A("| `%s` | %s | %s | %d mil | %s |" % (n["net"], n["cat"], eps, int(width), note))
    A("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def write_placement_csv(path):
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["位号", "名称", "X(mm)", "Y(mm)", "旋转(°)", "所在区", "封装"])
        zone = {
            "J1": "电源区", "U1": "电源区", "U2": "电源区", "C6": "电源区",
            "C3": "电源区", "C5": "电源区",
            "U3A": "主控区", "U3B": "主控区", "J8": "主控区",
            "U4A": "驱动区", "U4B": "驱动区", "U5A": "驱动区", "U5B": "驱动区",
            "C1": "驱动区", "C2": "驱动区", "C4": "驱动区",
            "J2": "接口区", "J3": "接口区", "J4": "接口区", "J5": "接口区",
            "J6": "接口区", "J7": "接口区",
        }
        for c in COMPONENTS:
            x, y, rot = c["place"]
            w.writerow([c["ref"], c["name"], "%.1f" % x, "%.1f" % y, rot,
                        zone.get(c["ref"], "未分区"), c["package"]])


def write_power_md(path):
    L = []
    A = L.append
    A("# 电流预算与走线宽度（%s %s）" % (BOARD["name"], BOARD["rev"]))
    A("")
    A("线宽按 IPC-2221 外层、铜厚 %.1f oz、温升 10℃ 计算；采用值取「常态/峰值两者较大」、"
      "向上取整到 10mil、不低于板级最小线宽 %.0f mil，并封顶 %.0f mil（单根走线的实用上限）。"
      % (BOARD["copper_oz"], BOARD["min_trace_mil"], WIDTH_CAP_MIL))
    A("")
    A("| 网络 | 说明 | 常态(A) | 峰值(A) | 常态线宽(mil) | 峰值线宽(mil) | 采用线宽(mil) | 备注 |")
    A("|---|---|---:|---:|---:|---:|---:|---|")
    for b in POWER_BUDGET:
        plan = width_plan(b["normal_a"], b["peak_a"], BOARD["copper_oz"])
        A("| `%s` | %s | %.2f | %.2f | %.1f | %.1f | **%d** | %s |"
          % (b["net"], b["desc"], b["normal_a"], b["peak_a"],
             plan["normal_mil"], plan["peak_mil"], plan["used_mil"], b["note"]))
    A("")
    A("### 需要补强的网络（峰值超过单根走线实用上限）")
    A("")
    for b in POWER_BUDGET:
        plan = width_plan(b["normal_a"], b["peak_a"], BOARD["copper_oz"])
        if plan["reinforce"]:
            A("- `%s`：%s" % (b["net"], plan["reinforce_note"]))
    A("")
    A("## 执行要点")
    A("")
    A("1. `VM` / `VBAT` / `GND` 三根主干按上表**采用线宽**走（%d mil），"
      "双面并行 + 过孔缝合；换层处至少 4 个过孔。"
      % width_plan(2.0, 6.5, BOARD["copper_oz"])["used_mil"])
    A("2. 上表中需要补强的网络，按「铺铜区域 + 双面并走 + 大电流段开窗加锡」处理，"
      "不要试图用一根 %.0f mil 以上的单线硬扛。" % WIDTH_CAP_MIL)
    A("3. 马达输出走线不低于 %d mil（单路峰值 1.5A）。"
      % width_plan(0.6, 1.5, BOARD["copper_oz"])["used_mil"])
    A("4. 信号线（8 路 PWM、PS2、舵机、STBY）按板级最小线宽 %d mil 起步，"
      "同组信号平行等距、不与功率线混行。" % int(BOARD["min_trace_mil"]))
    A("5. 拐角一律 45°，禁止直角；焊盘与走线连接处加泪滴；"
      "顶底层均铺 GND 铜，间距 %.0f mil，孤岛必须处理。" % BOARD["clearance_mil"])
    A("")
    A("## 风险 R3：VM 路供电能力")
    A("")
    A("mini 降压模块额定按 %.1fA 计；VM 路常态 %.1fA、峰值 %.1fA。"
      % (MINI_MODULE_RATING_A, 2.0, 5.5))
    A("")
    A("- **对策 A（推荐）**：mini#2 换成持续输出 ≥3A 的降压模块（成本增加约 5～10 元）；")
    A("- **对策 B**：舵机改由第三块 mini（或核心板 5V 经独立 5V 稳压）供电；")
    A("- **对策 C**：软件限流（启动缓升、堵转检测降占空比）+ C1/C2 储能电容（已设计进本版）；")
    A("- 三策可叠加。（历史记录：V1 阶段要求“在选型确认前不要下单”整板；**V2 已于 10-17 完成选型取舍并下单——订单 Y1**，见《工程日志》。）")
    A("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def write_assembly_md(path, pin_owner):
    """板子回来后、通电前的逐项核对清单。"""
    L = []
    A = L.append
    A("# 装配核对表（%s %s）" % (BOARD["name"], BOARD["rev"]))
    A("")
    A("> 板子回来后**先做完本表再通电**。全部用万用表蜂鸣档（断电测量）。")
    A("")
    A("## A. 焊接后、插模块前（断电，蜂鸣档）")
    A("")
    A("| # | 核对项 | 预期 | 结果 |")
    A("|---|---|---|---|")
    checks = [
        ("J1.BAT+ ↔ U1.IN+ / U2.IN+ / C6.+", "全部响（同一 VBAT 网）"),
        ("J1.BAT- ↔ U1.IN- / U2.IN- / U3A.G / U3B.G1 / U3B.G2 / U4A.GND1 / U4B.GND2 / U4B.GND3 / U5A.GND1 / U5B.GND2 / U5B.GND3 / J7.GND", "全部响（GND 网）"),
        ("J1.BAT+ ↔ U4B.VM / U5B.VM / J6.V", "**不响**（VBAT 与 VM 必须完全隔离）"),
        ("U1.OUT+ ↔ U3A.5V / C3.+ / J8.+5V", "响（+5V 网）"),
        ("U2.OUT+ ↔ U4B.VM / U5B.VM / J6.V / C1.+ / C2.+ / C4", "响（VM 网）"),
        ("U1.OUT+ ↔ U2.OUT+", "**不响**（+5V 与 VM 不得短接）"),
        ("U3A.B12 ↔ U4A.STBY ↔ U5A.STBY", "三点互相响（DRV_STBY）"),
        ("U4B.AO1 ↔ J2.1，U4B.AO2 ↔ J2.2", "分别响，且两组之间不响"),
        ("U4B.BO1 ↔ J3.1，U4B.BO2 ↔ J3.2", "分别响"),
        ("U5B.AO1 ↔ J4.1，U5B.AO2 ↔ J4.2", "分别响"),
        ("U5B.BO2 ↔ J5.1，U5B.BO1 ↔ J5.2", "分别响（RR 为镜像接法）"),
        ("U3B.A6/A7 ↔ U4A.AIN1/BIN1；U3A.A8/A9 ↔ U4A.AIN2/BIN2", "一一对应响"),
        ("U3B.B0/B1 ↔ U5A.AIN1/BIN1；U3A.A10/A11 ↔ U5A.AIN2/BIN2", "一一对应响"),
        ("U3A.B3/B4/B5/B6 ↔ J7.CS/CLK/CMD/DAT", "一一对应响"),
        ("U3A.B7 ↔ J6.S；U3B.A2 ↔ J8.TX；U3B.A3 ↔ J8.RX", "分别响"),
        ("U4A/U5A 的两个 NC 脚 ↔ 任何网络", "**不响**（模块第 1/7 位 NC 脚悬空）"),
        ("U4B/U5B 的 VM 脚 ↔ C1/C2 的 + 脚", "响（VM 网，含储能电容）"),
        ("C1/C2/C6 的 + 脚 ↔ GND", "**不响**（极性电容方向核对）"),
    ]
    for i, (item, expect) in enumerate(checks, 1):
        A("| A%02d | %s | %s | ☐ |" % (i, item, expect))
    A("")
    A("## B. 模块插入方向（插错必烧）")
    A("")
    A("| 模块 | 核对方法 | 结果 |")
    A("|---|---|---|")
    A("| U1/U2 mini 降压 | 模块丝印 IN+ 对准底板 IN+，OUT+ 对准 OUT+；反插会反灌烧核心板 | ☐ |")
    A("| U4/U5 DRV8833（2×8，16 脚） | 左列 NC/AIN2/AIN1/STBY/BIN1/BIN2/NC/GND 对准 U4A/U5A；右列 VM/NC/GND/AO1/AO2/BO2/BO1/GND 对准 U4B/U5B（BO2 在上）；插反插错位必烧 | ☐ |")
    A("| U3 STM32 核心板 | USB 座朝板外；5V 脚对准底板 5V（插反必烧核心板） | ☐ |")
    A("| J6 舵机 | 黄=信号(S)、红=V、棕=G；插反舵机立即烧毁 | ☐ |")
    A("| J7 PS2 | VCC 接 +3V3；确认不是接到 +5V | ☐ |")
    A("")
    A("## C. 首次上电（分级，冒烟即断电）")
    A("")
    A("| 步骤 | 操作 | 预期 | 结果 |")
    A("|---|---|---|---|")
    A("| C1 | 不插任何模块，只接电池测 J1 两端 | 7.0～8.4V | ☐ |")
    A("| C2 | 插 U1，测 U1.OUT+ | **5.00V ±0.05**（超过 5.25V 立刻断电，会烧核心板） | ☐ |")
    A("| C3 | 插 U2，测 U2.OUT+ | **5.00V ±0.10** | ☐ |")
    A("| C4 | 断电，插 U3 核心板，再上电 | 核心板电源灯亮，无冒烟/发热 | ☐ |")
    A("| C5 | 插 U4/U5（先不插 STBY 之外的负载） | 无发热 | ☐ |")
    A("| C6 | 烧录固件，给 DRV_STBY 高电平，单轮点动 | 对应轮转动，方向不对就整组对调端子线 | ☐ |")
    A("| C7 | 四轮齐动 + 舵机 | 无复位、无明显压降（VM 跌落 ≤0.5V） | ☐ |")
    A("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def write_wiring_html(path, pin_owner):
    """可打印的施工单：接线表 + 装配核对表，A4 打印带去工位。"""
    rows = []
    for c in COMPONENTS:
        lines = []
        for pin in c["pins"]:
            net = pin_owner.get((c["ref"], pin), "—")
            lines.append("<tr><td class='p'>%s</td><td class='n'>%s</td></tr>" % (pin, net))
        if c["nc"]:
            lines.append("<tr><td class='p'>%s</td><td class='nc'>NC（不连接）</td></tr>"
                         % " / ".join(c["nc"]))
        rows.append(
            "<div class='mod'><div class='mh'><span class='ref'>%s</span>"
            "<span class='nm'>%s</span><span class='pk'>%s</span></div>"
            "<table>%s</table></div>"
            % (c["ref"], c["name"], c["package"],
               "<tr><th>引脚</th><th>网络</th></tr>" + "".join(lines)))

    checks = []
    items = [
        ("J1.BAT+ 与 U4B.VM / U5B.VM / J6.V", "必须不导通（VBAT 与 VM 隔离）"),
        ("J1.BAT- 与 U3A.G / U3B.G1 / U3B.G2 / U4A.GND1 / U4B.GND2 / U5A.GND1 / U5B.GND2 / J7.GND", "全部导通（GND）"),
        ("U1.OUT+ 与 U2.OUT+", "必须不导通（+5V 与 VM 隔离）"),
        ("U3A.B12 与 U4A.STBY 与 U5A.STBY", "三点互导（DRV_STBY）"),
        ("U4A/U5A 的 NC 脚（第 1、7 位）与任何网络", "必须不导通（模块 NC 脚悬空）"),
        ("C1/C2/C6 的正极与 GND", "必须不导通（极性方向）"),
        ("U5B.BO2 与 J5.1、U5B.BO1 与 J5.2", "分别导通（RR 镜像接法）"),
        ("U1.OUT+ 实测电压", "5.00V ±0.05（超过 5.25V 立即断电）"),
        ("U2.OUT+ 实测电压", "5.00V ±0.10"),
    ]
    for i, (item, expect) in enumerate(items, 1):
        checks.append("<li><span class='cb'></span><b>%02d</b> %s —— <i>%s</i></li>" % (i, item, expect))

    html = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>%(title)s 施工单</title>
<style>
:root{--ink:#111;--dim:#666;--line:#ccc;--acc:#0a5;--bad:#c00;--bg:#fff}
*{box-sizing:border-box}
body{margin:0;padding:16px;font:13px/1.5 -apple-system,"Microsoft YaHei",sans-serif;color:var(--ink);background:var(--bg)}
h1{font-size:20px;margin:0 0 4px}
h2{font-size:15px;margin:20px 0 8px;padding-bottom:4px;border-bottom:2px solid var(--ink)}
.meta{color:var(--dim);font-size:12px;margin-bottom:8px}
.grid{display:flex;flex-wrap:wrap;gap:10px}
.mod{border:1px solid var(--line);border-radius:6px;padding:8px 10px;min-width:230px;flex:1 1 230px;break-inside:avoid}
.mh{display:flex;align-items:baseline;gap:8px;margin-bottom:6px;flex-wrap:wrap}
.ref{font-weight:700;background:#111;color:#fff;border-radius:4px;padding:1px 6px;font-size:12px}
.nm{font-weight:600;font-size:13px}
.pk{color:var(--dim);font-size:11px;font-family:Consolas,monospace}
table{border-collapse:collapse;width:100%%}
th{font-size:11px;color:var(--dim);text-align:left;border-bottom:1px solid var(--line);padding:2px 4px}
td{padding:2px 4px;border-bottom:1px dotted #eee;font-size:12px}
td.p{width:38%%;font-family:Consolas,monospace}
td.n{font-weight:600;color:var(--acc)}
td.nc{color:var(--bad);font-size:11px}
ol{padding-left:0;list-style:none}
li{margin:5px 0;display:flex;gap:8px;align-items:flex-start}
.cb{width:13px;height:13px;border:1.5px solid #333;border-radius:3px;flex:0 0 auto;margin-top:2px}
i{color:var(--dim);font-style:normal}
@media print{body{padding:0}.mod{min-width:210px}}
</style></head><body>
<h1>%(title)s %(rev)s · 施工单</h1>
<div class="meta">板框 %(w).0f×%(h).0fmm · %(layers)d层/1oz · 最小线宽 %(mt)d mil · 过孔 %(v)d/%(vd)d mil · 顶层底层全铺 GND ｜ 由 crtc_baseboard_v1.py 自动生成</div>
<h2>接线表（同名即同网，照抄）</h2>
<div class="grid">%(mods)s</div>
<h2>装配核对（断电·蜂鸣档，通电前必做）</h2>
<ol>%(checks)s</ol>
</body></html>""" % {
        "title": BOARD["name"], "rev": BOARD["rev"],
        "w": BOARD["outline_mm"][0], "h": BOARD["outline_mm"][1],
        "layers": BOARD["layers"], "mt": int(BOARD["min_trace_mil"]),
        "v": int(BOARD["min_via_mil"]), "vd": int(BOARD["min_via_drill_mil"]),
        "mods": "".join(rows), "checks": "".join(checks),
    }
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)


def write_erc_report(path, errors, warnings, stats):
    L = []
    A = L.append
    A("ERC 报告 · %s %s" % (BOARD["name"], BOARD["rev"]))
    A("=" * 60)
    A("")
    A("统计：元件 %d 个 / 引脚 %d 个 / 网络 %d 条 / 已连接端点 %d 个"
      % (stats["元件数"], stats["引脚总数"], stats["网络数"], stats["已连接端点"]))
    A("")
    A("错误 %d 条，警告 %d 条" % (len(errors), len(warnings)))
    A("")
    if errors:
        A("--- 错误（必须修复） ---")
        for e in sorted(errors):
            A("  " + e)
        A("")
    if warnings:
        A("--- 警告（需确认或落实对策） ---")
        for w in sorted(warnings):
            A("  " + w)
        A("")
    if not errors:
        A("结论：ERC 通过，可进入布局布线。")
    else:
        A("结论：存在阻断性错误，禁止进入布局。")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


# =============================================================================
# 5. 主流程
# =============================================================================

def main(argv):
    quiet = "--quiet" in argv
    comp_index = {c["ref"]: c for c in COMPONENTS}
    errors, warnings, stats = run_erc(comp_index)

    pin_owner = {}
    for n in NETS:
        for ep in n["endpoints"]:
            pin_owner[ep] = n["net"]

    out = ensure_out_dir()
    write_bom(os.path.join(out, "BOM.csv"))
    write_netlist(os.path.join(out, "NETLIST.csv"), comp_index)
    write_wiring_md(os.path.join(out, "接线表.md"), comp_index, pin_owner)
    write_placement_csv(os.path.join(out, "布局坐标.csv"))
    write_power_md(os.path.join(out, "线宽与电流预算.md"))
    write_assembly_md(os.path.join(out, "装配核对表.md"), pin_owner)
    write_wiring_html(os.path.join(out, "施工单.html"), pin_owner)
    write_erc_report(os.path.join(out, "ERC报告.txt"), errors, warnings, stats)

    if not quiet:
        print("=" * 64)
        print(" %s %s —— 设计数据生成完成" % (BOARD["name"], BOARD["rev"]))
        print("=" * 64)
        print(" 板框：%.0f × %.0f mm | 元件 %d | 引脚 %d | 网络 %d"
              % (BOARD["outline_mm"][0], BOARD["outline_mm"][1],
                 stats["元件数"], stats["引脚总数"], stats["网络数"]))
        print(" 产物目录：%s" % out)
        for fn in ("BOM.csv", "NETLIST.csv", "接线表.md", "布局坐标.csv",
                   "线宽与电流预算.md", "装配核对表.md", "施工单.html", "ERC报告.txt"):
            print("   - %s" % fn)
        print("-" * 64)
        print(" ERC：错误 %d 条 / 警告 %d 条" % (len(errors), len(warnings)))
        for e in sorted(errors):
            print("   [E] " + e)
        for w in sorted(warnings):
            print("   [W] " + w)
        if not errors:
            print(" 结论：ERC 通过，可进入布局布线。")
        else:
            print(" 结论：存在阻断性错误，修复后再继续。")
        print("-" * 64)

    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
