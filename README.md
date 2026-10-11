# CRTC_Smart_Car

重庆大学机器人训练大赛 CRTC2026 参赛小车（破晓战队）协作仓库。

- 赛事：CRTC2026（重庆大学机器人训练大赛）
- 机器人：麦轮底盘 + 前端收集机构，PS2 手柄遥控
- 主控：STM32F103C8T6
- 建队：2026-09-19 ｜ 仓库建立：2026-10-10
- 中期考核材料提交截止：**2026-10-19 24:00**

## 文档索引

| 文档                                                                   | 内容                                       |
| -------------------------------------------------------------------- | ---------------------------------------- |
| [docs/competition-requirements.md](docs/competition-requirements.md) | 赛事规则提取。每条标注手册页码                          |
| [docs/hardware-inventory.md](docs/hardware-inventory.md)             | 硬件清单、标签确认结果、待确认参数、兼容性初判、补购清单             |
| [docs/structure-analysis.md](docs/structure-analysis.md)             | 3D 打印件与铲子机构分析、规则符合性检查、设计检查清单             |
| [docs/control-architecture.md](docs/control-architecture.md)         | 遥控链路三阶段（有线 → 蓝牙 → PS2）、指令协议、失联保护、软硬件协作清单 |
| [docs/development-log.md](docs/development-log.md)                   | 开发日志（做了什么 + 为什么这样做）                      |

## 规划中的文档

`docs/` 下按需增补，不建空文件：

- `power-design.md` — 供电方案（等电机额定电压与电池串并联确认后）
- `pin-mapping.md` — 引脚分配与接线表（等 STM32 板实物确认后）
- `wiring-guide.md` — 逐根接线指引
- `hardware-test-log.md` — 分模块测试记录
- `debugging-log.md` — 故障排查记录
- `mecanum-kinematics.md` — 麦轮运动学与速度分配（阶段五）

## 目录

```
docs/                     项目文档（手册 P10 要求：全部文档放在 docs/ 下）
```

> 立创 EDA 工程（原理图 / PCB）位于仓库同级目录 `../PCB/`，未纳入本仓库。

## 硬件现状

见 [docs/hardware-inventory.md](docs/hardware-inventory.md)。当前尚未确认的核心件：TT 减速马达、PS2 手柄及接收器、ST-Link。
