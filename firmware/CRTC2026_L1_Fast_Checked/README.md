# firmware — 1 号车嵌入式固件（STM32F103C8T6）

> 工程 **CRTC2026_L1_Fast_Checked**：PS2 手柄 → 四麦轮移动 + 单舵机翻铲示教。
> SPI 软件实现、HAL 库、无 RTOS、无外部晶振（HSI PLL 64 MHz）。CubeIDE 1.18.1 编译 **0 错 0 警**；89 项 ELF 指令级仿真检查 **全部 PASS**。

## 目录

| 路径 | 内容 |
| --- | --- |
| `Core/Src/main.c` | 初始化与主循环：时钟 / GPIO / 定时器 / 看门狗 / 状态灯 / 故障处理 |
| `Core/Src/robot.c` | 业务控制：PS2 解析、四轮混控、电机输出、翻铲状态机 |
| `Core/Inc/robot.h` | 接口、可调参数、翻铲状态编号、调试状态结构 |
| `Core/Inc/main.h · tim.h · iwdg.h` | 句柄与调试变量声明 |
| `Core/Startup/` | 启动汇编（`startup_stm32f103xb.s`） |
| `tests/` | 固件回归测试（`test_firmware.py` + `test_report.json`，89 项） |
| `build.ps1` | 命令行构建脚本（arm-none-eabi-gcc） |
| `STM32F103C8_FLASH.ld` | 链接脚本 |
| `BUILD_INFO.txt` | 构建/验证信息与 SHA256 |
| `先看这里_L1快速与代码检查.md` | 本版变更说明、代码检查记录、**上车前安全检查（必读）** |

**未入库**（可由 CubeMX / CubeIDE 重新生成）：`Drivers/`（HAL + CMSIS，5.7 MB）、`Debug/`、`build/`（编译产物）。

## 编译与烧录

1. CubeIDE：`File → Import → General → Existing Projects into Workspace` 选择本目录（Drivers 需由 CubeMX 按同一配置重新生成，或从 STM32CubeF1 包补齐）；
2. 或命令行：`powershell -File build.ps1 -Compiler <arm-none-eabi-gcc 路径>`；
3. 烧录 `Debug/CRTC2026_L1_Fast_Checked.elf`（构建后生成）。

## 关键口径（详见《遥控与指令篇》"嵌入式固件实现"板块）

- 普通 PWM 50%，**按住 L1 加速到 75%**（松开回 50%）；
- **START 使能**（LED 常亮）；R1 保持运行；SELECT 软件停机 + 清示教；
- 舵机脉宽 **600~2000 µs**（初始 1500 µs，步进 10 µs/20 ms）——也是"角度达不到 180°"的直接原因之一；
- 电机通道与实车对应：PA6/PA8=左后、PA7/PA9=右后、PB0/PA10=左前、PB1/PA11=右前（与设计命名相反，固件已补偿）。

## ⚠️ 上车前安全检查（摘自本版说明，继续有效）

此前出现过**供电迅速发热、黄灯灭后全车无响应、必须拔电重启**；根因**未确认**。故障未排除前：
- 不进行快速落地行驶；
- 先断电检查焊点、绝缘与实际供电分配，再在动力断开条件下烧录；
- 确认控制板供电稳定后先架空测试普通模式，最后短时间试 L1；
- 出现熄灯、抖动、卡住、持续嗡鸣或发热立即断电；
- 软件 SELECT **不能**代替物理断电。
