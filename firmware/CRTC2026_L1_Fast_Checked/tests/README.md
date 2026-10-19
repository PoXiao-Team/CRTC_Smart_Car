本测试需 Python、unicorn、pyelftools：`python -m pip install unicorn pyelftools`。
运行：`python <项目路径>/tests/test_firmware.py`。

测试读取实际 build/CRTC2026_F103C8_DRV8833.elf，执行 ARM Thumb 指令。
模拟通信字节、配置命令、延时和毫秒时间，模拟 RAM 和定时器寄存器。
没有测试真实无线链路、电气时序、电机、舵机、看门狗硬件或铲子结构。
89 项通过结果见 test_report.json；不能替代实车验收。

速度、死区、脉宽范围等配置变化后，应按新需求修改相应断言。

本包build中的ELF是CubeIDE实际编译生成的Debug ELF副本。重新在IDE构建后，请将Debug/CRTC2026_L1_Fast_Checked.elf复制到build/CRTC2026_F103C8_DRV8833.elf，再运行测试。

