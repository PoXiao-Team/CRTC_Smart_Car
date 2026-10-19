/**
 * @file robot.h
 * @brief 对外接口、可调参数、翻铲状态编号、调试状态变量。
 * 展示重点：PWM上限、右摇杆死区、舵机脉宽和状态机定义。
 */
#ifndef CRTC_ROBOT_H
#define CRTC_ROBOT_H
#include "main.h"

/* 在 CubeMX 初始化 GPIO、TIM1/2/3/4 后调用一次；主循环持续调用 Task。 */
void Robot_Init(void); /* 初始化，只调用一次。 */
void Robot_Task(void); /* 主循环反复调用，内部20ms轮询。 */
void Robot_Stop(void); /* 立即清零电机PWM，撤销舵机脉冲、清除示教。 */

/* 翻铲版：上电不输出脉冲，START 使能后才输出。首次务必卸下舵盘测试。
 * 自动动作必须先用方框和圆圈教会两个位置；断电/停机后需重新示教。
 */
#define ROBOT_DRIVE_LIMIT 500  /* 千分比：首次调试最多输出 50% PWM */
#define ROBOT_FAST_LIMIT 750  /* 按住L1加速：75%PWM；松开恢复普通50% */
#define ROBOT_DEADZONE 18     /* 右摇杆中心±18不参与转向 */

/* 当前仅一个翻铲位置舵机，PB7/TIM4 CH2。
 * 2026 手册的三舵机考核尚需两个具体机构；PB8/PB9 预留，输出始终为 0。
 * MIN/MAX 只是初始试验窗口，不是已验证的机械安全限位。
 */
/* 已确认L2使铲子下降：本版将下降端从800us扩大到600us。
 * 这只是软件脉宽范围，不保证机构可用，不表示精确增加60度。
 * 已装铲子时只能短按逐步接近目标；到达机械限位前必须停止。
 */
#define SHOVEL_MIN_US 600   /* L2下降端；仅软件限幅，不能当作机械可用端点 */
#define SHOVEL_INITIAL_US 1500 /* 保留原初始脉宽；落地位置用L2/R2点动并以方框记录 */
#define SHOVEL_MAX_US 2000  /* 上升端保持2000us，不扩大；遇阻不能继续 */
#define SHOVEL_STEP_US 10      /* 每20ms改变10us；点动和自动移动的指令变化速率为上一版2倍 */
#define SHOVEL_SETTLE_MS 500
#define SHOVEL_DUMP_HOLD_MS 800

enum { SHOVEL_IDLE, SHOVEL_TO_DOWN, SHOVEL_WAIT_DOWN,
       SHOVEL_TO_DUMP, SHOVEL_WAIT_DUMP, SHOVEL_RETURN, SHOVEL_WAIT_RETURN };
/* CubeIDE 调试器的 Expressions 可添加 g_robot_status 查看实时状态。
 * 暂停调试会导致 IWDG 复位；查看变量时必须断开动力。
 */
typedef struct {
    uint16_t servo_us, down_us, dump_us, buttons; /* 当前/铲下/翻倒脉宽和按键位图 */
    uint8_t connected, armed, ps2_mode, shovel_state, taught_mask; /* 帧有效/运行许可/模式/动作状态/示教标志 */
    uint8_t lx, ly, rx, ry; /* 原始摇杆值，左摇杆仅供观察 */
    uint32_t bad_frames; /* 无效通信帧累计次数 */
} RobotStatus;
extern volatile RobotStatus g_robot_status;
#endif
