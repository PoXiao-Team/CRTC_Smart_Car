/**
 * @file robot.c
 * @brief CRTC期中展示：PS2十字键四麦轮移动、右摇杆旋转、单舵机翻铲。
 *
 * 阅读顺序：13控制调度 -> 6数据解析 -> 10四轮混控 -> 7电机输出 -> 11翻铲。
 * 调用关系：main -> Robot_Init(一次) -> Robot_Task(循环) -> 各功能函数。
 * 左摇杆仅记录原始值；不控制车辆。所有底盘运动要求按住R1。
 *
 * 【当前实车接法，已按用户说明确认】
 * 数组序号0：原前轮板A通道，PA6/PA8，实际接左后轮。
 * 数组序号1：原前轮板B通道，PA7/PA9，实际接右后轮。
 * 数组序号2：原后轮板A通道，PB0/PA10，实际接左前轮。
 * 数组序号3：原后轮板B通道，PB1/PA11，实际接右前轮。
 * 保留既有通道顺序及drive中的x反号，保持十字键版的现有运动指令。
 * 本版保留此前方向修正；方向键按住时禁止右摇杆叠加旋转。
 * 右摇杆仅在方向键全部松开时控制旋转。此改动排除混控输入干扰，
 * 不保证修复电机单向不转、接触不良或供电异常。
 * 已确认L2下降，舵机下降端扩大到600us，上升端保持2000us；初始1500us不作未经标定的方向猜测。
 * 不声称修复左后单向不转或之前报告的电气发热。
 * 无编码器，当前为开环PWM控制；相同占空比不能保证实际轮速相同。
 */

#include "robot.h"
#include "tim.h"
#include <stdint.h>

/* ==================== 1. 电机通道表和运行状态 ====================
 * 每个电机用两根PWM输入控制正反转，TIM3接IN1，TIM1接IN2。
 * TIM3 CH1~4默认映射PA6/PA7/PB0/PB1；TIM1 CH1~4映射PA8~PA11。
 * motors是硬件通道顺序，不能再按原接线文档的FL/FR/RL/RR称呼实物。
 * polarity=1保持原极性；不要用修改极性掩盖某个方向不转的电气故障。
 */
typedef struct {
    uint32_t channel;
    int8_t polarity;
} Motor;
static const Motor motors[4] = {
    {TIM_CHANNEL_1, 1}, /* PA6 / PA8  -> 实际左后 */
    {TIM_CHANNEL_2, 1}, /* PA7 / PA9  -> 实际右后 */
    {TIM_CHANNEL_3, 1}, /* PB0 / PA10 -> 实际左前 */
    {TIM_CHANNEL_4, 1}  /* PB1 / PA11 -> 实际右前 */
};
/* applied记录实际下发的带符号PWM；reversal_wait记录换向停顿周期。 */
static int16_t applied[4];
static uint8_t reversal_wait[4];
/* armed=是否允许操作；healthy_frames=连续有效通信帧计数，最多累计到10。 */
static uint8_t armed, healthy_frames;
/* 翻铲期间停轮；翻铲结束后要求先松开R1再重新按下。 */
static uint8_t drive_rearm_needed;
/* 与上一帧比较，用来识别“刚刚按下”，避免按住方框时重复记录位置。 */
static uint16_t previous_buttons;
static uint32_t last_poll, last_config;
volatile RobotStatus g_robot_status = {.servo_us = SHOVEL_INITIAL_US};
static uint32_t shovel_since;

/* 2. PS2 按键：协议中 0 表示按下；下面解析后转为 1 表示按下。 */
enum {
    SELECT = 1u << 0, START = 1u << 3,
    UP = 1u << 4, RIGHT = 1u << 5, DOWN = 1u << 6, LEFT = 1u << 7,
    L2 = 1u << 8, R2 = 1u << 9, L1 = 1u << 10, R1 = 1u << 11,
    TRIANGLE = 1u << 12, CIRCLE = 1u << 13,
    CROSS = 1u << 14, SQUARE = 1u << 15
};
typedef struct {
    uint16_t buttons;
    uint8_t lx, ly, rx, ry;
} Pad;

/* 3. TIM2 以 1 MHz 计数提供微秒时序，避免用未经标定的软件空循环。
 * 本模块必须在主循环使用，不能放到中断中。TIM2 周期必须为 65535。
 */
static void delay_us(uint16_t us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    /* 正常计时仍由TIM2提供。guard不是计时替代，只防定时器停止后无限等待。
     * 不在这里喂狗；若计数器不动，记录故障并进入统一停机处理。
     */
    uint32_t guard = 100000u;
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - start) < us) {
        if (--guard == 0u) {
            g_fault_tag = 2u; /* TIM2微秒等待超出循环预算 */
            Error_Handler();
        }
    }
}
static void ps2_clk(GPIO_PinState s) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, s); }
static void ps2_cmd(GPIO_PinState s) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, s); }
static void ps2_att(GPIO_PinState s) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, s); }

/* 4. 软件同步串行：最低位先发送、CLK 空闲高、低期间输出 CMD。
 * 与卖家 F1 例程一致：低半周期末读 DAT，然后拉高 CLK。半周期 16us。
 * DAT 接 PB6。这里只采用协议事实；代码是自己的 HAL 实现。
 */
static uint8_t ps2_byte(uint8_t tx)
{
    uint8_t rx = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        ps2_cmd((tx & (1u << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ps2_clk(GPIO_PIN_RESET);
        delay_us(16);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
            rx |= (uint8_t)(1u << bit);
        ps2_clk(GPIO_PIN_SET);
        delay_us(16);
    }
    ps2_cmd(GPIO_PIN_SET);
    delay_us(20);  /* 接收器字节间处理时间；本参考实现不读取 ACK */
    return rx;
}
/* 一包数据：CS拉低选中接收器 -> 逐字节交换 -> CS拉高结束。
 * rx为空指针时只发送命令，不保存返回数据。 */
static void ps2_packet(const uint8_t *tx, uint8_t *rx, uint8_t count)
{
    ps2_clk(GPIO_PIN_SET);
    ps2_att(GPIO_PIN_RESET);
    delay_us(20);
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t v = ps2_byte(tx[i]);
        if (rx) rx[i] = v;
    }
    ps2_att(GPIO_PIN_SET);
    delay_us(30);
}

/* 5. 请求模拟摇杆模式。兼容常见 PS2 接收器，但需要实测具体型号。
 * 先轮询唤醒，再进入配置、请求模拟模式、退出；不锁死 MODE 切换。
 * 卖家 F1 示例只轮询，接收器可能不支持配置命令，仍需按 MODE 到 0x73。
 * 失败后保持停机并重试。
 */
static void ps2_configure(void)
{
    const uint8_t poll[]  = {1,0x42,0,0,0,0,0,0,0};
    const uint8_t enter[] = {1,0x43,0,1,0,0,0,0,0};
    const uint8_t mode[]  = {1,0x44,0,1,0,0,0,0,0};
    const uint8_t leave[] = {1,0x43,0,0,0x5A,0x5A,0x5A,0x5A,0x5A};
    ps2_packet(poll, 0, sizeof poll); HAL_Delay(10);
    ps2_packet(enter, 0, sizeof enter); HAL_Delay(10);
    ps2_packet(mode, 0, sizeof mode); HAL_Delay(10);
    ps2_packet(leave, 0, sizeof leave); HAL_Delay(10);
}

/* 6. 只接受模拟模式 0x73/0x79 和正确 0x5A 标记。
 * 数字模式 0x41 不用于驱动车辆，避免将不存在的摇杆字节解释成满速。
 * 0x79 的完整压力数据帧为 21 字节，读完但只使用按键与四个摇杆。
 */
static uint8_t ps2_read(Pad *p)
{
    uint8_t rx[21];
    ps2_clk(GPIO_PIN_SET);
    ps2_att(GPIO_PIN_RESET);
    delay_us(20);
    rx[0] = ps2_byte(1);
    rx[1] = ps2_byte(0x42);
    rx[2] = ps2_byte(0);
    g_robot_status.ps2_mode = rx[1];
    uint8_t count = (rx[1] == 0x79) ? 21 : ((rx[1] == 0x73 || rx[1] == 0x53) ? 9 : 5);
    for (uint8_t i = 3; i < count; ++i) rx[i] = ps2_byte(0);
    ps2_att(GPIO_PIN_SET);
    delay_us(30);
    if (rx[0] != 0xFF || rx[2] != 0x5A ||
        (rx[1] != 0x73 && rx[1] != 0x79)) return 0;
    /* PS2按键原始位为0表示按下，按位取反后统一成1表示按下。 */
    p->buttons = (uint16_t)~((uint16_t)rx[3] | ((uint16_t)rx[4] << 8));
    /* 第5/6字节为右摇杆XY，第7/8字节为左摇杆XY。中心通常约128。 */
    p->rx = rx[5]; p->ry = rx[6]; p->lx = rx[7]; p->ly = rx[8];
    return 1;
}

/* 7. DRV8833 双输入 PWM，参数 -1000..1000。
 * 正向 IN1=PWM,IN2=0；反向 IN1=0,IN2=PWM；停止两输入均为 0（滑行）。
 * 两方向都采用快衰减，避免一边快衰减一边慢衰减带来不对称。
 * Robot_Task 的斜坡及反转停顿避免直接正反切换。
 */
static void motor_write(uint8_t index, int16_t value)
{
    const Motor *m = &motors[index];
    int32_t v = (int32_t)value * m->polarity;
    /* 先清空同一电机两方向的占空比，再只写入所需方向。 */
    __HAL_TIM_SET_COMPARE(&htim3, m->channel, 0);
    __HAL_TIM_SET_COMPARE(&htim1, m->channel, 0);
    uint8_t forward = v > 0;
    if (v < 0) v = -v;
    if (v > 1000) v = 1000;
    /* 输入千分比乘以计数周期，转换成CCR比较值；ARR=999时数值相等。 */
    uint32_t duty = (uint32_t)v * (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1u) / 1000u;
    if (forward) __HAL_TIM_SET_COMPARE(&htim3, m->channel, duty);
    else __HAL_TIM_SET_COMPARE(&htim1, m->channel, duty);
}

/* 8. 软件停机：底盘立即停止，舵机撤销控制脉冲，不执行自动回零。
 * 这不会物理切断电源；舵机停止脉冲后也不能保证承重机构保持位置。
 */
void Robot_Stop(void)
{
    armed = 0;
    /* 若模块已引出 nSLEEP，接 PB12；未引出时该引脚可不接。
     * 不要连接模块上被硬接电源的端子，需先核对实际红板。
     */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    healthy_frames = 0;
    drive_rearm_needed = 0;
    g_robot_status.armed = 0;
    g_robot_status.shovel_state = SHOVEL_IDLE;
    g_robot_status.taught_mask = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        applied[i] = 0; reversal_wait[i] = 0; motor_write(i, 0);
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

/* ==================== 9. 右摇杆处理和渐变函数 ====================
 * axis：中心128，中心附近±18忽略，避免松手后轻微偏转导致车辆旋转。
 * 输出范围-1000~1000；此版仅右摇杆X使用本函数。
 */
static int16_t axis(uint8_t raw)
{
    int32_t v = (int32_t)raw - 128;
    int32_t a = (v < 0) ? -v : v;
    if (a <= ROBOT_DEADZONE) return 0;
    int32_t span = (v < 0 ? 128 : 127) - ROBOT_DEADZONE;
    int32_t result = (a - ROBOT_DEADZONE) * 1000 / span;
    return (int16_t)(v < 0 ? -result : result);
}
/* 每次最多变化step，避免电机/舵机命令突然跳变。此函数不等待、不阻塞。 */
static int16_t approach(int16_t from, int16_t to, int16_t step)
{
    if (to > from + step) return from + step;
    if (to < from - step) return from - step;
    return to;
}

/* ==================== 10. 四麦轮混控、限幅、起步斜坡 ====================
 * 外部输入：x向右平移，y向前，z右摇杆顺时针旋转。
 * 当前电机前后通道整体交换，因此先x=-x，再按既有通道顺序计算v[4]。
 * v[0..3]实际对应左后、右后、左前、右前；不是标准FL/FR/RL/RR顺序。
 * 最大绝对值超过1000时同比例缩放，保留平移与旋转的比例。
 * limit=500为普通50%PWM，750为按住L1快速75%PWM；不是实测速率。
 * 20ms每帧，PWM每帧最多变化40；换向先降到0并留约40ms停顿。
 */
static void drive(int16_t x, int16_t y, int16_t z, int16_t limit)
{
    /* 实车校正：横移与原混控符号相反。仅反转横移分量；
     * 前后 y、右摇杆旋转 z 保持不变，斜向与十字键同步校正。 */
    x = -x;
    int32_t v[4] = {y+x+z, y-x-z, y-x+z, y+x-z};
    /* 先查找最大幅度；小于1000时不放大，避免把轻推右摇杆放大到满幅。 */
    int32_t max = 1000;
    for (uint8_t i = 0; i < 4; ++i) {
        int32_t a = v[i] < 0 ? -v[i] : v[i];
        if (a > max) max = a;
    }
    for (uint8_t i = 0; i < 4; ++i) {
        int16_t target = (int16_t)(v[i] * limit / max);
        /* 请求反转时先减到零，再停两个控制周期（约 40ms）。 */
        if ((applied[i] > 0 && target < 0) || (applied[i] < 0 && target > 0)) {
            target = 0; reversal_wait[i] = 2;
        }
        if (applied[i] == 0 && reversal_wait[i]) {
            --reversal_wait[i]; target = 0;
        }
        applied[i] = approach(applied[i], target, 40);
        motor_write(i, applied[i]);
    }
}

/* 11. 翻铲状态机：点动调试 -> 记下铲下和翻倒两个位置 -> 一键循环。
 * 无位置传感器：软件仅知道发出的脉宽，不知道是否真的到位、堵转或方块是否倒入框。
 * 因而自动循环只在本次使能后完成两次示教、两位置相差 >=50us 时允许。
 * 全程无 HAL_Delay，控制循环和通信检查继续运行。
 */
static void shovel_step_to(uint16_t target)
{
    g_robot_status.servo_us = (uint16_t)approach((int16_t)g_robot_status.servo_us,
                                               (int16_t)target, SHOVEL_STEP_US);
}
static uint8_t shovel_update(uint16_t buttons, uint16_t pressed, uint32_t now)
{
    /* 手动调节、记录位置或执行自动循环时，均请求底盘停轮。 */
    uint8_t was_busy = g_robot_status.shovel_state != SHOVEL_IDLE;
    const uint16_t actions = L2 | R2 | SQUARE | CIRCLE | TRIANGLE | CROSS;
    /* 13.5 更新翻铲状态，再决定本帧是否允许底盘移动。 */
    uint8_t block_drive = was_busy || ((buttons & actions) != 0);
    if (pressed & CROSS) {
        /* 取消自动循环，保持当前目标位置；不会突然回到铲下。 */
        g_robot_status.shovel_state = SHOVEL_IDLE;
    } else if (g_robot_status.shovel_state == SHOVEL_IDLE) {
        /* L2减小脉宽，R2增大；达到初始测试上下限后不再继续。 */
        int32_t value = g_robot_status.servo_us + SHOVEL_STEP_US *
                        (!!(buttons & R2) - !!(buttons & L2));
        if (value < SHOVEL_MIN_US) value = SHOVEL_MIN_US;
        if (value > SHOVEL_MAX_US) value = SHOVEL_MAX_US;
        g_robot_status.servo_us = (uint16_t)value;
        /* 用释放后新按下的按键记录位置，不连发。 */
        if (pressed & SQUARE) {
            g_robot_status.down_us = g_robot_status.servo_us;
            g_robot_status.taught_mask |= 1;
        }
        if (pressed & CIRCLE) {
            g_robot_status.dump_us = g_robot_status.servo_us;
            g_robot_status.taught_mask |= 2;
        }
        /* 两端都已记录且差值至少50us，才允许运行自动翻铲。 */
        int32_t span = (int32_t)g_robot_status.dump_us - g_robot_status.down_us;
        if (span < 0) span = -span;
        if ((pressed & TRIANGLE) && g_robot_status.taught_mask == 3 && span >= 50) {
            g_robot_status.shovel_state = SHOVEL_TO_DOWN;
        }
    } else {
        /* 每帧只推进一步，用时间戳等待，不用HAL_Delay阻塞手柄轮询。 */
        switch (g_robot_status.shovel_state) {
        case SHOVEL_TO_DOWN: /* ①缓慢移动到铲下位置 */
            shovel_step_to(g_robot_status.down_us);
            if (g_robot_status.servo_us == g_robot_status.down_us) {
                g_robot_status.shovel_state = SHOVEL_WAIT_DOWN; shovel_since = now;
            }
            break;
        case SHOVEL_WAIT_DOWN: /* ②铲下停留500ms */
            if ((uint32_t)(now - shovel_since) >= SHOVEL_SETTLE_MS)
                g_robot_status.shovel_state = SHOVEL_TO_DUMP;
            break;
        case SHOVEL_TO_DUMP: /* ③缓慢移动到翻倒位置 */
            shovel_step_to(g_robot_status.dump_us);
            if (g_robot_status.servo_us == g_robot_status.dump_us) {
                g_robot_status.shovel_state = SHOVEL_WAIT_DUMP; shovel_since = now;
            }
            break;
        case SHOVEL_WAIT_DUMP: /* ④翻倒停留800ms */
            if ((uint32_t)(now - shovel_since) >= SHOVEL_DUMP_HOLD_MS)
                g_robot_status.shovel_state = SHOVEL_RETURN;
            break;
        case SHOVEL_RETURN: /* ⑤缓慢回到铲下位置 */
            shovel_step_to(g_robot_status.down_us);
            if (g_robot_status.servo_us == g_robot_status.down_us) {
                g_robot_status.shovel_state = SHOVEL_WAIT_RETURN; shovel_since = now;
            }
            break;
        case SHOVEL_WAIT_RETURN: /* ⑥停留500ms后结束 */
            if ((uint32_t)(now - shovel_since) >= SHOVEL_SETTLE_MS)
                g_robot_status.shovel_state = SHOVEL_IDLE;
            break;
        default: g_robot_status.shovel_state = SHOVEL_IDLE; break;
        }
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, g_robot_status.servo_us);
    /* PB8 的第二路永远不驱动任何机构。 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    return block_drive;
}

/* 12. 初始化时先确保输出为零，再启动定时器；任何 HAL 错误都不允许运行。 */
void Robot_Init(void)
{
    armed = 0; previous_buttons = 0;
    ps2_att(GPIO_PIN_SET); ps2_clk(GPIO_PIN_SET); ps2_cmd(GPIO_PIN_SET);
    Robot_Stop();
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
    for (uint8_t i = 0; i < 4; ++i) {
        if (HAL_TIM_PWM_Start(&htim3, motors[i].channel) != HAL_OK) Error_Handler();
        if (HAL_TIM_PWM_Start(&htim1, motors[i].channel) != HAL_OK) Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    HAL_Delay(300);
    ps2_configure();
    last_config = HAL_GetTick(); last_poll = last_config;
}

/* 13. 控制调度：20ms 轮询一次。无效帧立即停机；主循环超过 100ms
 * 未执行本函数，恢复执行后也立即解除使能。真正防程序卡死需 IWDG。
 * 重新使能需要10个连续有效帧、右摇杆回中、其他按键释放，再新按START。
 * 左摇杆回中不作要求；软件只校验接收器返回帧，不能保证识别所有无线断链。
 */
void Robot_Task(void)
{
    /* 13.1 按20ms调度；若主循环间隔超过100ms，先解除运行许可。 */
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_poll) < 20) return;
    if ((uint32_t)(now - last_poll) > 100) Robot_Stop();
    last_poll = now;
    /* 13.2 获取一帧；包头/模式无效时立即停止，并间隔1秒重试配置。 */
    Pad p;
    if (!ps2_read(&p)) {
        g_robot_status.connected = 0;
        ++g_robot_status.bad_frames;
        Robot_Stop(); previous_buttons = 0;
        if ((uint32_t)(now - last_config) >= 1000) {
            ps2_configure(); last_config = HAL_GetTick(); last_poll = last_config;
        }
        return;
    }
    g_robot_status.connected = 1;
    g_robot_status.buttons = p.buttons;
    g_robot_status.lx = p.lx; g_robot_status.ly = p.ly;
    g_robot_status.rx = p.rx; g_robot_status.ry = p.ry;
    if (healthy_frames < 10) ++healthy_frames;
    /* 13.3 当前为1且上一帧为0的位，是新按下的按键。 */
    uint16_t pressed = p.buttons & (uint16_t)~previous_buttons;
    previous_buttons = p.buttons;
    /* 左摇杆仅保留原始读数供调试，不参与移动或 START 回中检查。
     * 十字键控制平移，同时按相邻两键可斜移；相反两键互相抵消。 */
    /* 车头改向180度：同时反转平移XY，斜向同步改变，旋转Z不变。
     * drive内部的x反号仍是前后电机通道对调补偿，两个修正用途不同。 */
    int16_t x = 1000 * (!!(p.buttons & LEFT) - !!(p.buttons & RIGHT));
    int16_t y = 1000 * (!!(p.buttons & DOWN) - !!(p.buttons & UP));
    /* 十字键优先：只要任一方向键按住，就强制旋转分量为0。
     * 即使右摇杆回中偏差超出死区，也不允许给平移叠加转向。
     * 松开全部方向键后，右摇杆才可单独转向；不再边平移边旋转。
     * 相反方向键同时按下时平移相互抵消，也不回退到摇杆旋转。
     */
    uint8_t dpad_active = (p.buttons & (UP | DOWN | LEFT | RIGHT)) != 0;
    int16_t z = dpad_active ? 0 : axis(p.rx);
    /* 13.4 SELECT优先停机；未允许运行时只检查START条件。 */
    if (p.buttons & SELECT) { Robot_Stop(); return; }
    if (!armed) {
        if (healthy_frames >= 10 && (pressed & START) && !x && !y && !z &&
            !(p.buttons & (uint16_t)~START)) {
            armed = 1; g_robot_status.armed = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_Delay(2); /* DRV8833 从 nSLEEP 唤醒需等待；PWM 此时仍为 0。 */
        }
        return;
    }
    /* 13.5 更新翻铲状态，再决定本帧是否允许底盘移动。 */
    uint8_t block_drive = shovel_update(p.buttons, pressed, now);
    if (block_drive) drive_rearm_needed = 1;
    if (!(p.buttons & R1)) drive_rearm_needed = 0;
    /* R1 为底盘保持运行键。松开或翻铲过程中立即停轮，不经过斜坡。 */
    if (!(p.buttons & R1) || block_drive || drive_rearm_needed) {
        for (uint8_t i = 0; i < 4; ++i) {
            applied[i] = 0; reversal_wait[i] = 0; motor_write(i, 0);
        }
    } else {
        /* R1保持运行；L1按住为75%快速，松开恢复50%普通。 */
        drive(x, y, z, (p.buttons & L1) ? ROBOT_FAST_LIMIT : ROBOT_DRIVE_LIMIT);
    }
}
