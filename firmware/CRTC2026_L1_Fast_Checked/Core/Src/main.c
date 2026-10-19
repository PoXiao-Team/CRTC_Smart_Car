/**
 * @file main.c
 * @brief 硬件初始化和主循环：建立时钟、GPIO、PWM、看门狗，再调用robot模块。
 * A：释放JTAG引脚；B：64MHz时钟；C：GPIO；D：四个定时器；E：看门狗；
 * G：主程序和状态灯；F：中断与故障处理。业务控制细节见robot.c。
 * 本工程使用HAL库，不使用RTOS，不依赖外部8MHz晶振。
 */

#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "robot.h"

TIM_HandleTypeDef htim1, htim2, htim3, htim4;
IWDG_HandleTypeDef hiwdg;
static uint8_t board_ready;
volatile uint32_t g_reset_flags;
volatile uint32_t g_loop_heartbeat;
volatile uint32_t g_fault_tag;
/* 启动汇编已负责 .data/.bss；无 C++ 构造器，提供 newlib 初始化钩子。 */
void _init(void) {}
void _fini(void) {}

/* A. HAL 公共初始化：启用 AFIO，关闭 JTAG 但保留 SWD 调试。 */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    /* PS2占用PB3/PB4，必须释放JTAG；PA13/PA14的SWD下载功能仍保留。 */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/* B. 内部 HSI/2×16=64MHz，APB1=32MHz，TIM2/3/4 时钟=64MHz。 */
static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSIState = RCC_LSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* C. 在配置输出之前写入初始电平，防止使能引脚意外脉冲。 */
static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_6; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
    /* PWM 引脚先保持下拉输入，初始化定时器后才切换到复用输出。 */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN;
    g.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; HAL_GPIO_Init(GPIOB, &g);
    /* 常见 F103C8 最小系统板 PC13 LED 为低电平亮；不是手柄配对灯。 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_13; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOC, &g);
}

/* D. TIM2 微秒时基；TIM1/TIM3 各四路 1kHz PWM；TIM4 三路 50Hz PWM。
 * 当前只有 TIM4 CH2 使用；CH3/CH4 始终输出 0，留作两路舵机扩展。
 */
static void timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* D1. TIM2：64MHz/(63+1)=1MHz，每计数1us，供PS2通信延时使用。 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 63; htim2.Init.Period = 65535;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    TIM_ClockConfigTypeDef source = {0};
    source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &source) != HAL_OK) Error_Handler();
    /* D2. TIM1/TIM3：64MHz/64/1000=1kHz，合计8路PWM驱动4个电机。 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63; htim1.Init.Period = 999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 63; htim3.Init.Period = 999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
    /* D3. TIM4：64MHz/64/20000=50Hz；CCR直接表示高电平微秒数。 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 63; htim4.Init.Period = 19999;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();
    TIM_OC_InitTypeDef pwm = {0};
    pwm.OCMode = TIM_OCMODE_PWM1; pwm.Pulse = 0;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH; pwm.OCFastMode = TIM_OCFAST_DISABLE;
    const uint32_t channels[] = {TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
    for (unsigned i=0; i<4; ++i) {
        if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm, channels[i]) != HAL_OK) Error_Handler();
        if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, channels[i]) != HAL_OK) Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    /* D4. 外设初始化结束后，才把对应引脚切换为定时器复用输出。 */
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA,&g);
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; HAL_GPIO_Init(GPIOB,&g);
    board_ready = 1;
}

/* E. 主循环卡死或 HardFault 时不喂狗，约 1 秒复位。 */
static void watchdog_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 624;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}
/* G. 录制视频可先展示这里：初始化只执行一次，while循环持续处理手柄。 */
int main(void)
{
    /* 先保存本次启动的RCC复位来源，再清除标志，便于识别下次复位。
     * RCC_CSR_IWDGRSTF表示看门狗复位；PORRSTF表示上电/掉电复位标志。
     * 单个标志不能定位瞬态供电故障，仍需电压测量。
     */
    g_reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    HAL_Init(); /* 初始化HAL并建立SysTick毫秒时基。 */
    watchdog_init(); /* 提前启动：覆盖后续时钟和外设初始化异常。
                     * HAL_IWDG_Init会强制开启LSI，无需等clock_init。 */
    clock_init();    /* 系统和外设时钟。 */
    gpio_init();     /* 电平和输入/输出模式。 */
    timer_init();    /* 电机PWM、舵机PWM、微秒计时。 */
    Robot_Init(); /* 启动各PWM通道，保持停机，然后尝试配置PS2。 */
    while (1) {
        Robot_Task(); /* 内部按20ms执行一帧；不要在循环内增加长时间HAL_Delay。 */
        ++g_loop_heartbeat; /* 持续递增表明主循环仍在执行，不代表电源或无线必然正常。 */
        uint32_t tick = HAL_GetTick();
        /* PC13低电平亮：快闪=无有效帧，慢闪=有帧未启动，常亮=已START。
         * 接收器即使无线未配对也可能返回有效帧，不能用此灯证明无线已配对。 */
        uint8_t led_on = g_robot_status.armed ? 1 :
                         (uint8_t)((tick / (g_robot_status.connected ? 500u : 100u)) & 1u);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        /* 仅主循环执行到这里才喂狗；中断里不喂狗。 */
        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) Error_Handler();
    }
}

/* F. SysTick 为 HAL 毫秒时基；异常先停机，再等待看门狗复位。 */
void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void)
{
    if (g_fault_tag == 0u) g_fault_tag = 1u; /* 通用HAL/初始化错误 */
    if (board_ready) Robot_Stop();
    __disable_irq();
    while (1) {}
}
void HardFault_Handler(void) { g_fault_tag = 3u; Error_Handler(); }
void MemManage_Handler(void) { g_fault_tag = 4u; Error_Handler(); }
void BusFault_Handler(void) { g_fault_tag = 5u; Error_Handler(); }
void UsageFault_Handler(void) { g_fault_tag = 6u; Error_Handler(); }
void NMI_Handler(void) { g_fault_tag = 7u; Error_Handler(); }
