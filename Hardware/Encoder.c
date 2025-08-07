#include "stm32f10x.h"

/**
  * 函    数：编码器初始化（TIM3-PA6/PA7 和 TIM4-PB6/PB7）
  * 参    数：无
  * 返 回 值：无
  */
  

void Encoder_Init(void)
{
    /* 1. 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);  // 同时使能 TIM3 和 TIM4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); // 使能 GPIOA 和 GPIOB

    /* 2. GPIO 初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    // 上拉输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* PA6 (TIM3_CH1), PA7 (TIM3_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PB6 (TIM4_CH1), PB7 (TIM4_CH2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 3. 时基单元初始化（TIM3 和 TIM4 配置相同） */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;     // 16 位最大值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;      // 不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);  // 初始化 TIM3
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);  // 初始化 TIM4

    /* 4. 输入捕获初始化（TIM3 和 TIM4） */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xF;          // 滤波器参数（抗抖动）

    /* TIM3 通道1 (PA6) 和 通道2 (PA7) */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* TIM4 通道1 (PB6) 和 通道2 (PB7) */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    /* 5. 编码器接口配置 */
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* 6. 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
/**
  * 函    数：获取指定编码器的增量值
  * 参    数：EncoderID - 编码器编号（1 或 2）
  * 返 回 值：自上此调用后，编码器的增量值（int16_t 范围）
  */
int16_t Encoder_Get(uint8_t EncoderID)
{
    int16_t Temp = 0;
    if (EncoderID == 1) {
        Temp = TIM_GetCounter(TIM3);  // 读取 TIM3 的计数器值
        TIM_SetCounter(TIM3, 0);      // 清零 TIM3
    } else if (EncoderID == 2) {
        Temp = TIM_GetCounter(TIM4);  // 读取 TIM4 的计数器值
        TIM_SetCounter(TIM4, 0);      // 清零 TIM4
    }
    return Temp;
}
