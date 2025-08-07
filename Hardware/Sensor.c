#include "stm32f10x.h"
#include "Sensor.h"
#include "Motor.h"
#include "Delay.h"

// 传感器状态数组
uint8_t SensorState[4];

// 区域状态变量
uint8_t Area = 0;
uint8_t Last_Area = 0;

/*
    SENSOR0(B1)  SENSOR1(B0)  SENSOR2(B4)  SENSOR3(B5)
*/
void Sensor_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化循迹变量
    Area = 0;
    Last_Area = 0;
    for(int i = 0; i < 4; i++)
    {
        SensorState[i] = 0;
    }
}

uint8_t Sensor0_Get_State()
{
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
}

uint8_t Sensor1_Get_State()
{
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
}

uint8_t Sensor2_Get_State()
{
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
}

uint8_t Sensor3_Get_State()
{
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);
}

/**
  * @brief  更新所有传感器状态
  * @param  无
  * @retval 无
  */
void Sensor_UpdateAll(void)
{
    SensorState[0] = Sensor0_Get_State();
    SensorState[1] = Sensor1_Get_State();
    SensorState[2] = Sensor2_Get_State();
    SensorState[3] = Sensor3_Get_State();
}

/**
  * @brief  获取传感器状态数组
  * @param  states 传感器状态数组指针
  * @retval 无
  */
void Sensor_GetStates(uint8_t *states)
{
    for(int i = 0; i < 4; i++)
    {
        states[i] = SensorState[i];
    }
}

/**
  * @brief  小车前进
  * @param  speed 前进速度
  * @retval 无
  */
void Car_Forward(uint16_t speed)
{
    Motor_SetPWM(1, speed);   // 左电机正转
    Motor_SetPWM(2, speed);   // 右电机正转
}

/**
  * @brief  小车右转
  * @param  speed 转向速度
  * @retval 无
  */
void Car_TurnRight(uint16_t speed)
{
    Motor_SetPWM(1, speed);   // 左电机快速
    Motor_SetPWM(2, speed/2); // 右电机慢速
}

/**
  * @brief  小车左转
  * @param  speed 转向速度
  * @retval 无
  */
void Car_TurnLeft(uint16_t speed)
{
    Motor_SetPWM(1, speed/2); // 左电机慢速
    Motor_SetPWM(2, speed);   // 右电机快速
}

/**
  * @brief  小车停止
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
}

/**
  * @brief  循迹控制主函数
  * @param  无
  * @retval 无
  */
void Sensor_LineTracking(void)
{
    Sensor_UpdateAll();
    uint16_t base_speed = 12; // 初始速度

    uint8_t state = (SensorState[0]<<3)|(SensorState[1]<<2)|(SensorState[2]<<1)|SensorState[3];

    switch(state)
    {
        case 0x9: // 0b1001 - 直线
            Area = 3;
            Car_Forward(base_speed);
            break;
        case 0xD: // 0b1101 - 右小弯
            Area = 4;
            Car_TurnRight(base_speed * 1.0f); // 右小弯倍率1.0
            break;
        case 0xE: // 0b1110 - 右大弯
            Area = 6;
            Car_TurnRight(base_speed * 1.25f); // 右大弯倍率1.5
            break;
        case 0xB: // 0b1011 - 左小弯
            Area = 2;
            Car_TurnLeft(base_speed * 1.0f); // 左小弯倍率1.0
            break;
        case 0x7: // 0b0111 - 左大弯
            Area = 0;
            Car_TurnLeft(base_speed * 1.25f); // 左大弯倍率1.5
            break;
        case 0x0: // 0b0000 - 全黑
            if (Last_Area == 4) {
                Area = 5;
                Car_TurnRight(base_speed * 1.125f); // 全黑右转倍率1.25
            } else if (Last_Area == 2) {
                Area = 1;
                Car_TurnLeft(base_speed * 1.125f); // 全黑左转倍率1.25
            } else {
                Car_Stop();
            }
            break;
        case 0xF: // 0b1111 - 全白
            Car_Stop();
            break;
        default:
            Car_Stop();
            break;
    }
    Last_Area = Area;
}

/**
  * @brief  获取当前区域状态
  * @param  无
  * @retval 当前区域编号
  */
uint8_t Sensor_GetArea(void)
{
    return Area;
}

/**
  * @brief  获取上一个区域状态
  * @param  无
  * @retval 上一个区域编号
  */
uint8_t Sensor_GetLastArea(void)
{
    return Last_Area;
}

