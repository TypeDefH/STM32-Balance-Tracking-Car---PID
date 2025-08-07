#ifndef _SENSOR_H_
#define _SENSOR_H_

// 传感器状态数组
extern uint8_t SensorState[4];

// 区域状态变量
extern uint8_t Area;
extern uint8_t Last_Area;

// 基础传感器函数
void Sensor_Init(void);
uint8_t Sensor0_Get_State(void);
uint8_t Sensor1_Get_State(void);
uint8_t Sensor2_Get_State(void);
uint8_t Sensor3_Get_State(void);

// 传感器状态管理函数
void Sensor_UpdateAll(void);
void Sensor_GetStates(uint8_t *states);

// 小车运动控制函数
void Car_Forward(uint16_t speed);
void Car_TurnRight(uint16_t speed);
void Car_TurnLeft(uint16_t speed);
void Car_Stop(void);

// 循迹控制函数
void Sensor_LineTracking(void);
uint8_t Sensor_GetArea(void);
uint8_t Sensor_GetLastArea(void);

#endif
