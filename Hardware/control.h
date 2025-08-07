//#ifndef __CONTROL_H
//#define __CONTROL_H

//#include "stm32f10x.h"

//// 控制模式枚举
//typedef enum {
//    CONTROL_MODE_STOP = 0,      // 停止模式
//    CONTROL_MODE_BALANCE,       // 纯平衡模式
//    CONTROL_MODE_LINE_TRACK,    // 循迹平衡模式
//    CONTROL_MODE_MANUAL,         // 手动模式
//	CONTROL_MODE_PURE_LINE     // 纯循迹模式（不带平衡）
//} ControlMode_t;

//// 控制状态结构体
//typedef struct {
//    ControlMode_t CurrentMode;  // 当前控制模式
//    ControlMode_t LastMode;     // 上一个控制模式
//    uint8_t BalanceEnable;      // 平衡控制使能标志
//    uint8_t LineTrackEnable;    // 循迹控制使能标志
//    float BalanceTarget;        // 平衡目标角度
//    float LineTrackSpeed;       // 循迹基础速度
//} ControlState_t;

//// 全局控制状态变量
//extern ControlState_t ControlState;

//// 控制模块函数声明
//void Control_Init(void);
//void Control_SetMode(ControlMode_t mode);
//ControlMode_t Control_GetMode(void);
//void Control_Update(void);

//// 平衡控制函数
//void Balance_Control(void);
//void Balance_Enable(uint8_t enable);
//void Balance_SetTarget(float target);

//// 循迹平衡控制函数
//void LineTrack_Balance_Control(void);
//void LineTrack_Enable(uint8_t enable);
//void LineTrack_SetSpeed(float speed);

//#endif
#ifndef __MPUEXTI_H
#define __MPUEXTI_H

#include "PID.h"
void MPU_exti_init(void);
extern PID_t pid_balance;  // 姿态（平衡）环
extern PID_t pid_speed;    // 速度环
extern PID_t pid_turn;     // 转向环
#endif
