//#ifndef __CONTROL_H
//#define __CONTROL_H

//#include "stm32f10x.h"

//// ����ģʽö��
//typedef enum {
//    CONTROL_MODE_STOP = 0,      // ֹͣģʽ
//    CONTROL_MODE_BALANCE,       // ��ƽ��ģʽ
//    CONTROL_MODE_LINE_TRACK,    // ѭ��ƽ��ģʽ
//    CONTROL_MODE_MANUAL,         // �ֶ�ģʽ
//	CONTROL_MODE_PURE_LINE     // ��ѭ��ģʽ������ƽ�⣩
//} ControlMode_t;

//// ����״̬�ṹ��
//typedef struct {
//    ControlMode_t CurrentMode;  // ��ǰ����ģʽ
//    ControlMode_t LastMode;     // ��һ������ģʽ
//    uint8_t BalanceEnable;      // ƽ�����ʹ�ܱ�־
//    uint8_t LineTrackEnable;    // ѭ������ʹ�ܱ�־
//    float BalanceTarget;        // ƽ��Ŀ��Ƕ�
//    float LineTrackSpeed;       // ѭ�������ٶ�
//} ControlState_t;

//// ȫ�ֿ���״̬����
//extern ControlState_t ControlState;

//// ����ģ�麯������
//void Control_Init(void);
//void Control_SetMode(ControlMode_t mode);
//ControlMode_t Control_GetMode(void);
//void Control_Update(void);

//// ƽ����ƺ���
//void Balance_Control(void);
//void Balance_Enable(uint8_t enable);
//void Balance_SetTarget(float target);

//// ѭ��ƽ����ƺ���
//void LineTrack_Balance_Control(void);
//void LineTrack_Enable(uint8_t enable);
//void LineTrack_SetSpeed(float speed);

//#endif
#ifndef __MPUEXTI_H
#define __MPUEXTI_H

#include "PID.h"
void MPU_exti_init(void);
extern PID_t pid_balance;  // ��̬��ƽ�⣩��
extern PID_t pid_speed;    // �ٶȻ�
extern PID_t pid_turn;     // ת��
#endif
