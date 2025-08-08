
#include <stm32f10x.h>
#include "control.h"
//#include "sys.h"
#include "MPU6050.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Delay.h"
#include "Motor.h"
#include "Timer.h"
#include "Encoder.h"
#include "Sensor.h"
#include "PID.h"

float pitch,roll,yaw;	//ŷ����  
float Speed, SP_Left, SP_Right, filt_Speed, last_filt_Speed, Speed_sum, speed_target, speed_actual, speed_error;//�ٶȻ�����
float Target,Target2,Actual,Actual2,Error0,Error1,Error02,Error12,ErrorInt,ErrorInt2;//ֱ��������
float turn_target, turn_actual, turn_kp, turn_error;//ת�򻷲���
float Out,Out2,speed_out,speed_out2, turn_out, pwm_left, pwm_right;//�����������
extern int Motor_OFF;//��������������ر�


void MPU_exti_init() {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. ʹ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // ���뿪��AFIOʱ��
    
    // 2. ����PA5Ϊ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;            // �޸�ΪPA5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. ��PA5ӳ�䵽�ⲿ�ж���5
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);  // �޸�ΪPinSource5
    
    // 4. ����EXTI5�ж�
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;           // �޸�ΪLine5
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // �½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 5. ����NVIC�жϣ�EXTI9_5_IRQn��PA5~PA9�Ĺ����жϣ�
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   // ע�⣺PA5��ʹ��EXTI9_5_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
PID_t pid_balance;  // ��̬��ƽ�⣩��
PID_t pid_speed;    // �ٶȻ�
PID_t pid_turn;     // ת��

// �ⲿ�ж���10�������PID�����ڴ˺�����
void EXTI9_5_IRQHandler(void)
{
	if (MPU6050_ReadDMP(&pitch, &roll, &yaw) == 0)
	{
		//=== ��̬�� PID�����Ŀ���ٶȣ� ===//
		pid_balance.Target = 0;
		pid_balance.Actual = roll;
		PID_Update(&pid_balance);

		float speed_target = pid_balance.Out;

		//=== �ٶȲɼ����˲� ===//
		float SP_Left = Encoder_Get(1);
		float SP_Right = Encoder_Get(2);
		float Speed = (SP_Left + SP_Right) / 2.0f;

		float a = 0.3f;
		filt_Speed = a * Speed + (1 - a) * last_filt_Speed;
		last_filt_Speed = filt_Speed;

		Speed_sum += filt_Speed;
		if (Speed_sum > 3000) Speed_sum = 3000;
		if (Speed_sum < -3000) Speed_sum = -3000;

		//=== �ٶȻ� PID ===//
		pid_speed.Target = speed_target;
		pid_speed.Actual = filt_Speed;
		pid_speed.ErrorInt = Speed_sum;
		PID_Update(&pid_speed);

		//=== ת�� PID ===//
		pid_turn.Target = turn_target;  // Ĭ���趨Ϊ 0������ͨ��ң�����趨
		pid_turn.Actual = roll;
		PID_Update(&pid_turn);

		//=== �ϳ������� PWM ��� ===//
		float pwm_left  = pid_speed.Out;// + pid_turn.Out;
		float pwm_right = -pid_speed.Out;// - pid_turn.Out;

		//=== PWM �޷������������ ===//
		if (pwm_left > 2000) pwm_left = 2000;
		if (pwm_left < -2000) pwm_left = -2000;
		if (pwm_right > 2000) pwm_right = 2000;
		if (pwm_right < -2000) pwm_right = -2000;

		//=== �������� ===//
		if (Motor_OFF == 1)
		{
			Motor_SetPWM(1, 0);
			Motor_SetPWM(2, 0);
		}
		else
		{
			Motor_SetPWM(1, pwm_left);
			Motor_SetPWM(2, pwm_right);
		}
	}

	// ����жϱ�־λ
	EXTI_ClearITPendingBit(EXTI_Line5);
}
//����pid
//void EXTI9_5_IRQHandler(void)
//{
//	if(MPU6050_ReadDMP(&pitch,&roll,&yaw)==0)
//	{ 	
////λ�û�
//			float Kp= 480 ,Kd= 2800 ;
//			Target2 = 0;
//		  Target  = 0;
//			Actual =  roll;
//			Actual2 = -roll;			
//			Error1 = Error0;
//			Error0 = Target - Actual;												 //���=Ŀ��ֵ-ʵ��ֵ
//			Error12 = Error02;
//			Error02 = Target2 - Actual2;	
//		
//			Out = Kp * Error0 + Kd * (Error0 - Error1);
//			Out2 = Kp * Error02 + Kd * (Error02 - Error12);				
//			
////�ٶȻ�
//			float vkp= 310 , vki= 1.55;
//			SP_Left= Encoder_Get(1); SP_Right=Encoder_Get(2);												//�������ٶ�
//			Speed=(SP_Left+SP_Right)/2;											//ƽ���ٶ�
//			float a=0.3;																		//�ٶȻ��˲�����0.3
//			filt_Speed = a*Speed + (1-a)*last_filt_Speed;		//�˲�
//			Speed_sum +=  filt_Speed;												//�ٶȵ��ۼ�
//			last_filt_Speed = filt_Speed;										//�˴��ٶȼ�¼Ϊ�ϴ��ٶ�
//			if(Speed_sum>3000){Speed_sum=3000;}							//�����޷�
//			if(Speed_sum<-3000){Speed_sum=-3000;}
//			
//			speed_target = 0 ;
//			speed_actual = filt_Speed ;
//			speed_error = speed_target - speed_actual ;
//			
//			speed_out  = vkp*speed_error - vki*Speed_sum;
//			speed_out2 = -vkp*speed_error + vki*Speed_sum;

////ת��
//			float turn_kp=80;
////			turn_target=0;	//ת��Ŀ��ֵ�趨Ϊ0
//			turn_actual=yaw;	// ת��ʵ��ֵ=������yaw��
//			turn_error=turn_target-turn_actual;//ת�����
//			
//			turn_out=turn_kp*turn_error;
//			
////��������޷�
//			Total_out	= Out + speed_out + turn_out;
//			Total_out2	= Out2 + speed_out2 + turn_out;		
////			if (Total_out > 7200) {Total_out = 7200;}
////			if (Total_out < -7200) {Total_out = -7200;}
////			if (Total_out2 > 7200) {Total_out2 = 7200;}
////			if (Total_out2 < -7200) {Total_out2 = -7200;}	
//			if (Total_out > 2000) {Total_out = 2000;}
//			if (Total_out < -2000) {Total_out = -2000;}
//			if (Total_out2 > 2000) {Total_out2 = 2000;}
//			if (Total_out2 < -2000) {Total_out2 = -2000;}			
////δ��������װ��ʱ�������ֵ��Ϊ���PWM
//			if(Motor_OFF==1)				
//			{Motor_SetPWM(1,0);Motor_SetPWM(2,0);}
//			else
//			{Motor_SetPWM(1,Total_out);	Motor_SetPWM(2,Total_out2);}
//	}
//	EXTI_ClearITPendingBit(EXTI_Line5); //����жϱ�־λ
//}


//#include "Control.h"
//#include "PID.h"
//#include "MPU6050.h"
//#include "Motor.h"
//#include "Encoder.h"
//#include "Sensor.h"

//// ȫ�ֿ���״̬����
//ControlState_t ControlState;
//// ƽ�����PID�ṹ�壨��main.c�ƶ�������
//extern PID_t Inner;  // �ڻ����ٶȻ���
//extern PID_t Outer;  // �⻷���ǶȻ���

//extern float Pitch,Roll,Yaw;

//// ѭ��ƽ����ƵĶ������
//static float LineTrack_LeftSpeed = 0;
//static float LineTrack_RightSpeed = 0;
//static float Balance_Output = 0;

///**
//  * @brief  ����ģ���ʼ��
//  * @param  ��
//  * @retval ��
//  */
//void Control_Init(void)
//{
//    // ��ʼ������״̬
//    ControlState.CurrentMode = CONTROL_MODE_STOP;
//    ControlState.LastMode = CONTROL_MODE_STOP;
//    ControlState.BalanceEnable = 0;
//    ControlState.LineTrackEnable = 0;
//    ControlState.BalanceTarget = 0.0f;
//    ControlState.LineTrackSpeed = 5.0f;
//    
//    // ����PID���
//    Inner.Out = 0;
//    Outer.Out = 0;
//    Balance_Output = 0;
//}

///**
//  * @brief  ���ÿ���ģʽ
//  * @param  mode Ŀ�����ģʽ
//  * @retval ��
//  */
//void Control_SetMode(ControlMode_t mode)
//{
//    if (mode != ControlState.CurrentMode)
//    {
//        ControlState.LastMode = ControlState.CurrentMode;
//        ControlState.CurrentMode = mode;
//        
//        // ����ģʽ����ʹ�ܱ�־
//        switch (mode)
//        {
//            case CONTROL_MODE_STOP:
//                Balance_Enable(0);
//                LineTrack_Enable(0);
//                Car_Stop();
//                break;
//                
//            case CONTROL_MODE_BALANCE:
//                Balance_Enable(1);
//                LineTrack_Enable(0);
//                break;
//			
//		    case CONTROL_MODE_PURE_LINE: 
//                Balance_Enable(0);
//                LineTrack_Enable(1);
//                break;
//			
//            case CONTROL_MODE_LINE_TRACK:
//                Balance_Enable(1);
//                LineTrack_Enable(1);
//                break;
//                
//            case CONTROL_MODE_MANUAL:
//                Balance_Enable(0);
//                LineTrack_Enable(0);
//                break;
//                
//            default:
//                Control_SetMode(CONTROL_MODE_STOP);
//                break;
//        }
//    }
//}

///**
//  * @brief  ��ȡ��ǰ����ģʽ
//  * @param  ��
//  * @retval ��ǰ����ģʽ
//  */
//ControlMode_t Control_GetMode(void)
//{
//    return ControlState.CurrentMode;
//}

///**
//  * @brief  ���Ƹ������������ڶ�ʱ���ж��е��ã�
//  * @param  ��
//  * @retval ��
//  */
//void Control_Update(void)
//{
//    switch (ControlState.CurrentMode)
//    {
//        case CONTROL_MODE_BALANCE:
//            Balance_Control();
//            break;
//            
//        case CONTROL_MODE_LINE_TRACK:
//            LineTrack_Balance_Control();
//            break;
//            
//		case CONTROL_MODE_PURE_LINE:  
//            Sensor_LineTracking();
//            break;
//		
//        case CONTROL_MODE_STOP:
//        case CONTROL_MODE_MANUAL:
//        default:
//            // ֹͣ���ֶ�ģʽ��ִ���Զ�����
//            break;
//    }
//}

///**
//  * @brief  ��ƽ����ƺ���
//  * @param  ��
//  * @retval ��
//  */
////float pitch,roll,yaw;	//ŷ����  
////float Speed, SP_Left, SP_Right, filt_Speed, last_filt_Speed, Speed_sum, speed_target, speed_actual, speed_error;//�ٶȻ�����
////float Target,Target2,Actual,Actual2,Error0,Error1,Error02,Error12,ErrorInt,ErrorInt2;//ֱ��������
////float turn_target, turn_actual, turn_kp, turn_error;//ת�򻷲���
////float Out,Out2,speed_out,speed_out2, turn_out, Total_out, Total_out2;//�����������
////extern int Motor_OFF;//��������������ر�
//float SP_Left, SP_Right;
//void Balance_Control(void)
//{

//    if (!ControlState.BalanceEnable) return;
//    
//    static int16_t Speed, Location;
//    //float Pitch, Roll, Yaw;
//    
//    //��ȡMPU6050��̬����
//    MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
//    
//    // ��ȡ�������ٶ�
//	Speed = (Encoder_Get(1) + Encoder_Get(2)) / 2;
//    
//    // �⻷���ǶȻ�������
//    Outer.Target = ControlState.BalanceTarget;  // ƽ��Ŀ��Ƕ�
//    Outer.Actual = Roll;  // ʹ�ø�������Ϊ����
//    PID_Update(&Outer);
//    
//    // �ڻ����ٶȻ�������
//    Inner.Target = Outer.Out;  // �⻷�����Ϊ�ڻ�Ŀ��
//    Inner.Actual = Speed;
//    PID_Update(&Inner);
//    
//    // ��������
//    Motor_SetPWM(1, Inner.Out);
//    Motor_SetPWM(2, Inner.Out);
//    
//    Balance_Output = Inner.Out;
////			float Kp= 480 ,Kd= 2800 ;
////			Target2 = 0;
////		    Target  = 0;
////			Actual =  pitch;
////			Actual2 =  -pitch;			
////			Error1 = Error0;
////			Error0 = Target - Actual;												 //���=Ŀ��ֵ-ʵ��ֵ
////			Error12 = Error02;
////			Error02 = Target2 - Actual2;	
////		
////			Out = Kp * Error0 + Kd * (Error0 - Error1);
////			Out2 = Kp * Error02 + Kd * (Error02 - Error12);				
////			
//////�ٶȻ�
////			float vkp= 310 , vki= 1.55;
////			SP_Left= Encoder_Get();SP_Right=	Encoder2_Get();												//�������ٶ�
////			Speed=(SP_Left+SP_Right)/2;											//ƽ���ٶ�
////			float a=0.3;																		//�ٶȻ��˲�����0.3
////			filt_Speed = a*Speed + (1-a)*last_filt_Speed;		//�˲�
////			Speed_sum +=  filt_Speed;												//�ٶȵ��ۼ�
////			last_filt_Speed = filt_Speed;										//�˴��ٶȼ�¼Ϊ�ϴ��ٶ�
////			if(Speed_sum>3000){Speed_sum=3000;}							//�����޷�
////			if(Speed_sum<-3000){Speed_sum=-3000;}
////			
////			speed_target = 0 ;
////			speed_actual = filt_Speed ;
////			speed_error = speed_target - speed_actual ;
////			
////			speed_out  = vkp*speed_error - vki*Speed_sum;
////			speed_out2 = -vkp*speed_error + vki*Speed_sum;

//////ת��
////			float turn_kp=80;
//////			turn_target=0;	//ת��Ŀ��ֵ�趨Ϊ0
////			turn_actual=yaw;	// ת��ʵ��ֵ=������yaw��
////			turn_error=turn_target-turn_actual;//ת�����
////			
////			turn_out=turn_kp*turn_error;
////			
//////��������޷�
////			Total_out	= Out + speed_out + turn_out;
////			Total_out2	= Out2 + speed_out2 + turn_out;		
////			if (Total_out > 7200) {Total_out = 7200;}
////			if (Total_out < -7200) {Total_out = -7200;}
////			if (Total_out2 > 7200) {Total_out2 = 7200;}
////			if (Total_out2 < -7200) {Total_out2 = -7200;}	
////			
//////δ��������װ��ʱ�������ֵ��Ϊ���PWM
////			if(Motor_OFF==1)
////			{Motor_SetPWM(1,0);Motor_SetPWM(2,0);}
////			else
////			{Motor_SetPWM(1,Total_out);Motor_SetPWM(2,Total_out2);}
//}

///**
//  * @brief  ѭ��ƽ����ƺ���
//  * @param  ��
//  * @retval ��
//  */
//void LineTrack_Balance_Control(void)
//{
//    if (!ControlState.BalanceEnable || !ControlState.LineTrackEnable) return;
//    
//    static int16_t Speed, Location;
//    //float Pitch, Roll, Yaw;
//    uint8_t area;
//    
//    // ��ȡMPU6050��̬����
//    MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
//    
//    // ��ȡ�������ٶ�
//    Speed = Encoder_Get(2);
//    Location += Speed;
//    
//    // ���´�����״̬����ȡ������Ϣ
//    Sensor_UpdateAll();
//    area = Sensor_GetArea();
//    
//    // ����ѭ��״̬����������Ŀ���ٶ�
//    switch (area)
//    {
//        case 3:  // ֱ��
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed;
//            break;
//            
//        case 4:  // ��С��
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 1.2f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 0.6f;
//            break;
//            
//        case 6:  // �Ҵ���
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 1.5f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 0.3f;
//            break;
//            
//        case 2:  // ��С��
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 0.6f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 1.2f;
//            break;
//            
//        case 0:  // �����
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 0.3f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 1.5f;
//            break;
//            
//        default:
//            LineTrack_LeftSpeed = 0;
//            LineTrack_RightSpeed = 0;
//            break;
//    }
//    
//    // ƽ����ƣ��ǶȻ���
//    Outer.Target = ControlState.BalanceTarget;
//    Outer.Actual = Roll;
//    PID_Update(&Outer);
//    
//    Balance_Output = Outer.Out;
//    
//    // ��ƽ�������ѭ�����ƽ��
//    float LeftOutput = Balance_Output + LineTrack_LeftSpeed;
//    float RightOutput = Balance_Output + LineTrack_RightSpeed;
//    
//    // ����޷�
//    if (LeftOutput > 100) LeftOutput = 100;
//    if (LeftOutput < -100) LeftOutput = -100;
//    if (RightOutput > 100) RightOutput = 100;
//    if (RightOutput < -100) RightOutput = -100;
//    
//    // ��������
//    Motor_SetPWM(1, (int16_t)LeftOutput);   // ����
//    Motor_SetPWM(2, (int16_t)RightOutput);  // �ҵ��
//}

///**
//  * @brief  ʹ��/����ƽ�����
//  * @param  enable 1-ʹ�ܣ�0-����
//  * @retval ��
//  */
//void Balance_Enable(uint8_t enable)
//{
//    ControlState.BalanceEnable = enable;
//    if (!enable)
//    {
//        // ����ʱ����PID
//        Inner.Out = 0;
//        Outer.Out = 0;
//        Inner.ErrorInt = 0;
//        Outer.ErrorInt = 0;
//    }
//}

///**
//  * @brief  ����ƽ��Ŀ��Ƕ�
//  * @param  target Ŀ��Ƕ�
//  * @retval ��
//  */
//void Balance_SetTarget(float target)
//{
//    ControlState.BalanceTarget = target;
//}

///**
//  * @brief  ʹ��/����ѭ������
//  * @param  enable 1-ʹ�ܣ�0-����
//  * @retval ��
//  */
//void LineTrack_Enable(uint8_t enable)
//{
//    ControlState.LineTrackEnable = enable;
//}

///**
//  * @brief  ����ѭ�������ٶ�
//  * @param  speed �����ٶ�
//  * @retval ��
//  */
//void LineTrack_SetSpeed(float speed)
//{
//    ControlState.LineTrackSpeed = speed;
//}
