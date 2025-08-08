
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

float pitch,roll,yaw;	//欧拉角  
float Speed, SP_Left, SP_Right, filt_Speed, last_filt_Speed, Speed_sum, speed_target, speed_actual, speed_error;//速度环参数
float Target,Target2,Actual,Actual2,Error0,Error1,Error02,Error12,ErrorInt,ErrorInt2;//直立环参数
float turn_target, turn_actual, turn_kp, turn_error;//转向环参数
float Out,Out2,speed_out,speed_out2, turn_out, pwm_left, pwm_right;//各环及总输出
extern int Motor_OFF;//声明变量：电机关闭


void MPU_exti_init() {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 必须开启AFIO时钟
    
    // 2. 配置PA5为上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;            // 修改为PA5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 将PA5映射到外部中断线5
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);  // 修改为PinSource5
    
    // 4. 配置EXTI5中断
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;           // 修改为Line5
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 5. 配置NVIC中断（EXTI9_5_IRQn是PA5~PA9的共享中断）
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   // 注意：PA5仍使用EXTI9_5_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
PID_t pid_balance;  // 姿态（平衡）环
PID_t pid_speed;    // 速度环
PID_t pid_turn;     // 转向环

// 外部中断线10服务程序，PID控制在此函数中
void EXTI9_5_IRQHandler(void)
{
	if (MPU6050_ReadDMP(&pitch, &roll, &yaw) == 0)
	{
		//=== 姿态环 PID（输出目标速度） ===//
		pid_balance.Target = 0;
		pid_balance.Actual = roll;
		PID_Update(&pid_balance);

		float speed_target = pid_balance.Out;

		//=== 速度采集与滤波 ===//
		float SP_Left = Encoder_Get(1);
		float SP_Right = Encoder_Get(2);
		float Speed = (SP_Left + SP_Right) / 2.0f;

		float a = 0.3f;
		filt_Speed = a * Speed + (1 - a) * last_filt_Speed;
		last_filt_Speed = filt_Speed;

		Speed_sum += filt_Speed;
		if (Speed_sum > 3000) Speed_sum = 3000;
		if (Speed_sum < -3000) Speed_sum = -3000;

		//=== 速度环 PID ===//
		pid_speed.Target = speed_target;
		pid_speed.Actual = filt_Speed;
		pid_speed.ErrorInt = Speed_sum;
		PID_Update(&pid_speed);

		//=== 转向环 PID ===//
		pid_turn.Target = turn_target;  // 默认设定为 0，或者通过遥控器设定
		pid_turn.Actual = roll;
		PID_Update(&pid_turn);

		//=== 合成左右轮 PWM 输出 ===//
		float pwm_left  = pid_speed.Out;// + pid_turn.Out;
		float pwm_right = -pid_speed.Out;// - pid_turn.Out;

		//=== PWM 限幅（保险起见） ===//
		if (pwm_left > 2000) pwm_left = 2000;
		if (pwm_left < -2000) pwm_left = -2000;
		if (pwm_right > 2000) pwm_right = 2000;
		if (pwm_right < -2000) pwm_right = -2000;

		//=== 输出到电机 ===//
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

	// 清除中断标志位
	EXTI_ClearITPendingBit(EXTI_Line5);
}
//并行pid
//void EXTI9_5_IRQHandler(void)
//{
//	if(MPU6050_ReadDMP(&pitch,&roll,&yaw)==0)
//	{ 	
////位置环
//			float Kp= 480 ,Kd= 2800 ;
//			Target2 = 0;
//		  Target  = 0;
//			Actual =  roll;
//			Actual2 = -roll;			
//			Error1 = Error0;
//			Error0 = Target - Actual;												 //误差=目标值-实际值
//			Error12 = Error02;
//			Error02 = Target2 - Actual2;	
//		
//			Out = Kp * Error0 + Kd * (Error0 - Error1);
//			Out2 = Kp * Error02 + Kd * (Error02 - Error12);				
//			
////速度环
//			float vkp= 310 , vki= 1.55;
//			SP_Left= Encoder_Get(1); SP_Right=Encoder_Get(2);												//左右轮速度
//			Speed=(SP_Left+SP_Right)/2;											//平均速度
//			float a=0.3;																		//速度环滤波参数0.3
//			filt_Speed = a*Speed + (1-a)*last_filt_Speed;		//滤波
//			Speed_sum +=  filt_Speed;												//速度的累加
//			last_filt_Speed = filt_Speed;										//此次速度记录为上次速度
//			if(Speed_sum>3000){Speed_sum=3000;}							//积分限幅
//			if(Speed_sum<-3000){Speed_sum=-3000;}
//			
//			speed_target = 0 ;
//			speed_actual = filt_Speed ;
//			speed_error = speed_target - speed_actual ;
//			
//			speed_out  = vkp*speed_error - vki*Speed_sum;
//			speed_out2 = -vkp*speed_error + vki*Speed_sum;

////转向环
//			float turn_kp=80;
////			turn_target=0;	//转向环目标值设定为0
//			turn_actual=yaw;	// 转向环实际值=陀螺仪yaw角
//			turn_error=turn_target-turn_actual;//转向环误差
//			
//			turn_out=turn_kp*turn_error;
//			
////总输出及限幅
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
////未触发保护装置时，总输出值作为电机PWM
//			if(Motor_OFF==1)				
//			{Motor_SetPWM(1,0);Motor_SetPWM(2,0);}
//			else
//			{Motor_SetPWM(1,Total_out);	Motor_SetPWM(2,Total_out2);}
//	}
//	EXTI_ClearITPendingBit(EXTI_Line5); //清除中断标志位
//}


//#include "Control.h"
//#include "PID.h"
//#include "MPU6050.h"
//#include "Motor.h"
//#include "Encoder.h"
//#include "Sensor.h"

//// 全局控制状态变量
//ControlState_t ControlState;
//// 平衡控制PID结构体（从main.c移动过来）
//extern PID_t Inner;  // 内环（速度环）
//extern PID_t Outer;  // 外环（角度环）

//extern float Pitch,Roll,Yaw;

//// 循迹平衡控制的额外变量
//static float LineTrack_LeftSpeed = 0;
//static float LineTrack_RightSpeed = 0;
//static float Balance_Output = 0;

///**
//  * @brief  控制模块初始化
//  * @param  无
//  * @retval 无
//  */
//void Control_Init(void)
//{
//    // 初始化控制状态
//    ControlState.CurrentMode = CONTROL_MODE_STOP;
//    ControlState.LastMode = CONTROL_MODE_STOP;
//    ControlState.BalanceEnable = 0;
//    ControlState.LineTrackEnable = 0;
//    ControlState.BalanceTarget = 0.0f;
//    ControlState.LineTrackSpeed = 5.0f;
//    
//    // 重置PID输出
//    Inner.Out = 0;
//    Outer.Out = 0;
//    Balance_Output = 0;
//}

///**
//  * @brief  设置控制模式
//  * @param  mode 目标控制模式
//  * @retval 无
//  */
//void Control_SetMode(ControlMode_t mode)
//{
//    if (mode != ControlState.CurrentMode)
//    {
//        ControlState.LastMode = ControlState.CurrentMode;
//        ControlState.CurrentMode = mode;
//        
//        // 根据模式设置使能标志
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
//  * @brief  获取当前控制模式
//  * @param  无
//  * @retval 当前控制模式
//  */
//ControlMode_t Control_GetMode(void)
//{
//    return ControlState.CurrentMode;
//}

///**
//  * @brief  控制更新主函数（在定时器中断中调用）
//  * @param  无
//  * @retval 无
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
//            // 停止或手动模式不执行自动控制
//            break;
//    }
//}

///**
//  * @brief  纯平衡控制函数
//  * @param  无
//  * @retval 无
//  */
////float pitch,roll,yaw;	//欧拉角  
////float Speed, SP_Left, SP_Right, filt_Speed, last_filt_Speed, Speed_sum, speed_target, speed_actual, speed_error;//速度环参数
////float Target,Target2,Actual,Actual2,Error0,Error1,Error02,Error12,ErrorInt,ErrorInt2;//直立环参数
////float turn_target, turn_actual, turn_kp, turn_error;//转向环参数
////float Out,Out2,speed_out,speed_out2, turn_out, Total_out, Total_out2;//各环及总输出
////extern int Motor_OFF;//声明变量：电机关闭
//float SP_Left, SP_Right;
//void Balance_Control(void)
//{

//    if (!ControlState.BalanceEnable) return;
//    
//    static int16_t Speed, Location;
//    //float Pitch, Roll, Yaw;
//    
//    //读取MPU6050姿态数据
//    MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
//    
//    // 获取编码器速度
//	Speed = (Encoder_Get(1) + Encoder_Get(2)) / 2;
//    
//    // 外环（角度环）控制
//    Outer.Target = ControlState.BalanceTarget;  // 平衡目标角度
//    Outer.Actual = Roll;  // 使用俯仰角作为反馈
//    PID_Update(&Outer);
//    
//    // 内环（速度环）控制
//    Inner.Target = Outer.Out;  // 外环输出作为内环目标
//    Inner.Actual = Speed;
//    PID_Update(&Inner);
//    
//    // 输出到电机
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
////			Error0 = Target - Actual;												 //误差=目标值-实际值
////			Error12 = Error02;
////			Error02 = Target2 - Actual2;	
////		
////			Out = Kp * Error0 + Kd * (Error0 - Error1);
////			Out2 = Kp * Error02 + Kd * (Error02 - Error12);				
////			
//////速度环
////			float vkp= 310 , vki= 1.55;
////			SP_Left= Encoder_Get();SP_Right=	Encoder2_Get();												//左右轮速度
////			Speed=(SP_Left+SP_Right)/2;											//平均速度
////			float a=0.3;																		//速度环滤波参数0.3
////			filt_Speed = a*Speed + (1-a)*last_filt_Speed;		//滤波
////			Speed_sum +=  filt_Speed;												//速度的累加
////			last_filt_Speed = filt_Speed;										//此次速度记录为上次速度
////			if(Speed_sum>3000){Speed_sum=3000;}							//积分限幅
////			if(Speed_sum<-3000){Speed_sum=-3000;}
////			
////			speed_target = 0 ;
////			speed_actual = filt_Speed ;
////			speed_error = speed_target - speed_actual ;
////			
////			speed_out  = vkp*speed_error - vki*Speed_sum;
////			speed_out2 = -vkp*speed_error + vki*Speed_sum;

//////转向环
////			float turn_kp=80;
//////			turn_target=0;	//转向环目标值设定为0
////			turn_actual=yaw;	// 转向环实际值=陀螺仪yaw角
////			turn_error=turn_target-turn_actual;//转向环误差
////			
////			turn_out=turn_kp*turn_error;
////			
//////总输出及限幅
////			Total_out	= Out + speed_out + turn_out;
////			Total_out2	= Out2 + speed_out2 + turn_out;		
////			if (Total_out > 7200) {Total_out = 7200;}
////			if (Total_out < -7200) {Total_out = -7200;}
////			if (Total_out2 > 7200) {Total_out2 = 7200;}
////			if (Total_out2 < -7200) {Total_out2 = -7200;}	
////			
//////未触发保护装置时，总输出值作为电机PWM
////			if(Motor_OFF==1)
////			{Motor_SetPWM(1,0);Motor_SetPWM(2,0);}
////			else
////			{Motor_SetPWM(1,Total_out);Motor_SetPWM(2,Total_out2);}
//}

///**
//  * @brief  循迹平衡控制函数
//  * @param  无
//  * @retval 无
//  */
//void LineTrack_Balance_Control(void)
//{
//    if (!ControlState.BalanceEnable || !ControlState.LineTrackEnable) return;
//    
//    static int16_t Speed, Location;
//    //float Pitch, Roll, Yaw;
//    uint8_t area;
//    
//    // 读取MPU6050姿态数据
//    MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
//    
//    // 获取编码器速度
//    Speed = Encoder_Get(2);
//    Location += Speed;
//    
//    // 更新传感器状态并获取区域信息
//    Sensor_UpdateAll();
//    area = Sensor_GetArea();
//    
//    // 根据循迹状态计算左右轮目标速度
//    switch (area)
//    {
//        case 3:  // 直线
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed;
//            break;
//            
//        case 4:  // 右小弯
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 1.2f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 0.6f;
//            break;
//            
//        case 6:  // 右大弯
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 1.5f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 0.3f;
//            break;
//            
//        case 2:  // 左小弯
//            LineTrack_LeftSpeed = ControlState.LineTrackSpeed * 0.6f;
//            LineTrack_RightSpeed = ControlState.LineTrackSpeed * 1.2f;
//            break;
//            
//        case 0:  // 左大弯
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
//    // 平衡控制（角度环）
//    Outer.Target = ControlState.BalanceTarget;
//    Outer.Actual = Roll;
//    PID_Update(&Outer);
//    
//    Balance_Output = Outer.Out;
//    
//    // 将平衡输出和循迹控制结合
//    float LeftOutput = Balance_Output + LineTrack_LeftSpeed;
//    float RightOutput = Balance_Output + LineTrack_RightSpeed;
//    
//    // 输出限幅
//    if (LeftOutput > 100) LeftOutput = 100;
//    if (LeftOutput < -100) LeftOutput = -100;
//    if (RightOutput > 100) RightOutput = 100;
//    if (RightOutput < -100) RightOutput = -100;
//    
//    // 输出到电机
//    Motor_SetPWM(1, (int16_t)LeftOutput);   // 左电机
//    Motor_SetPWM(2, (int16_t)RightOutput);  // 右电机
//}

///**
//  * @brief  使能/禁用平衡控制
//  * @param  enable 1-使能，0-禁用
//  * @retval 无
//  */
//void Balance_Enable(uint8_t enable)
//{
//    ControlState.BalanceEnable = enable;
//    if (!enable)
//    {
//        // 禁用时重置PID
//        Inner.Out = 0;
//        Outer.Out = 0;
//        Inner.ErrorInt = 0;
//        Outer.ErrorInt = 0;
//    }
//}

///**
//  * @brief  设置平衡目标角度
//  * @param  target 目标角度
//  * @retval 无
//  */
//void Balance_SetTarget(float target)
//{
//    ControlState.BalanceTarget = target;
//}

///**
//  * @brief  使能/禁用循迹控制
//  * @param  enable 1-使能，0-禁用
//  * @retval 无
//  */
//void LineTrack_Enable(uint8_t enable)
//{
//    ControlState.LineTrackEnable = enable;
//}

///**
//  * @brief  设置循迹基础速度
//  * @param  speed 基础速度
//  * @retval 无
//  */
//void LineTrack_SetSpeed(float speed)
//{
//    ControlState.LineTrackSpeed = speed;
//}
