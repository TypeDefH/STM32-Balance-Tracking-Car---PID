#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "Serial.h"
#include "PID.h"
#include "Key.h"
#include "RP.h"
/*
tb6612  / pwm pa8 pa11 /motor pb12 13   pb14 15
oled    / pb8 pb9
mpu6050 / scl pb10  sda pb11  int pa5
key     / pa12
RP      / pa2 3 4 5
uart    / RXpa10 TXpa9
sensor  / pb0 1 4 5  |vcc pb0 pb1 pb4 pb5 gnd|
encoder / pa6 7   pb6 7

*/
//声明外部变量
uint8_t Motor_OFF;
extern float pitch,roll,yaw;
extern float Out,Out2;
extern float Speed,SP_Left,SP_Right;
extern float pwm_left,pwm_right ;
extern float turn_target;
extern float Speed_sum,filt_Speed,last_filt_Speed;
	
void PID_Init()
{
    // 平衡环 PID
    pid_balance.Kp = 480;
    pid_balance.Ki = 0;
    pid_balance.Kd = 2800;
    pid_balance.OutMax = 1000;
    pid_balance.OutMin = -1000;

    // 速度环 PID
    pid_speed.Kp = 310;
    pid_speed.Ki = 1.55;
    pid_speed.Kd = 0;
    pid_speed.OutMax = 2000;
    pid_speed.OutMin = -2000;

    // 转向环 PID
    pid_turn.Kp = 80;
    pid_turn.Ki = 0;
    pid_turn.Kd = 0;
    pid_turn.OutMax = 1000;
    pid_turn.OutMin = -1000;

    Speed_sum = 0;
    filt_Speed = last_filt_Speed = 0;
}


int main(void)
{
	
//初始化函数
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	OLED_Init();
	Motor_Init();
	Encoder_Init();
//	RP_Init();
	Serial_Init();
	MPU6050_DMPInit();
	MPU_exti_init();
	PID_Init();

	while(1)
	{	
//      Kp = RP_GetValue(1) / 4095.0 * 100;
//      Ki = RP_GetValue(2) / 4095.0 * 10;
//		Kd = RP_GetValue(3) / 4095.0 * 10;
//俯仰角过大时，关闭电机
		if(roll<-25 || roll>25)
		{Motor_OFF=1;}
		else
		{Motor_OFF=0;}	

//串口接收数据包
//		if (Serial_GetRxFlag() == 1)
//		{
//				turn_target=Serial_RxPacket[0];
//		}
//OLED显示欧拉角	
		OLED_Printf(0, 16, OLED_6X8,"P:%+03.1f", pitch);
		OLED_Printf(0, 24, OLED_6X8,"R:%+03.1f", roll);
		OLED_Printf(0, 32, OLED_6X8,"Y:%+03.1f", yaw);
		OLED_Printf(0, 40, OLED_6X8,"ON:%d", Motor_OFF);
		OLED_Printf(0, 48, OLED_6X8,"out:%05.1f", pwm_left);
		OLED_Printf(0, 56, OLED_6X8,"out:%05.1f", pwm_right);
		OLED_Printf(72, 48, OLED_6X8,"Sl:%05.1f", SP_Left);
		OLED_Printf(72, 56, OLED_6X8,"Sr:%05.1f", SP_Right);
		OLED_Update();
		
		Serial_Printf("%f,%f,%f,%f,%f\r\n", roll, SP_Left, SP_Right, pwm_left, pwm_right);		
	}
}
//#include "stm32f10x.h"
//#include "Delay.h"
//#include "OLED.h"
//#include "LED.h"
//#include "Timer.h"
//#include "Key.h"
//#include "RP.h"
//#include "Motor.h"
//#include "Encoder.h"
//#include "Serial.h"
//#include "MPU6050.h"
//#include "PID.h"
//#include "Control.h"
//#include "Sensor.h"


//// PID结构体变量（供Control模块使用）
//PID_t Inner = {
//    .Kp = 0.3,
//    .Ki = 0.3,
//    .Kd = 0,
//    .OutMax = 40,
//    .OutMin = -40,
//};

//PID_t Outer = {
//    .Kp = 0.3,
//    .Ki = 0,
//    .Kd = 0.4,
//    .OutMax = 40,
//    .OutMin = -40,
//};

//uint8_t state;
//uint8_t KeyNum;
//int16_t Speed, Location;
//float Pitch,Roll,Yaw;

//int main(void)
//{
//	/*模块初始化*/
//	OLED_Init();		//OLED初始化
//	Key_Init();			//非阻塞式按键初始化
//	Serial_Init();
//	MPU6050_DMPInit();
//	Timer_Init();		//定时器初始化，1ms定时中断一次
//	
//    Motor_Init();
//    Encoder_Init();
//    RP_Init();

//	Sensor_Init();	
//    Control_Init();

//    Control_SetMode(CONTROL_MODE_PURE_LINE); 
//	
//	while (1)
//	{

////		Inner.Kp = RP_GetValue(1) / 4095.0 * 2;
////        Inner.Ki = RP_GetValue(2) / 4095.0 * 2;
////        Inner.Kd = RP_GetValue(3) / 4095.0 * 2;
////		
////        Outer.Kp = RP_GetValue(1) / 4095.0 * 8;
////        Outer.Ki = RP_GetValue(2) / 4095.0 * 8;
////        Outer.Kd = RP_GetValue(3) / 4095.0 * 8;
//		Outer.Target=0;
////        Balance_SetTarget(RP_GetValue(4) / 4095.0 * 20 - 10);  // -10到+10度
////		MPU6050_ReadDMP(&Pitch, &Roll, &Yaw);
//		KeyNum = Key_GetNum();		//获取键码
//		if (KeyNum == 1)			//如果K1按下
//		{
//			state ++;					//变量j加1
//			if(state==4){ state=0 ;}
//		}

//		switch(state)
//		{
//			case 0:Control_SetMode(CONTROL_MODE_PURE_LINE);break;     // 纯循迹模式（不带平衡）
//			case 1:Control_SetMode(CONTROL_MODE_BALANCE);break;       // 纯平衡模式
//			case 2:Control_SetMode(CONTROL_MODE_LINE_TRACK);break;    // 循迹平衡模式
//			case 3:Control_SetMode(CONTROL_MODE_STOP);break;          // 停止模式
//			default:         break;	                                  
//		}
//	
//		OLED_Printf(0, 16, OLED_6X8, "Mode:%d", state);	
//		OLED_Printf(0, 24, OLED_6X8, "Kp:%4.2f", Outer.Kp);
//        OLED_Printf(0, 32, OLED_6X8, "Ki:%4.2f", Outer.Ki);
//        OLED_Printf(0, 40, OLED_6X8, "Kd:%4.2f", Outer.Kd);
//        OLED_Printf(0, 48, OLED_6X8, "Tar:%+04.1f", ControlState.BalanceTarget);
//        OLED_Printf(0, 56, OLED_6X8, "Out:%+04.0f", Inner.Out);
//		OLED_Printf(64, 16, OLED_6X8, "P:%+06.1f", Pitch);
//		OLED_Printf(64, 24, OLED_6X8, "R:%+06.1f", Roll);
//		OLED_Printf(64, 32, OLED_6X8, "Y:%+06.1f", Yaw);
//		
//		/*OLED更新*/
//		OLED_Update();
//		
//		Serial_Printf("%d,%f,%f,%f\r\n", Control_GetMode(), ControlState.BalanceTarget, Outer.Actual, Outer.Out);
//		

//	}
//}

//void TIM2_IRQHandler(void)
//{
//    static uint16_t Count1,Count2;
//    
//    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
//    {
//        Key_Tick();
//        
//        Count1++;
//		if (Count1 >= 40)		//如果计次40次，则if成立，即if每隔40ms进一次
//		{
//			Count1 = 0;			//计次清零，便于下次计次
//			Control_Update();
//			
//		}
//        
//        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//    }
//}
