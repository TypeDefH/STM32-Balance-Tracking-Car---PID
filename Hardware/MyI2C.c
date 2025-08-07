#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/*��������*/
#define MyI2C_SCL        GPIO_Pin_10    //PB10
#define MyI2C_SDA        GPIO_Pin_11    //PB11

/**
  * @brief  I2CдSCL����
  * @param  BitValue Ҫд���ֵ����Χ��0��1
  * @retval ��
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, MyI2C_SCL, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * @brief  I2CдSDA����
  * @param  BitValue Ҫд���ֵ����Χ��0��1
  * @retval ��
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, MyI2C_SDA, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * @brief  I2C��SDA����
  * @param  ��
  * @retval ���ض�ȡ��ֵ����Χ��0��1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;

	BitValue = GPIO_ReadInputDataBit(GPIOB, MyI2C_SDA);
	Delay_us(10);
	return BitValue;
}

/**
  * @brief  I2C��ʼ����������ʼ��PB10ΪI2C_SCL�����PB11ΪI2C_SDA���
  * @param  ��
  * @retval ��
  */
void MyI2C_Init(void)
{
	//����RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//����GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;   //��©���ģʽ
	GPIO_InitStructure.GPIO_Pin = MyI2C_SCL | MyI2C_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, MyI2C_SCL | MyI2C_SDA);
}

/**
  * @brief  I2C��ʼ������SCL�ߵ�ƽ�ڼ䣬SDA�ɸߵ�ƽ�л����͵�ƽ
  * @param  ��
  * @retval ��
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

/**
  * @brief  I2C��ֹ������SCL�ߵ�ƽ�ڼ䣬SDA�ɵ͵�ƽ�л����ߵ�ƽ
  * @param  ��
  * @retval ��
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

/**
  * @brief  I2C����һ���ֽں�����SCL�͵�ƽ�ڼ䣬����������λ���ηŵ�SDA�ϣ���λ��ǰ��������SCL��ѭ��8�Σ�
  *         �ӻ���SCL�ߵ�ƽ�ڼ��ȡ����λ
  * @param  Byte Ҫ���͵��ֽ�
  * @retval ��
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

/**
  * @brief  I2C����һ���ֽں�����SCL�͵�ƽ�ڼ䣬�ӻ�������λ���ηŵ�SDA�ϣ���λ��ǰ��������SCL��ѭ��8�Σ�
  *         ������SCL�ߵ�ƽ�ڼ��ȡ����λ�������ڽ���֮ǰ����Ҫ�ͷ�SDA��
  * @param  ��
  * @retval ���ؽ��յ��ֽ�
  */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;

	MyI2C_W_SDA(1);
	for(i = 0; i < 8; i++)
	{
		MyI2C_W_SCL(1);
		if(MyI2C_R_SDA() == 1)  {Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

/**
  * @brief  I2C����Ӧ�������ڽ�����һ���ֽ�֮����������һ��ʱ�ӷ���Ӧ��
  * @param  AckBit Ҫ���͵�Ӧ��λ��0��ʾӦ��1��ʾ��Ӧ��
  * @retval 
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

/**
  * @brief  I2C����Ӧ�������ڷ�����һ���ֽ�֮����������һ��ʱ�ӽ���һλ���ݣ��жϴӻ��Ƿ�Ӧ��
  *         �������ڽ���֮ǰ����Ҫ�ͷ�SDA��
  * @param  ��
  * @retval ���ؽ��յ�Ӧ��λ��0��ʾӦ��1��ʾ��Ӧ��
  */
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}
