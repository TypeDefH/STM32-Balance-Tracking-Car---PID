#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

#define	MPU6050_SMPLRT_DIV		    0x19    //�����ʷ�Ƶ�Ĵ���
#define	MPU6050_CONFIG			    0x1A    //���üĴ���
#define	MPU6050_GYRO_CONFIG		    0x1B    //���������üĴ���
#define	MPU6050_ACCEL_CONFIG        0x1C    //���ٶȼ����üĴ���

#define	MPU6050_ACCEL_XOUT_H	    0x3B    //���ٶȼ�X���λ�Ĵ���
#define	MPU6050_ACCEL_XOUT_L	    0x3C    //���ٶȼ�X���λ�Ĵ���
#define	MPU6050_ACCEL_YOUT_H	    0x3D    //���ٶȼ�Y���λ�Ĵ���
#define	MPU6050_ACCEL_YOUT_L	    0x3E    //���ٶȼ�Y���λ�Ĵ���
#define	MPU6050_ACCEL_ZOUT_H	    0x3F    //���ٶȼ�Z���λ�Ĵ���
#define	MPU6050_ACCEL_ZOUT_L	    0x40    //���ٶȼ�Z���λ�Ĵ���
#define	MPU6050_TEMP_OUT_H		    0x41    //�¶ȸ�λ�Ĵ���             
#define	MPU6050_TEMP_OUT_L		    0x42    //�¶ȵ�λ�Ĵ���
#define	MPU6050_GYRO_XOUT_H		    0x43    //������X���λ�Ĵ���
#define	MPU6050_GYRO_XOUT_L		    0x44    //������X���λ�Ĵ���
#define	MPU6050_GYRO_YOUT_H		    0x45    //������Y���λ�Ĵ���
#define	MPU6050_GYRO_YOUT_L		    0x46    //������Y���λ�Ĵ���
#define	MPU6050_GYRO_ZOUT_H		    0x47    //������Z���λ�Ĵ���
#define	MPU6050_GYRO_ZOUT_L		    0x48    //������Z���λ�Ĵ���
                                    
#define	MPU6050_PWR_MGMT_1		    0x6B	//��Դ����Ĵ���1
#define	MPU6050_PWR_MGMT_2		    0x6C	//��Դ����Ĵ���2
#define	MPU6050_WHO_AM_I		    0x75	//ID�Ĵ�����Ĭ����ֵ0x68��ֻ����

#endif
