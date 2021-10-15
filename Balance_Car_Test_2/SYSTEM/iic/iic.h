#ifndef  __IIC_H
#define  __IIC_H

#include "main.h"

//IO��������
#define SDA_IN()  {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)8<<28;}	//PC12����ģʽ
#define SDA_OUT() {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)3<<28;} 	//PC12���ģʽ
/////////////////////////////////////////////////////////////////////////////////
#define SCL PBout(14)//SCL�ź���
#define SDA PBout(15)//SDA�ź���
#define Read_SDA PBin(15)//��ȡSDA�ź����ϵĵ�ƽ
#define Ack 0  //AckӦ���ź�
#define NAck 1 //NAck��Ӧ���ź���


void IIC_Init(void);//��ʼ��IIC���������
void IIC_Start(void);//����IICָ��
void IIC_Stop(void);//�ر�IICָ��
void IIC_WriteByte(uint8_t Value);//дIICһ���ֽ�
uint8_t IIC_ReadByte(unsigned char Data);//��IICһ���ֽ�
uint8_t IIC_WaitAck(void);//�ȴ�IICӦ��ָ��
void IIC_Ack(void);//����IICӦ��ָ��
void IIC_NAck(void);//������IICӦ��ָ��

#endif

