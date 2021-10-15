#ifndef  __IIC_H
#define  __IIC_H

#include "main.h"

//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)8<<28;}	//PC12输入模式
#define SDA_OUT() {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)3<<28;} 	//PC12输出模式
/////////////////////////////////////////////////////////////////////////////////
#define SCL PBout(14)//SCL信号线
#define SDA PBout(15)//SDA信号线
#define Read_SDA PBin(15)//读取SDA信号线上的电平
#define Ack 0  //Ack应答信号
#define NAck 1 //NAck非应答信号线


void IIC_Init(void);//初始化IIC输出的引脚
void IIC_Start(void);//开启IIC指令
void IIC_Stop(void);//关闭IIC指令
void IIC_WriteByte(uint8_t Value);//写IIC一个字节
uint8_t IIC_ReadByte(unsigned char Data);//读IIC一个字节
uint8_t IIC_WaitAck(void);//等待IIC应答指令
void IIC_Ack(void);//发送IIC应答指令
void IIC_NAck(void);//不发送IIC应答指令

#endif

