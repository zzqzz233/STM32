#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"	

/**************************************************************************
 作  者 ：大鱼电子
 淘宝地址：https://shop119207236.taobao.com
 微信公众号【大鱼机器人】
 后台回复【平衡小车】：获取平衡小车全套DIY资料
 后台回复【电子开发工具】：获取电子工程师必备开发工具
 后台回复【电子设计资料】：获取电子设计资料包
 知乎：张巧龙 
**************************************************************************/

#define PWMA   TIM1->CCR1  //PA8

#define AIN2   PAout(3)
#define AIN1   PAout(2)
#define BIN1   PAout(5)
#define BIN2   PAout(4)

#define PWMB   TIM1->CCR4  //PA11

void Motor_Init(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
void Xianfu_Pwm(void);
void Turn_Off(float angle,int accz);
#endif
