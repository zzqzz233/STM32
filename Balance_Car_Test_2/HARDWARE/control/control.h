#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
/**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
 ΢�Ź��ںš���������ˡ�
 ��̨�ظ���ƽ��С��������ȡƽ��С��ȫ��DIY����
 ��̨�ظ������ӿ������ߡ�����ȡ���ӹ���ʦ�ر���������
 ��̨�ظ�������������ϡ�����ȡ����������ϰ�
 ֪���������� 
**************************************************************************/
#define PI 3.14159265
extern int Buletooth_Movement;
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
#endif