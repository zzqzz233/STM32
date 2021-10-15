#include "control.h"
/**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
 ΢�Ź��ںš���������ˡ�
 ��̨�ظ���ƽ��С��������ȡƽ��С��ȫ��DIY����
 ��̨�ظ������ӿ������ߡ�����ȡ���ӹ���ʦ�ر���������
 ��̨�ظ�������������ϡ�����ȡ����������ϰ�
 ֪���������� 
**************************************************************************/
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
				 ��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
				 ���߿���imv_mpu.h�ļ���26�еĺ궨������޸�(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
//int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

//float Mechanical_angle = -5.6; 
int Buletooth_Movement = 0;

float balance_UP_KP = -500; 	 // С��ֱ����PD����  501  -300
float balance_UP_KD = -1.2;           // 1.2     0.7

float velocity_KP = -160;     // С���ٶȻ�PI����     -130
float velocity_KI = -0.8;                             //-0.65
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
��    �ߣ��������
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias = Angle - Mechanical_balance;    							 //===���ƽ��ĽǶ���ֵ�ͻ�е���
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
��    �ߣ��������
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;

   //=============�ٶ�PI������=======================//	
		Encoder_Least = (Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral += Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral = Encoder_Integral - Buletooth_Movement;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  
    {
      Encoder_Integral=10000;             //===�����޷�
    }
		if(Encoder_Integral<-10000)	
    {
      Encoder_Integral=-10000;            //===�����޷�	
    }
		Velocity = Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===�ٶȿ���	
	  if(pitch<-40||pitch>40) 	
    {
      Encoder_Integral=0;     						//===����رպ��������
    }
	  return Velocity;
}

/**************************************************************************
�������ܣ�ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
//int turn(int encoder_left,int encoder_right,float gyro)//ת�����
//{
//	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert = 0.9,Turn_Count;
//	  float Turn_Amplitude=50,Kp=150,Kd=0;     
//	  //=============ң��������ת����=======================//
//  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
//		{
//			if(++Turn_Count == 1)
//      {
//        Encoder_temp = myabs(encoder_left+encoder_right);
//      }
//			Turn_Convert = 50/Encoder_temp;
//			if(Turn_Convert<0.6)
//      {
//        Turn_Convert = 0.6;
//      }
//			if(Turn_Convert>3)
//      {
//        Turn_Convert = 3;
//      }
//		}	
//	  else
//		{
//			Turn_Convert=0.9;
//			Turn_Count=0;
//			Encoder_temp=0;
//		}			
//		if(1==Flag_Left)	       
//    {
//      Turn_Target -= Turn_Convert;
//    }
//		else if(1==Flag_Right)	 
//    {
//      Turn_Target += Turn_Convert; 
//    }
//		else 
//    {
//      Turn_Target = 0;
//    }
//    if(Turn_Target>Turn_Amplitude) 
//    {
//      Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
//    }
//	  if(Turn_Target<-Turn_Amplitude)
//    {
//      Turn_Target=-Turn_Amplitude;
//    }
//		if(Flag_Qian==1||Flag_Hou==1) 
//    {
//      Kd=0.5;        
//    }
//		else
//    {
//      Kd = 0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
//    }
//  	//=============ת��PD������=======================//
//		Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
//	  return Turn;
//}
/**************************************************************************
�������ܣ�ת��PD����
��ڲ����������������ֵ��Z����ٶ�
����  ֵ��ת�����PWM
��    �ߣ��������
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=100,Kp=10,Kd=0;     
	  //=============ң��������ת����=======================//
	  //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
  	if(1==Flag_Left||1==Flag_Right)                      
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);      
			Turn_Convert=55/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert;
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת��	�ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	  return Turn;
}
