#include "iic.h"

void IIC_Init(void)
{ 
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
  
  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
}
//IIC��ʼ�ź�
void IIC_Start(void)
{
  SDA_OUT();//����SDA�ź���Ϊ���ģʽ
  SDA = 1;
  SCL = 1;
  delay_us(2);
  SDA = 0;//��SDA��SCLͬʱ����Ȼ��SDA������Ϊ��ʼ�ź�
  delay_us(2);
  SCL = 0;//��SCL���ͣ�׼����ʼ���ջ����ź�
}

//IIC�����ź�
void IIC_Stop(void)
{ 
  SDA_OUT();//����SDA�ź���Ϊ���ģʽ
  SDA = 0;
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SDA = 1;//��SCL�ߵ�ƽʱ��SDA�ӵ�������Ϊ�����ź�
  delay_us(2);
}

//IICдһ���ֽڵ�����
void IIC_WriteByte(uint8_t Value)
{
  uint8_t i;
  
  SDA_OUT();//����SDA�ź���Ϊ���
  SCL = 0;//����ʱ���ߣ�׼��д������
  
  for(i = 0;i < 8;i++)
  {
    if((Value&0x80) != 0)//IICЭ���ȷ��͸�λ
    {
      SDA = 1;
    }
    else
    {
      SDA = 0;
    }
    delay_us(2);
    SCL = 1;
    delay_us(2);
    SCL = 0;
    delay_us(2);
    SDA = 1;
    Value = Value << 1;//���͵���������һλ  
  }
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_WaitAck(void)
{
  uint8_t timeout = 0;
  
  SDA_IN();
  SDA = 1;//����SDA
  SCL = 0;
  delay_us(1);
  SCL = 1;
  delay_us(1);
  while(Read_SDA == 1)//�ȵ�Ӧ���ź�
  {
    timeout++;
    if(timeout > 250)//δ�յ�Ӧ���źţ����ط�Ӧ���ź�
    {
      IIC_Stop();     
      return NAck;
    }
  }
  SCL = 0;
  return Ack;
}

//����Ӧ���ź�
void IIC_Ack(void)
{
  SDA_OUT();//SDAΪ���
  SDA = 0;//SDAΪ�͵�ƽʱΪӦ���ź�
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SCL = 0;
  delay_us(2);  
}

//������Ӧ���ź�
void IIC_NAck(void)
{
  SDA_OUT();//SDAΪ���
  SDA = 1;//SDAΪ�ߵ�ƽʱΪ��Ӧ���ź�
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SCL = 0;
  delay_us(2); 
}

//��ȡһ���ֽڵ����ݣ�ackΪ1������Ӧ���źţ�Ϊ0����Ӧ���ź�
uint8_t IIC_ReadByte(uint8_t ack)
{
  uint8_t i;
  uint8_t receive;
  
  SDA_IN();//����SDA�ź���Ϊ����
  
  for(i = 0;i < 8;i++)
  {
    SCL = 0;
    delay_us(2);
    SCL = 1;
    receive = receive << 1;//�������λ�ƶ�һ��
    if(Read_SDA == 1)//���Ϊ1�ͽ����ݼ�һ
    {
      receive++;
    }
    delay_us(1);
    SCL = 0;
  }  
  if(ack == 1)//������Ӧ���ź�
  {
    IIC_NAck();//����NACK
  }
  if(ack == 0)//����Ӧ���ź�
  {
    IIC_Ack();
  } 
  return receive;
}
  


