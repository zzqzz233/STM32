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
//IIC起始信号
void IIC_Start(void)
{
  SDA_OUT();//设置SDA信号线为输出模式
  SDA = 1;
  SCL = 1;
  delay_us(2);
  SDA = 0;//将SDA和SCL同时拉高然后将SDA拉低作为开始信号
  delay_us(2);
  SCL = 0;//将SCL拉低，准备开始接收或发送信号
}

//IIC结束信号
void IIC_Stop(void)
{ 
  SDA_OUT();//设置SDA信号线为输出模式
  SDA = 0;
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SDA = 1;//在SCL高电平时将SDA从低拉高作为结束信号
  delay_us(2);
}

//IIC写一个字节的数据
void IIC_WriteByte(uint8_t Value)
{
  uint8_t i;
  
  SDA_OUT();//设置SDA信号线为输出
  SCL = 0;//拉低时钟线，准备写入数据
  
  for(i = 0;i < 8;i++)
  {
    if((Value&0x80) != 0)//IIC协议先发送高位
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
    Value = Value << 1;//发送的数据左移一位  
  }
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_WaitAck(void)
{
  uint8_t timeout = 0;
  
  SDA_IN();
  SDA = 1;//拉高SDA
  SCL = 0;
  delay_us(1);
  SCL = 1;
  delay_us(1);
  while(Read_SDA == 1)//等到应答信号
  {
    timeout++;
    if(timeout > 250)//未收到应答信号，返回非应答信号
    {
      IIC_Stop();     
      return NAck;
    }
  }
  SCL = 0;
  return Ack;
}

//产生应答信号
void IIC_Ack(void)
{
  SDA_OUT();//SDA为输出
  SDA = 0;//SDA为低电平时为应答信号
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SCL = 0;
  delay_us(2);  
}

//产生非应答信号
void IIC_NAck(void)
{
  SDA_OUT();//SDA为输出
  SDA = 1;//SDA为高电平时为非应答信号
  SCL = 0;
  delay_us(2);
  SCL = 1;
  delay_us(2);
  SCL = 0;
  delay_us(2); 
}

//读取一个字节的数据，ack为1产生非应答信号，为0产生应答信号
uint8_t IIC_ReadByte(uint8_t ack)
{
  uint8_t i;
  uint8_t receive;
  
  SDA_IN();//设置SDA信号线为输入
  
  for(i = 0;i < 8;i++)
  {
    SCL = 0;
    delay_us(2);
    SCL = 1;
    receive = receive << 1;//数据向高位移动一个
    if(Read_SDA == 1)//如果为1就将数据加一
    {
      receive++;
    }
    delay_us(1);
    SCL = 0;
  }  
  if(ack == 1)//产生非应答信号
  {
    IIC_NAck();//发送NACK
  }
  if(ack == 0)//产生应答信号
  {
    IIC_Ack();
  } 
  return receive;
}
  


