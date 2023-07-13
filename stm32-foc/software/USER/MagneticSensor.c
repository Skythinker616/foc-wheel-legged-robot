/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
本项目删除了部分代码，将IIC改为了软件模拟IIC
************************************************/

#include "MagneticSensor.h" 
#include "foc_utils.h"
#include "gpio.h"

/******************************************************************************/
long cpr;
float full_rotation_offset;
long angle_data, angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

#define IIC_SCL_GPIO_PORT               GPIOB
#define IIC_SCL_GPIO_PIN                GPIO_PIN_6
#define IIC_SDA_GPIO_PORT               GPIOB
#define IIC_SDA_GPIO_PIN                GPIO_PIN_7

#define IIC_SCL(x)        do{ x ? \
							  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
							  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
						  }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
							  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
							  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
						  }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* ????SDA */

//2us
static void iic_delay(void)
{
	for(uint32_t j=0;j<10;j++);
}

void iic_start(void)
{
	IIC_SDA(1);
	IIC_SCL(1);
	iic_delay();
	IIC_SDA(0);
	iic_delay();
	IIC_SCL(0);
	iic_delay();
}

void iic_stop(void)
{
	IIC_SDA(0);     
	iic_delay();
	IIC_SCL(1);
	iic_delay();
	IIC_SDA(1);    
	iic_delay();
}

void iic_ack(void)
{
	IIC_SDA(0);     
	iic_delay();
	IIC_SCL(1);     
	iic_delay();
	IIC_SCL(0);
	iic_delay();
	IIC_SDA(1);     
	iic_delay();
}

void iic_nack(void)
{
	IIC_SDA(1);     
	iic_delay();
	IIC_SCL(1);     
	iic_delay();
	IIC_SCL(0);
	iic_delay();
}

uint8_t iic_wait_ack(void)
{
	uint8_t waittime = 0;
	uint8_t rack = 0;

	IIC_SDA(1);    
	iic_delay();
	IIC_SCL(1);    
	iic_delay();

	while (IIC_READ_SDA)   
	{
		waittime++;

		if (waittime > 250)
		{
			iic_stop();
			rack = 1;
			break;
		}
	}

	IIC_SCL(0);    
	iic_delay();
	return rack;
}

uint8_t iic_read_byte(uint8_t ack)
{
	uint8_t i, receive = 0;

	for (i = 0; i < 8; i++ )    
	{
		receive <<= 1;  
		IIC_SCL(1);
		iic_delay();

		if (IIC_READ_SDA)
		{
			receive++;
		}
		
		IIC_SCL(0);
		iic_delay();
	}

	if (!ack)
	{
		iic_nack();     
	}
	else
	{
		iic_ack();     
	}

	return receive;
}

void iic_send_byte(uint8_t data)
{
	uint8_t t;
	
	for (t = 0; t < 8; t++)
	{
		IIC_SDA((data & 0x80) >> 7);    
		iic_delay();
		IIC_SCL(1);
		iic_delay();
		IIC_SCL(0);
		data <<= 1;     
	}
	IIC_SDA(1);       
}

uint16_t AS5600_ReadTwoByte(uint16_t readAddr)
{
	uint16_t temp=0xFFFF;                                                                                   
	iic_start();  
	iic_send_byte((0X36<<1)|0x00);    
	iic_wait_ack(); 
	iic_send_byte(readAddr);   
	iic_wait_ack();        
	iic_start();              
	iic_send_byte((0X36<<1)|0x01);              
	iic_wait_ack();     
	temp=iic_read_byte(1);   
	temp=temp<<8|iic_read_byte(0); 
	iic_stop();     
	return temp;
}

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C
#define  AS5600_CPR      4096

unsigned short I2C_getRawCount()
{
	return AS5600_ReadTwoByte(0x0C);
}

/******************************************************************************/
void MagneticSensor_Init(void)
{
	cpr=AS5600_CPR;
	angle_data = angle_data_prev = I2C_getRawCount();  
	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(void)
{
	float d_angle;
	
	angle_data = I2C_getRawCount();
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (full_rotation_offset + ( angle_data * 1.0 / cpr) * _2PI) ;
}
/******************************************************************************/



