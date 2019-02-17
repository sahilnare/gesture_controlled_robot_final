#define F_CPU 8000000UL									/* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>										/* Include AVR std. library file */
#include <util/delay.h>									/* Include delay header file */
#include <inttypes.h>									/* Include integer type header file */
#include <stdlib.h>										/* Include standard library file */
#include <stdio.h>										/* Include standard library file */
#include <math.h>								/* Include math function */

#define SCL_CLK 100000L							/* Define SCL clock frequency */
#define BITRATE(TWSR)	((F_CPU/SCL_CLK)-16)/(2*pow(4,(TWSR&((1<<TWPS0)|(1<<TWPS1))))) /* Define bit rate */
#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)	/* Define prescale value */
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define INT_ENABLE 0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B


void USART_Init(unsigned long);				/* USART initialize function */
char USART_RxChar();						/* Data receiving function */
void USART_TxChar(char);					/* Data transmitting function */
void USART_SendString(char*);				/* Send string of USART data function */
void intarray(int num);

void I2C_Init();								/* I2C initialize function */
uint8_t  I2C_Repeated_Start(char);				/* I2C repeated start function */
void I2C_Stop();								/* I2C stop function */
void I2C_Start_Wait(char);						/* I2C start wait function */
uint8_t  I2C_Write(char);						/* I2C write function */
char I2C_Read_Ack();							/* I2C read ack function */
char I2C_Read_Nack();							/* I2C read nack function */


float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;


void MPU6050_Init()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

void I2C_Init()												/* I2C initialize function */
{
	TWBR = 0x00;    //Clear Register
	TWSR = 0x00;    //Clear Register
	TWSR &= ~(1<<TWPS0);  //Set the prescaler bits to 0
	TWSR &= ~(1<<TWPS1);  //Set the prescaler bits to 0
	TWCR &= ~(1<<TWIE);   //Set acknowledge bit to 0
	TWBR = 32;
	//TWBR = BITRATE(TWSR = 0x00);							/* Get bit rate register value by formula */
}



uint8_t I2C_Repeated_Start(char slave_read_address)			/* I2C repeated start function */
{
	uint8_t status;											/* Declare variable */
	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);					/* Enable TWI, generate start condition and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (start condition) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status != 0x10)										/* Check weather repeated start condition transmitted successfully or not? */
	return 0;												/* If no then return 0 to indicate repeated start condition fail */
	TWDR = slave_read_address;								/* If yes then write SLA+R in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x40)										/* Check weather SLA+R transmitted & ack received or not? */
	return 1;												/* If yes then return 1 to indicate ack received */
	if (status == 0x20)										/* Check weather SLA+R transmitted & nack received or not? */
	return 2;												/* If yes then return 2 to indicate nack received i.e. device is busy */
	else
	return 3;												/* Else return 3 to indicate SLA+W failed */
}

void I2C_Stop()												/* I2C stop function */
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);					/* Enable TWI, generate stop condition and clear interrupt flag */
	while(TWCR & (1<<TWSTO));								/* Wait until stop condition execution */
}

void I2C_Start_Wait(char slave_write_address)				/* I2C start wait function */
{
	uint8_t status;											/* Declare variable */
	while (1)
	{
		TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);				/* Enable TWI, generate start condition and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (start condition) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x08)									/* Check weather start condition transmitted successfully or not? */
		continue;		
											/* If no then continue with start loop again */
		TWDR = slave_write_address;							/* If yes then write SLA+W in TWI data register */
		TWCR = (1<<TWEN)|(1<<TWINT);						/* Enable TWI and clear interrupt flag */
		while (!(TWCR & (1<<TWINT)));						/* Wait until TWI finish its current job (Write operation) */
		status = TWSR & 0xF8;								/* Read TWI status register with masking lower three bits */
		if (status != 0x18 )								/* Check weather SLA+W transmitted & ack received or not? */
		{
			I2C_Stop();										/* If not then generate stop condition */
			continue;										/* continue with start loop again */
		}
		break;												/* If yes then break loop */
	}
}

uint8_t I2C_Write(char data)								/* I2C write function */
{
	uint8_t status;											/* Declare variable */
	TWDR = data;											/* Copy data in TWI data register */
	TWCR = (1<<TWEN)|(1<<TWINT);							/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (Write operation) */
	status = TWSR & 0xF8;									/* Read TWI status register with masking lower three bits */
	if (status == 0x28)										/* Check weather data transmitted & ack received or not? */
	return 0;												/* If yes then return 0 to indicate ack received */
	if (status == 0x30)										/* Check weather data transmitted & nack received or not? */
	return 1;												/* If yes then return 1 to indicate nack received */
	else
	return 2;												/* Else return 2 to indicate data transmission failed */
}

char I2C_Read_Ack()										/* I2C read ack function */
{  
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);					/* Enable TWI, generation of ack and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}

char I2C_Read_Nack()										/* I2C read nack function */
{
	TWCR=(1<<TWEN)|(1<<TWINT);								/* Enable TWI and clear interrupt flag */
	while (!(TWCR & (1<<TWINT)));							/* Wait until TWI finish its current job (read operation) */
	return TWDR;											/* Return received data */
}

void USART_Init(unsigned long BAUDRATE)				/* USART initialize function */
{
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);				/* Enable USART transmitter and receiver */
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);	/* Write USCRC for 8 bit data and 1 stop bit */
	UBRR0L = BAUD_PRESCALE;							/* Load UBRRL with lower 8 bit of prescale value */
	UBRR0H = (BAUD_PRESCALE >> 8);					/* Load UBRRH with upper 8 bit of prescale value */
}

char USART_RxChar()									/* Data receiving function */
{
	while (!(UCSR0A & (1 << RXC0)));					/* Wait until new data receive */
	return(UDR0);									/* Get and return received data */
}

void USART_TxChar(char data)						/* Data transmitting function */
{
	UDR0 = data;										/* Write data to be transmitting in UDR */
	while (!(UCSR0A & (1<<UDRE0)));					/* Wait until data transmit and buffer get empty */
}

void USART_SendString(char *str)					/* Send string of USART data function */
{
	int i=0;
	while (str[i]!=0)
	{
		USART_TxChar(str[i]);						/* Send each char of string till the NULL */
		i++;
	}
}

void intarray(int num)                   //Function to transmit integers
{
	int i;
	int dup = num;
	int dup2 = num;
	int n = 0,k;
	
	for(i=0;dup>0;i++)
	{
		dup = dup/10;
		n++;
		
	}
	
	unsigned char number[n];
	
	for(i=0;i<n;i++)
	{
		k = dup2 % 10;
		number[i]= '0' + k;
		dup2 = (dup2)/10;
	}
	
	for(i=n-1;i>=0;i--)
	{
		USART_TxChar(number[i]);
	}
}

int main()
{
	//char buffer[20], float_[10];
	double roll = 0.0, yaw= 0.0, pitch=0.0;
	double roll_acc, yaw_acc, pitch_acc;
	//bool flag;
	double dt = 0.005;
	double alpha = 0.9;
	double pi = 3.1415;
	//float force;
	float Xa,Ya,Za;
	double a, b ,c;
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	USART_Init(4800);									/* Initialize USART with 9600 baud rate */
	
	while(1)
	{
		//flag = 0;
		Read_RawValue();
		
		Acc_x = Acc_x/16384.0;
		Acc_y = Acc_y/16384.0;
		Acc_z = Acc_z/16384.0;
		Gyro_x = Gyro_x/16.4;
		Gyro_y = Gyro_y/16.4;
		Gyro_z = Gyro_z/16.4;
		
		//force = float(sqrt((Acc_x*Acc_x)+(Acc_y*Acc_y)+(Acc_z*Acc_z)));
		
		roll_acc = (atan2(Acc_z,Acc_y))*180/pi;
		pitch_acc = (atan2(Acc_x,Acc_z))*180/pi;
		yaw_acc = (atan2(Acc_y,Acc_x))*180/pi;
		
		a = (roll + ((Gyro_x*dt)*180/pi))*alpha;
		b = (pitch + ((Gyro_y*dt)*180/pi))*alpha;
		c = (yaw + ((Gyro_z*dt)*180/pi))*alpha;
		
		
		roll = a + (1-alpha)*(roll_acc);
		pitch = b + (1-alpha)*(pitch_acc);
		yaw = c + (1-alpha)*(yaw_acc);
		
		Xa = float (roll);
		Ya = float (pitch);
		Za = float (yaw);
		
		

// 		dtostrf( Xa, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
// 		sprintf(buffer," Ax = %s         ",float_);
// 		USART_SendString(buffer);
// 		USART_TxChar('\n');
		
		
// 		dtostrf( force, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
// 		sprintf(buffer," F = %s         ",float_);
// 		USART_SendString(buffer);
// 		USART_TxChar('\n');
		


		
// 		dtostrf( Xa, 3, 2, float_ );
// 		sprintf(buffer," Ax = %s        ",float_);
// 		USART_SendString(buffer);
// 		USART_TxChar('\n');
		

// 		
// 		dtostrf( Za, 3, 2, float_ );
// 		sprintf(buffer," Az = %s         ",float_);
// 		USART_SendString(buffer);
// 		USART_TxChar('\n');
		

		if(((Ya<0.0) && (Ya>-100.0)) && ((Xa>65.0) && (Xa<95.0))) {
			USART_TxChar('L');
			//flag = 1;
		}
		
		else if(((Ya<100.0) && (Ya>0.0)) && ((Xa>65.0) && (Xa<95.0))) {
			USART_TxChar('R');
			//flag = 1;
		}
		
		else if(((Ya<10.0) && (Ya>-20.0)) && ((Xa>-50.0) && (Xa<90.0))) {
			USART_TxChar('F');
			//flag = 1;
		}
				
		else if(((Ya<12.0) && (Ya>-30.0)) && ((Xa>90.0) && (Xa<200.0))) {
			USART_TxChar('B');
			//flag = 1;
		}
		/*if(((Ya<20.0) && (Ya>-20.0)) && ((Xa>65.0) && (Xa<95.0))) {
			USART_TxChar('S');
			flag = 1;
		}*/
		else
		{
			//flag==0;
			USART_TxChar('S');
		}

	}
}