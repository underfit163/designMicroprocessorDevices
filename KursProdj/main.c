/*
 * tyshKurs.c
 *
 * Created: 18.10.2020 22:49:28
 * Author : 163ty
 */ 

#include <avr/io.h> // ���������, ����� ������������� �������� ������ �� ���������
#define F_CPU 8000000UL // ������� �������� ������� ����������������
#include <util/delay.h> // ���������, ����� ������������� ������� �������� � ���������
#include <avr/interrupt.h> //���������, ����� ������������� ����������

void writeChar(char data);//������� ������ ���������� �����
void outStrRAM(char *str);//������� ������ ������ ��������, ��������� ��������� �� ������(1 �������)
void LCDInit();//������������� �������
void writeCom(unsigned char com);//����� �������
void SetStr(unsigned char str);//��������� ������� �� ������� ������

volatile unsigned long fuelRate = 0; // ���������� ������� �������
volatile unsigned long metrs = 0; // ���������� ����
volatile unsigned char countRound = 0; // ������� ������ �� 1000�� �� ��������
volatile unsigned char countInterCT2 = 0; //���������� ���������� CT2
volatile unsigned char startStop = 0; //���� ������/���������
volatile unsigned char i2cData[11]; // ������ ������ ��� ��������
volatile unsigned long i2cMetrs = 0; // ���������� �������� ����������� ����
volatile unsigned long i2cFuel = 0; // ���������� �������� ������� �������
volatile unsigned int i2cAdr = 0; // ����� ������ ���� ������ 
volatile unsigned int i2cCount = 0; // ����� ������  
volatile unsigned long i2cMetrsPrev = 0;//���������� �������� ����������
volatile unsigned char I2CFlag = 1; //���� ��������� I2C(������ �������): 0-������,1-��������
volatile unsigned char I2cCountB = 0;// ����� ����
volatile unsigned char LCDFlag = 0;//���� ���������� ������� 

void interPort(void)
{
	DDRD &= ~((1<<2)|(1<<3));//��������� �� ����
	PORTD |= ((1<<2)|(1<<3)); // � ����������� � 1(PullUp)
	MCUCR &= ~(1<<ISC00);//�� ���������� ������ ������� �� ������ INT0
	MCUCR |= (1<<ISC01);
    MCUCR &= ~(1<<ISC10);//�� ���������� ������ ������� �� ������ INT1
	MCUCR |= (1<<ISC11);
	GICR |= (1<<INT1); //���������� ���������� INT1
		
}

ISR(INT0_vect)// ���������� ����
{
 metrs++; //����������� �� 1 ���� ����
 if(metrs>=1000000) // ��������� �� 1000 ��
 {
	 metrs=0; // �������� ����
	 countRound++; // ����������� �� 1 ������� �����
 }
}

ISR(INT1_vect) // ���������� �����/����
{
 if(startStop==1)//���� ���������� �������� - ���������
 {
     MCUCR &= ~(1<<ISC10);//�� ���������� ������ ������� �� ������ INT1
     MCUCR |= (1<<ISC11);	
	 startStop = 0;// ���� ���������
	 LCDFlag = 0; // �� ���������	 
	 GICR &= ~(1<<INT0);	//��������� ���������� �������
	 TIMSK &= ~(1<<OCIE2); //��������� ��������� ����������
	 
	 TCCR2  &= ~((1<<CS20)|(1<<CS21)|(1<<CS22));//������������� ������� 
	 TCCR1B &= ~((1<<CS10)|(1<<CS11)|(1<<CS12)); //������������� ������� 
 }
 else if(startStop==0)//���� ���������� �����������-��������
 {
    MCUCR |= (1<<ISC10)|(1<<ISC11);//�� ������������ ������ ������� �� ������ INT1
	startStop = 1;// ���� ������
	LCDFlag = 0; // �� ���������
	GICR |= (1<<INT0);	//�������� ������� �������
	TIMSK |= (1<<OCIE2); //�������� ��������� ����������
	TCCR1B |= (1<<CS10)|(1<<CS11)|(1<<CS12);//�������� ������� �������
	TCCR2 |= ((1<<CS22)|(1<<CS21));// clk/256 ������ ��2
	TCCR2 &= ~(1<<CS20);
 }
}
static unsigned char flag2sec = 0;//���� ��� ������ 2 �������, ��� ���������� ������ ��� ����������� 0.1%
ISR(TIMER2_COMP_vect)//��������� ���������� �� ��2
{
TCNT2 = 0; // �������� �������� �������
countInterCT2++; //����������� �� 1 ���. ����������
if(countInterCT2 == 125)// ���� ���������� ���������� 125
{
	if(flag2sec==1)
	{
	fuelRate +=TCNT1; // ���������� ������ �� 2 �������
	TCNT1 = 0; //���������� ��1
	flag2sec = 0;
	}
	else if(flag2sec==0)
	{
	   flag2sec = 1;	
	}
	countInterCT2=0;//���������� ���� ���������� ��2
	LCDFlag = 1; // ��������� �������
} 
}

ISR(TWI_vect)//��������� ���������� �� I2C
{
	switch(TWSR) //������� ���������
	{
		case 0x08: // ���� ������������ ��������� �����
		{
			TWDR=0xA2;//���������� ����� ���������� + 0
			TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//�������� ������
			break;
		}
		case 0x18://��� ������� ����� SLA+W � ������� �������������(ACK)
		{
			TWDR=(i2cAdr & 0xFF00)>>8;//���������� ������� ���� ������
			I2cCountB = 0;
			TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//�������� ������
			break;
		}
		case 0x28://��� ������� ����� ������ � ������� �������������(ACK)
		{
			if(I2cCountB == 12)// ���� ������ ����
			{
				I2cCountB = 0;//�������� ���� ������
				i2cAdr +=11;// ������ �����
				i2cCount++;//������ ���������� �������
				if(i2cCount == 372) //���� ����� ������, ��������
				{
					i2cCount = 0;	 
					i2cAdr = 0;
				}
				TWCR=(1<<TWEA)|(1<<TWEN)|(1<<TWSTO);//����
				I2CFlag = 1;
			}
			else{
				if(I2cCountB == 0)// ���� �������� ���� ����
				{
					TWDR = (i2cAdr & 0xFF); //�������� ������� ����� ������
					I2cCountB++;//������. �� 1 ����� ����� ����
				}
				else// ���� ������� >1 ����
				{	
					TWDR = (i2cData[I2cCountB-1]);//�������� ���� 
					I2cCountB++;//������. �� 1 ����� �����. ����
				}
					TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//�������� ������
				}	
				break;
		}
		default://��� �� ������
		{
			TWCR|=(1<<TWINT)|(1<<TWSTO); // ����� ������������ ��������� ����
			break;
		}
	}
}

void writeChar(char data)//������� ������ ���������� �����
{
	unsigned char tmp;

	tmp = PORTC & 0xF0;	//���������� ������� ��������� ������� ����� �����, ����� �� �������� ��������
	tmp |= (data & 0xF0)>>4;//� ������� ����� ����� ������� ����� ������
    PORTC = tmp;//������� ������� �������
	
	PORTB |= (1<<0); //��� �����
	_delay_us(40); //���
	PORTB &= ~(1<<0); //������� �����
	
	_delay_us(40);	//����� ����� ���������
	
	tmp  = PORTC & 0xF0;//���������� ������� ��������� ������� ����� �����, ����� �� �������� ��������
	tmp |= (data & 0x0F);//� ������� ����� ����� ������� ����� ������
	PORTC = tmp;//������� ������� �������

	PORTB |= (1<<0); //��� �����
	_delay_us(40); //���
	PORTB &= ~(1<<0); //������� �����
	_delay_us(40);	//����� ����� ���������
}

void LCDInit()//������� ������������� �������
{
	DDRC |= 0x0F;//�������������� �����
	DDRB |= 0x03; 
	_delay_ms(20);//����
	PORTB &= ~(1<<1);//����� �������
	writeChar(0x02);// �������: ������ � ����� ������� �������, ������ �� ���������
	_delay_ms(2);//���� 2 ������������
	writeChar(0x28);// �������: 4-� ������ ����, 2 ������, ����� 5*7
	writeChar(0x08);//��������� ������� 
	writeChar(0x01);//������ �����, ������� ������, ������ � ����� ������� �������
	_delay_ms(2);//���� 2 ������������, ����� ������� ��������
	writeChar(0x06);//����� ������ ���������� ������� ������ ������������� ���������� �� ���� ���������� ������
	writeChar(0x0C);//����������� ����� �����������, �� ������ �� �����
	PORTB |= (1<<1);//���� ������
	_delay_ms(40); //������� ���
}

void writeCom(unsigned char com)//����� �������
{
	PORTB &= ~(1<<1); //��� �������
	writeChar(com);//������� �������
	PORTB |= (1<<1); //��� ������
}

void SetStr(unsigned char str)//��������� ������� �� ������� ������
{
	switch(str)
	{
		case 1: {writeCom(0x80); break;}	//1 ������
		case 2: {writeCom(0xC0); break;}	//2 ������
		default: break;
	};
}

void outStrRAM(char *str)//������� ������ ������ ��������, ��������� ��������� �� ������
{
	while(*str)//���� ������ �� �����������
	writeChar(*str++);//������� ������� �������, ���������� � ������ 1
}


void writeLCD()
{
	SetStr(1);	//������ ������ �� 1 ������
	outStrRAM("Dist:");	//������� ������
	writeChar((i2cMetrs%10000000)/1000000+0x30);
	writeChar((i2cMetrs%1000000)/100000+0x30);
	writeChar((i2cMetrs%100000)/10000+0x30);
	writeChar((i2cMetrs%10000)/1000+0x30);
	writeChar('.');
	writeChar((i2cMetrs%1000)/100+0x30);
	writeChar((i2cMetrs%100)/10+0x30);
	writeChar(i2cMetrs%10+0x30);
	writeChar('k');
	writeChar('m');
	SetStr(2); //������ ������ �� 2 ������
	outStrRAM("Fuel:"); //������� ������
	writeChar((i2cFuel%10000000)/1000000+0x30);
	writeChar((i2cFuel%1000000)/100000+0x30);
	writeChar((i2cFuel%100000)/10000+0x30);
	writeChar((i2cFuel%10000)/1000+0x30);
	writeChar((i2cFuel%1000)/100+0x30);
	writeChar((i2cFuel%100)/10+0x30);
	writeChar(i2cFuel%10+0x30);
	outStrRAM("lit");
}
	  
int main(void)
{   
	LCDInit(); //�������������� LCD
	writeLCD(); //����� ��������� ������
	interPort();// �������� ������� ��� ����������	
	// ��������� CT2
	OCR2 = 250-1;// ������� � ��2 ��� ��������
	TCNT2=0; // �������� ��2
	TCNT1= 0;//�������� ��1
	metrs = 0; //�������� ����������
	fuelRate = 0; //�������� ������
	//��������� I2C 
	TWBR = 0x20;//�������� 100���
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));//����������� ������� ����������� 0	
	sei(); // ��������� ����������	

    while (1) 
    {
	 if((startStop!=0))//���� �� �����������, �������
	 {
		i2cMetrs = metrs; // ���������� �������� ����������
		i2cFuel = fuelRate*0.001; // ���������� ������ �������
		if(LCDFlag==1)//���� ������ �������
		{
			writeLCD();// ����� �� �������
			LCDFlag=0;			
		}
		if((i2cMetrs-i2cMetrsPrev)>=1000) // ���� ������� 10 ��, ��������� ������
		{
		i2cMetrsPrev = i2cMetrs; //���������� ������
		i2cData[0] = (i2cCount&0xFF00)>>8;//���������� �� � ������
		i2cData[1] = (i2cCount&0xFF);
		i2cData[2] = (i2cFuel&0xFF000000)>>24;//������
		i2cData[3] = (i2cFuel&0xFF0000)>>16;
		i2cData[4] = (i2cFuel&0xFF00)>>8;
		i2cData[5] = (i2cFuel&0xFF);
		i2cData[6] = (i2cMetrs&0xFF000000)>>24;//������
		i2cData[7] = (i2cMetrs&0xFF0000)>>16;
		i2cData[8] = (i2cMetrs&0xFF00)>>8;
		i2cData[9] = (i2cMetrs&0xFF);
		i2cData[10] = countRound;//���������� ������
		
		TWCR|=((1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE));// ��������� ��������� ����� � ���������� ����������
		while(I2CFlag==0);//������, ���� ������
	    }		
	 }
	}
}


