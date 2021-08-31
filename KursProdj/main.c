/*
 * tyshKurs.c
 *
 * Created: 18.10.2020 22:49:28
 * Author : 163ty
 */ 

#include <avr/io.h> // заголовок, чтобы задействовать контроль данных на контактах
#define F_CPU 8000000UL // задание тактовой частоты микроконтроллера
#include <util/delay.h> // заголовок, чтобы задействовать функции задержки в программе
#include <avr/interrupt.h> //заголовок, чтобы задействовать прерывания

void writeChar(char data);//Функция вывода отдельного байта
void outStrRAM(char *str);//Функция вывода строки символов, принимаем указатель на массив(1 элемент)
void LCDInit();//Инициализация дисплея
void writeCom(unsigned char com);//пишем команду
void SetStr(unsigned char str);//Установка курсора на текущую строку

volatile unsigned long fuelRate = 0; // переменная расхода топлива
volatile unsigned long metrs = 0; // Пройденный путь
volatile unsigned char countRound = 0; // сколько кругов по 1000км он накрутил
volatile unsigned char countInterCT2 = 0; //количество прерываний CT2
volatile unsigned char startStop = 0; //флаг старта/остановки
volatile unsigned char i2cData[11]; // массив данных для передачи
volatile unsigned long i2cMetrs = 0; // запоминаем значение пройденного пути
volatile unsigned long i2cFuel = 0; // запоминаем значение расхода топлива
volatile unsigned int i2cAdr = 0; // адрес ячейки флеш памяти 
volatile unsigned int i2cCount = 0; // номер записи  
volatile unsigned long i2cMetrsPrev = 0;//предыдущее значение расстояния
volatile unsigned char I2CFlag = 1; //флаг занятости I2C(ждущей вершины): 0-занято,1-свободно
volatile unsigned char I2cCountB = 0;// число байт
volatile unsigned char LCDFlag = 0;//флаг обновление дисплея 

void interPort(void)
{
	DDRD &= ~((1<<2)|(1<<3));//настройка на вход
	PORTD |= ((1<<2)|(1<<3)); // с подтяжкокой к 1(PullUp)
	MCUCR &= ~(1<<ISC00);//По спадающему фронту сигнала на выводе INT0
	MCUCR |= (1<<ISC01);
    MCUCR &= ~(1<<ISC10);//По спадающему фронту сигнала на выводе INT1
	MCUCR |= (1<<ISC11);
	GICR |= (1<<INT1); //Разрешение прерывания INT1
		
}

ISR(INT0_vect)// прерывание пути
{
 metrs++; //увеличиваем на 1 метр путь
 if(metrs>=1000000) // проверяем на 1000 км
 {
	 metrs=0; // обнуляем путь
	 countRound++; // увеличиваем на 1 подсчет круга
 }
}

ISR(INT1_vect) // прерывание старт/стоп
{
 if(startStop==1)//Если устройство запущено - выключаем
 {
     MCUCR &= ~(1<<ISC10);//По спадающему фронту сигнала на выводе INT1
     MCUCR |= (1<<ISC11);	
	 startStop = 0;// флаг остановки
	 LCDFlag = 0; // не обновляем	 
	 GICR &= ~(1<<INT0);	//Выключаем прерывание пробега
	 TIMSK &= ~(1<<OCIE2); //Выключаем секундное прерывание
	 
	 TCCR2  &= ~((1<<CS20)|(1<<CS21)|(1<<CS22));//Останавливаем подсчёт 
	 TCCR1B &= ~((1<<CS10)|(1<<CS11)|(1<<CS12)); //Останавливаем подсчёт 
 }
 else if(startStop==0)//Если устройство остановлено-включаем
 {
    MCUCR |= (1<<ISC10)|(1<<ISC11);//По нарастающему фронту сигнала на выводе INT1
	startStop = 1;// флаг старта
	LCDFlag = 0; // не обновляем
	GICR |= (1<<INT0);	//Включаем подсчёт пробега
	TIMSK |= (1<<OCIE2); //Включаем секундное прерывание
	TCCR1B |= (1<<CS10)|(1<<CS11)|(1<<CS12);//Включаем подсчёт расхода
	TCCR2 |= ((1<<CS22)|(1<<CS21));// clk/256 запуск СТ2
	TCCR2 &= ~(1<<CS20);
 }
}
static unsigned char flag2sec = 0;//флаг что прошло 2 секунды, для соблюдения нужной нам погрешности 0.1%
ISR(TIMER2_COMP_vect)//обработка прерывания от СТ2
{
TCNT2 = 0; // обнуляем значение секунды
countInterCT2++; //увеличиваем на 1 кол. прерываний
if(countInterCT2 == 125)// если количество прерываний 125
{
	if(flag2sec==1)
	{
	fuelRate +=TCNT1; // прибавляем расход за 2 секунды
	TCNT1 = 0; //сбрасываем СТ1
	flag2sec = 0;
	}
	else if(flag2sec==0)
	{
	   flag2sec = 1;	
	}
	countInterCT2=0;//сбрасываем счет прерываний СТ2
	LCDFlag = 1; // обновляем дисплей
} 
}

ISR(TWI_vect)//обработка прерывания от I2C
{
	switch(TWSR) //изучаем состояния
	{
		case 0x08: // было сформировано состояние старт
		{
			TWDR=0xA2;//отправляем адрес устройства + 0
			TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//загрузка данных
			break;
		}
		case 0x18://был передан пакет SLA+W и принято подтверждение(ACK)
		{
			TWDR=(i2cAdr & 0xFF00)>>8;//выставляем старший байт адреса
			I2cCountB = 0;
			TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//загрузка данных
			break;
		}
		case 0x28://Был передан пакет данных и принято подтверждение(ACK)
		{
			if(I2cCountB == 12)// если послед байт
			{
				I2cCountB = 0;//обнуляем счет байтов
				i2cAdr +=11;// увелич адрес
				i2cCount++;//увелич количество записей
				if(i2cCount == 372) //Если конец памяти, обнуляем
				{
					i2cCount = 0;	 
					i2cAdr = 0;
				}
				TWCR=(1<<TWEA)|(1<<TWEN)|(1<<TWSTO);//Стоп
				I2CFlag = 1;
			}
			else{
				if(I2cCountB == 0)// если передано ноль байт
				{
					TWDR = (i2cAdr & 0xFF); //передаем младший адрес записи
					I2cCountB++;//увелич. на 1 число перед байт
				}
				else// если передан >1 байт
				{	
					TWDR = (i2cData[I2cCountB-1]);//передаем байт 
					I2cCountB++;//увелич. на 1 число перед. байт
				}
					TWCR|=((1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE));//загрузка данных
				}	
				break;
		}
		default://что то другое
		{
			TWCR|=(1<<TWINT)|(1<<TWSTO); // будет сформировано состояние Стоп
			break;
		}
	}
}

void writeChar(char data)//Функция вывода отдельного байта
{
	unsigned char tmp;

	tmp = PORTC & 0xF0;	//Запоминаем текущее состояние старшей части порта, чтобы не потерять значения
	tmp |= (data & 0xF0)>>4;//В младшую часть пишем старшую часть данных
    PORTC = tmp;//Выводим старшую тетраду
	
	PORTB |= (1<<0); //Даём строб
	_delay_us(40); //Ждём
	PORTB &= ~(1<<0); //Снимаем строб
	
	_delay_us(40);	//Пауза между командами
	
	tmp  = PORTC & 0xF0;//Запоминаем текущее состояние старшей части порта, чтобы не потерять значения
	tmp |= (data & 0x0F);//В младшую часть пишем младшую часть данных
	PORTC = tmp;//Выводим младшую тетраду

	PORTB |= (1<<0); //Даём строб
	_delay_us(40); //Ждём
	PORTB &= ~(1<<0); //Снимаем строб
	_delay_us(40);	//Пауза между командами
}

void LCDInit()//Функция инициализации дисплея
{
	DDRC |= 0x0F;//Инициализируем порты
	DDRB |= 0x03; 
	_delay_ms(20);//Ждем
	PORTB &= ~(1<<1);//Вывод команды
	writeChar(0x02);// Команда: Курсор в левой верхней позиции, память не очищается
	_delay_ms(2);//Ждем 2 миллисекунды
	writeChar(0x28);// Команда: 4-х битная шина, 2 строки, шрифт 5*7
	writeChar(0x08);//Выключить дисплей 
	writeChar(0x01);//Пустой экран, очистка памяти, курсор в левой верхней позиции
	_delay_ms(2);//Ждем 2 миллисекунды, чтобы дисплей очухался
	writeChar(0x06);//После вывода очередного символа курсор автоматически сдвигается на одно знакоместо вправо
	writeChar(0x0C);//Разрешается вывод изображения, но курсор не виден
	PORTB |= (1<<1);//Даем данные
	_delay_ms(40); //Немного ждём
}

void writeCom(unsigned char com)//пишем команду
{
	PORTB &= ~(1<<1); //Даём команду
	writeChar(com);//Выводим команду
	PORTB |= (1<<1); //Даём данные
}

void SetStr(unsigned char str)//Установка курсора на текущую строку
{
	switch(str)
	{
		case 1: {writeCom(0x80); break;}	//1 строка
		case 2: {writeCom(0xC0); break;}	//2 строка
		default: break;
	};
}

void outStrRAM(char *str)//Функция вывода строки символов, принимаем указатель на массив
{
	while(*str)//Пока строка не закончилась
	writeChar(*str++);//Выводим элемент массива, прибавляем к адресу 1
}


void writeLCD()
{
	SetStr(1);	//Ставим курсор на 1 строку
	outStrRAM("Dist:");	//Выводим пробег
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
	SetStr(2); //Ставим курсор на 2 строку
	outStrRAM("Fuel:"); //Выводим расход
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
	LCDInit(); //Инициализируем LCD
	writeLCD(); //Вывод начальных данных
	interPort();// вызываем функцию для прерываний	
	// Настройка CT2
	OCR2 = 250-1;// заносим в СТ2 для сранения
	TCNT2=0; // обнуляем СТ2
	TCNT1= 0;//обнуляем СТ1
	metrs = 0; //обнуляем расстояние
	fuelRate = 0; //обнуляем расход
	//настройка I2C 
	TWBR = 0x20;//скорость 100кГц
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));//коэффициент деления пределителя 0	
	sei(); // разрешаем прерывания	

    while (1) 
    {
	 if((startStop!=0))//если не остановлено, заходим
	 {
		i2cMetrs = metrs; // запоминаем значение расстояния
		i2cFuel = fuelRate*0.001; // запоминаем расход топлива
		if(LCDFlag==1)//если прошла секунда
		{
			writeLCD();// пишем на дисплей
			LCDFlag=0;			
		}
		if((i2cMetrs-i2cMetrsPrev)>=1000) // если продено 10 км, передаемм данные
		{
		i2cMetrsPrev = i2cMetrs; //запоминаем пробег
		i2cData[0] = (i2cCount&0xFF00)>>8;//записываем всё в массив
		i2cData[1] = (i2cCount&0xFF);
		i2cData[2] = (i2cFuel&0xFF000000)>>24;//Расход
		i2cData[3] = (i2cFuel&0xFF0000)>>16;
		i2cData[4] = (i2cFuel&0xFF00)>>8;
		i2cData[5] = (i2cFuel&0xFF);
		i2cData[6] = (i2cMetrs&0xFF000000)>>24;//Пробег
		i2cData[7] = (i2cMetrs&0xFF0000)>>16;
		i2cData[8] = (i2cMetrs&0xFF00)>>8;
		i2cData[9] = (i2cMetrs&0xFF);
		i2cData[10] = countRound;//количество кругов
		
		TWCR|=((1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE));// формируем состояние старт с разрешеним прерывания
		while(I2CFlag==0);//занято, идет запись
	    }		
	 }
	}
}


