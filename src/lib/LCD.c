
/* Автор:              Papandopala Papandopalavich
 * Имя файла:          Atmega16_LCD_LIB.c
 * Тип МК:			   Atmega16, Flash 16 Kbytes, SRAM 1 Kbytes, EEPROM 512 bytes.
 * Тактовая частота:   F_CPU 8000000 Гц
 * Дата:               28.06.2013 21:17:29
 * Версия ПО:          ATMEL STUDIO VERSION 6.1.2562
 * FUSES:              HIGH xx, LOW xx;
 * Описание:           Урезанная библиотека для работы с LCD дисплеями базирующимися на HD44780 
					   контроллерах. Данная библиотека работает по 4х и 8ми битной схеме подключения
					   LCD, поддерживает только вывод данных на дисплей*/ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "LCD.h"

//Системные функции, их не трогаем, они для работы дисплея.
void Send_byte(uint8_t);	//Передача данных LCD, вызывается функциями Send_command и Send_data.
void Busy_flag(void);		//Проверка флага занятости.
void HiPin(void);			//Переводим порт в состояние HI.
void OutPin(void);			//Переводим порт в состояние "выход".
void InPin(void);			//Переводим порт в состояние "вход".
void Strob(void);			//Вкл/выкл строба Е


//Пользовательские функции, ими пользуемся в программе.
void LCDinit(void)							//Инициализируем дисплей
{
	//Перед инициализацией LCD, надо в init задержку в 100 мсек сделать, чтоб питание устаканилось. 
	_delay_ms(100);
	CDDR |=  (1<<RS)|(1<<E)|(1<<RW);   //Настройка портов 
	CPORT&=~((1<<RS)|(1<<E)|(1<<RW));  //Настройка портов 
	OutPin();						   //Настройка портов
	
	uint8_t i=0;
	while (i!=3)
	{
		#ifdef  LCD_8BIT       //отправка символа 0x30.
		        DPORT|=(0<<DB7)|(0<<DB6)|(1<<DB5)|(1<<DB4)|(0<<DB3)|(0<<DB2)|(0<<DB1)|(0<<DB0);
		#else
				DPORT|=(0<<DB7)|(0<<DB6)|(1<<DB5)|(1<<DB4);
		#endif
		
		Strob();
		_delay_ms(5);
		i++;
	}
	
	//Разминочный этап закончен, далее переходим к основным упражнениям.
	
	#ifdef  LCD_8BIT
			LCDcommand(0b00111000);//8ми битный интерфейс, две строки, 5x8 точек.
	#else   //Первый раз отправляем только пол старшей тетрады
			Busy_flag();
			OutPin();
			DPORT|=(0<<DB7)|(0<<DB6)|(1<<DB5)|(0<<DB4);//4х битный интерфейс 
			Strob();
			LCDcommand(0b00101000);//Две строки, 5x8 точек.
	#endif
	
	LCDcommand(0b1100);  //Включаем дисплей + без отображения курсоров.
	LCDcommand(0b110);   //Счетчик адреса всегда будет смещаться на n+1
	LCDcommand(0b10);    //курсор в позицию 0,0 + сброс всех сдвигов
	LCDcommand(0b1);     //очистка дисплея 
	//Основные упражнения закончены. Переходим к релаксации.
}
void LCDcommand(uint8_t i)					//Отправка команды, настройка дисплея.
{
	Busy_flag();	//Проверим сперва флаг занятости, а свободен ли дисплей?
	Send_byte(i);
}
void LCDdata(uint8_t i)						//Отправка символа для отображения на дисплее.
{
	Busy_flag();	//Проверим сперва флаг занятости, а свободен ли дисплей?
	CPORT|=(1<<RS); //RS=1 посылаем данные в LCD
	Send_byte(i);
	CPORT&=~(1<<RS);//RS=0
}

void LCDGotoXY(uint8_t x,uint8_t y)			//Устанавливаем курсор в X, Y позицию
{
	 uint8_t Address;
	
	switch(y)
	{
		case 0: Address = LINE0+x; 
				break;
		case 1: Address = LINE1+x; 
				break;
		default: Address = LINE0+x;
	}
	
	LCDcommand(1<<7 | Address);
}

void LCDstring(char *i,uint8_t x,uint8_t y) //Вывести строку на дисплей.
{
	LCDGotoXY(x,y);
	while( *i )
	{
		LCDdata(*i++ );
	}
}

void LCDclear(void)			//Очистка дисплея + курсор на позицию 0,0
{
	LCDcommand(0b1);
}

//Системные функции, их не трогаем, они для работы дисплея.
void Send_byte(uint8_t i)	//Передача данных LCD, вызывается функциями Send_command и Send_data.
{
	OutPin();
	
	#ifdef  LCD_8BIT
			if ((i&1)==1)
			{
				DPORT|=(1<<DB0);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB1);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB2);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB3);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB4);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB5);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB6);
			}
			i=i>>1;
			
			if ((i&1)==1)
			{
				DPORT|=(1<<DB7);
			}
			
		    Strob();
			
	#else
			
			uint8_t higb = ((i&0b11110000)>>4);
			uint8_t lowb =  (i&0b00001111);
	
			if ((higb&1)==1)
			{
				DPORT|=(1<<DB4);
			}
			higb=higb>>1;
			if ((higb&1)==1)
			{
				DPORT|=(1<<DB5);
			}
			higb=higb>>1;
			if ((higb&1)==1)
			{
				DPORT|=(1<<DB6);
			}
			higb=higb>>1;
			if ((higb&1)==1)
			{
				DPORT|=(1<<DB7);
			}
	
			Strob();
			OutPin();
	
			if ((lowb&1)==1)
			{
				DPORT|=(1<<DB4);
			}
			lowb=lowb>>1;
			if ((lowb&1)==1)
			{
				DPORT|=(1<<DB5);
			}
			lowb=lowb>>1;
			if ((lowb&1)==1)
			{
				DPORT|=(1<<DB6);
			}
			lowb=lowb>>1;
			if ((lowb&1)==1)
			{
				DPORT|=(1<<DB7);
			}
	
			Strob();
	#endif
	
	HiPin();
}
void Busy_flag(void)		//Проверка флага занятости
{
    _delay_us(50);
}

void HiPin(void)			//Переводим порт в состояние Hi.
{
	#ifdef  LCD_8BIT
			DDDR =0;
			DPORT=0;
	#else
			DDDR &=~((1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4));
			DPORT&=~((1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4));
    #endif
}

void OutPin(void)			//Переводим порт в состояние "выход".
{
	#ifdef  LCD_8BIT
			DDDR =0xFF;
			DPORT=0;
	#else
			DDDR |=  (1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4);
			DPORT&=~((1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4));
	#endif
}

void InPin(void)			//Переводим порт в состояние "вход".
{
	#ifdef  LCD_8BIT
			DDDR =0;
			DPORT=0xFF;
	#else
			DDDR &=~((1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4));
			DPORT|=  (1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4);
	#endif
}

void Strob(void)            //Включаем выключаем стробарь.
{
	CPORT|=(1<<E);//Включили строб.
	_delay_us(1);
	CPORT&=~(1<<E);//Выключили строб.
}
