
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

#include <inttypes.h>

//---------------------------------------------------------------------------------------------
//Если  хочешь использовать 8ми битную схему подключения, тогда раскомментируй #define LCD_8BIT
//#define LCD_8BIT
//---------------------------------------------------------------------------------------------

#ifndef LCD
#define LCD

#if 1

//Указываем порт к которому подключены выводы дисплея LCD DB0...DB7.
#define DPIN  PINA
#define DDDR  DDRA
#define DPORT PORTA
						
//Пины  МК      LCD   
#define DB0	0// DB0
#define DB1	1// DB1
#define DB2	2// DB2	
#define DB3	3// DB3		
#define DB4	4// DB4  
#define DB5	5// DB5
#define DB6	6// DB6
#define DB7	7// DB7 + BF флаг занятости дисплея.

//Указываем порт к которому подключены выводы дисплея E, RS, R/W.
#define CDDR  DDRA
#define CPORT PORTA

// Указываем номера пинов МК, к которым подключаем дисплей.
#define E	1	// E	 СТРОБ.
#define RW	2   // R/W   R/W=1 читаем из LCD, R/W=0 записываем в LCD.
#define RS	0 	// RS	 RS=0 посылаем команду в LCD, RS=1 посылаем данные в LCD.

#endif
//----------------------------------Настройки закончены---------------------------------
#if 1
//Пользовательские функции, ими пользуемся в программе.
void LCDinit(void);							//Инициализация LCD                    
void LCDcommand(uint8_t);					//Отправка команды, настройка дисплея  
void LCDGotoXY(uint8_t, uint8_t);			//Устанавливаем курсор в X, Y позицию
void LCDdata(uint8_t);						//Вывести 1 символ на дисплей.
void LCDstring(char*i,uint8_t,uint8_t);		//Вывести строку на дисплей в позицию x,y
void LCDclear(void);			//Очистить дисплей от инфо + курсор на позицию 0,0

//Двухстрочный дисплей.
#define LINE0 0x00
#define LINE1 0x40

#endif
#endif
