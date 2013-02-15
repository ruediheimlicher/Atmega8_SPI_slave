/*
    conio.c
    Standard conio routines.
    Part of MicroVGA CONIO library / demo project
    Copyright (c) 2008-9 SECONS s.r.o., http://www.MicroVGA.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "conio.h"
#include "kbd.h"
#include <avr/io.h>
#include <avr/wdt.h>


void clrscr(void)
{
  _putch('\033');
  _putch('[');
  _putch('2');
  _putch('J');
}

void clreol(void)
{
  _putch('\033');
  _putch('[');
  _putch('K');
}


void cursoron(void)
{
  _putch('\033');
  _putch('[');
  _putch('2');
  _putch('5');
  _putch('h');
}

void cursoroff(void)
{
  _putch('\033');
  _putch('[');
  _putch('2');
  _putch('5');
  _putch('l');
}

void textcolor(int color)
{
  _putch('\033');
  _putch('[');
  if (color & 0x8) 
	  _putch('1');
  else _putch('2');
  _putch('m');

  _putch('\033');
  _putch('[');
  _putch('3');
  _putch(((color&0x7)%10)+'0');
  _putch('m');
}

void textbackground(int color)
{
  _putch('\033');
  _putch('[');
  if (color & 0x8) 
	  _putch('5');
  else _putch('6');
  _putch('m');

  _putch('\033');
  _putch('[');
  _putch('4');
  _putch((color&0x7)+'0');
  _putch('m');
}

void textattr(int attr)
{
  textcolor(attr&0xF);
  textbackground(attr>>4);
}

void gotoxy(char x, char y)
{
  if (x>MAX_X || y>MAX_Y)
    return;
  
  x--;
  y--;

  _putch(0x1B);
  _putch('[');
  _putch((y/10)+'0');
  _putch((y%10)+'0');
  _putch(';');
  _putch((x/10)+'0');
  _putch((x%10)+'0');
  _putch('f');
}

#define FOSC 1000000    /* oscillator-frequency in Hz */
#define BAUD 57600  //valid values:9600, 19200, 57600 kbits


void _putch (char ch)
{
   
	while(PINB & (1<<PORT7)); //handshaking
   
#ifdef USR
	while(!(USR & (1<<UDRE))); //transmit buffer is ready to receive data
	
	UDR = ch;    // send character
	while(!(USR & (1<<TXC))); //wait for char to be send
   
	USR &= ~(1<<TXC || 1<<UDRE);
#else //2 UART MCU
	while(!(UCSR0A & (1<<UDRE0))); //transmit buffer is ready to receivce data
	
	UDR0 = ch;    // send character
	while(!(UCSR0A & (1<<TXC0))); //wait for char to be send
   
	UCSR0A &= ~(1<<TXC0 || 1<<UDRE0);
   
#endif
	
}

int _getch (void)
{
	int ch;
   
#ifdef USR
	while (!_kbhit()); /* Wait for incomming data */
	
	ch=UDR; //read uart
   
	if (ch==0)
	{
		while (!_kbhit()); /* Wait for incomming data */
		ch=UDR;
		ch = 0x100 | ch;
	}
#else
	//while (!_kbhit()); /* Wait for incomming data */
	
	ch=UDR0; //read uart
   
	if (ch==0)
	{
		//&while (!_kbhit()); /* Wait for incomming data */
		ch=UDR0;
		ch = 0x100 | ch;
	}
#endif
   
	return ch;
}


void _cputs(ROMDEF char *s)
{
   while (*s != 0) 
    _putch(*s++);
}

char * _cgets(char *s)
{
  char len;
  int ch;
  
  len=0;

  while (s[0]>len)
  {
    ch=_getch();
    
    if (ch==KB_ENTER)
      break; //enter hit, end of input

    if (ch==KB_ESC) {
      s[1]=0;
      s[2]=0;
      return &s[2]; 
    }

    
    if (ch==KB_BACK)
    {
      
        if (len>0) 
        {
            len--;
            //delete char and go back (if some chars left)
            _putch(KB_BACK);
            _putch(' '); 
            _putch(KB_BACK);
           
        }
         
         continue;
    }

    if (ch>0x80 || ch <' ') //skip functions keys
        continue;
  
    _putch((char)0xff&ch); //print back to screen
    s[len+2]=(char)0xff&ch;
    len++;
  }
  
  s[1]=len;
  s[len+2]=0;

  return &s[2]; 
}