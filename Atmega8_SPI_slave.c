//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "lcd.c"
#include "adc.c"

#include "onewire.c"
#include "ds18x20.c"
#include "crc8.c"

#include "conio.h"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

#define PROGRAMM_DS	0
#define GRUPPE_DS		0xC0
//#define GRUPPE_DS	0xB0

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB
#define SPI_MOSI     3  
#define SPI_MISO     4
#define SPI_SCK      5

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4

#define TASTE1		19
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		223
#define TASTE0		236
#define TASTER		248
#define TASTATURPORT PORTC
#define TASTATURPIN		3

#define MANUELL_PORT		PORTD
#define MANUELL_DDR		DDRD
#define MANUELL_PIN		PIND

#define MANUELL			7	// Bit 7 von Status 
#define MANUELLPIN		6	// Pin 6 von PORT D fuer Anzeige Manuell
#define MANUELLNEU		7	// Pin 7 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s


#define FOSC 1000000    /* oscillator-frequency in Hz */
#define BAUD 57600  //valid values:9600, 19200, 57600 kbits








volatile uint8_t					Programmstatus=0x00;
	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Countr fuer Timeout	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;

volatile uint8_t data;
volatile uint8_t lastdata;

volatile uint8_t spicount=0;
size_t inpos=0;
volatile uint8_t outpos=0;
volatile uint8_t textpos=0;


//volatile char text[] = {'A','B','C','D','E','F','G','H'};
volatile char* text = "* Slave *";

//#define MAXSENSORS 5
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;
static uint8_t gScratchPad[9];
static volatile uint8_t sensornummer=0;
// Code 1_wire start
//uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];


uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	
	ow_reset();
	
	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) 
	{
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("No Sensor found\0" );
			break;
		}
		
		if( diff == OW_DATA_ERR ) 
		{
			lcd_gotoxy(0,1);
			lcd_puts("Bus Error\0" );
			break;
		}
		lcd_gotoxy(4,1);

		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			{
				//lcd_gotoxy(15,1);
				//lcd_puthex(id[i]);

			gSensorIDs[nSensors][i] = id[i];
			//delay_ms(100);
			}
			
		nSensors++;
	}
	
	return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void){
        gTemp_measurementstatus=0;
        if ( DS18X20_start_meas(NULL) != DS18X20_OK) 
		  {
                gTemp_measurementstatus=1;
        }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
        uint8_t i;
        uint8_t subzero, cel, cel_frac_bits;
        for ( i=0; i<gNsensors; i++ ) 
		  {
			  
			  if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
											 &cel, &cel_frac_bits) == DS18X20_OK ) 
			  {
				  gTempdata[i]=cel*10;
				  gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
				  if (subzero)
				  {
					  gTempdata[i]=-gTempdata[i];
				  }
			  }
			  else
			  {
				  gTempdata[i]=0;
			  }
        }
}

uint8_t Sensornummerlesen(uint8_t index, uint8_t* nummer)
{
	uint8_t tempScratchPad[9];
	if  ((DS18X20_read_scratchpad(&gSensorIDs[index][0], tempScratchPad ))== DS18X20_OK)
	{
			lcd_gotoxy(14,1);
		lcd_puts("GUT\0");

		*nummer=tempScratchPad[2];
		return DS18X20_OK;
	}
	
	else 
	{
				lcd_gotoxy(14,1);
		lcd_puts("BAD\0");

		*nummer=0xFF;
		return DS18X20_ERROR;
	}
}

uint8_t Sensornummer(uint8_t index)
{
	uint8_t tempScratchPad[9];
	if  ((DS18X20_read_scratchpad(&gSensorIDs[index][0], tempScratchPad ))== DS18X20_OK)
	{
		lcd_gotoxy(14,1);
		lcd_puts("GUT\0");
		return tempScratchPad[2]; // Byte 2: Nummer des Sensors
	}
	
	else 
	{
			lcd_gotoxy(14,1);
		lcd_puts("BAD\0");

		return 0xFF;
	}
}



// Code 1_wire end

void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
//lcd_gotoxy(0,1);
//lcd_putint(Tastaturwert);
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}


void slaveinit(void)
{
//	MANUELL_DDR |= (1<<MANUELLPIN);		//Pin 5 von PORT D als Ausgang fuer Manuell
//	MANUELL_PORT &= ~(1<<MANUELLPIN);
 	//DDRD |= (1<<CONTROL_A);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	DDRD |= (1<<7);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	LOOPLED_DDR |= (1<<LOOPLED_PIN);
	//PORTD &= ~(1<<CONTROL_B);
	PORTD |= (1<<7);

	DDRB &= ~(1<<PORTB0);	//Bit 2 von PORT B als Eingang fuer Taster
	PORTB |= (1<<PORTB0);	//HI
	
//	DDRB |= (1<<PORTB2);	//Bit 2 von PORT B als Ausgang fuer PWM
//	PORTB &= ~(1<<PORTB2);	//LO

	DDRB |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang fuer PWM
	PORTB &= ~(1<<PORTB1);	//LO
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
//	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang für SDA
//	TWI_PORT |= (1<<SDAPIN); // HI
	
//	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang für SCL
//	TWI_PORT |= (1<<SCLPIN); // HI


	
//	DDRC &= ~(1<<PORTC0);	//Pin 0 von PORT C als Eingang fuer Vorlauf
//	PORTC |= (1<<DDC0); //Pull-up
//	DDRC &= ~(1<<PORTC1);	//Pin 1 von PORT C als Eingang fuer Ruecklauf
//	PORTC |= (1<<DDC1); //Pull-up
//	DDRC &= ~(1<<PORTC2);	//Pin 2 von PORT C als Eingang fuer Aussen
//	PORTC |= (1<<DDC2); //Pull-up
//	DDRC &= ~(1<<PORTC3);	//Pin 3 von PORT C als Eingang fuer Tastatur
	//PORTC &= ~(1<<DDC3); //Pull-up
	
}

char SPI_get_put_char(int cData)
{
   
   //Putchar -- Master
   /* Start transmission */
   SPDR = cData;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
      ;
   /* Return data register */
   return SPDR;
}

void SPI_Init(void)
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   
   /*
   //Master init
   // Set MOSI and SCK output, all others input 
   SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK);
   // Enable SPI, Master, set clock rate fck/16 
   SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
   */
  
    // Slave init
   unsigned char status;
   DDRB &= ~((1<<2)|(1<<3)|(1<<5));
   DDRB |= (1<<4);
   PORTB |= (1<<2)|(1<<3)|(1<<5);
   SPCR &= ~(1<<MSTR);
   
   SPCR |= (1<<SPR0)|(1<<SPR1);
   SPCR |= (1<<SPE);
   SPCR &= ~(1<<SPIE);
   status = SPSR;
   /*
   SPCR &= ~(1<<MSTR);
   SPCR &= ~(1<<2);
    // Set MISO output, all others input
    SPI_DDR = (1<<SPI_MISO);
   
   SPCR |= (1<<SPR0)|(1<<SPR1);
    // Enable SPI 
    SPCR = (1<<SPE);
  */
   
}


void _putch_u (char ch)
{
   
	while(PINB & (1<<PORT7)); //handshaking
   
#ifdef USR
	while(!(USR & (1<<UDRE))); //transmit buffer is ready to receive data
	
	UDR = ch;    // send character
	while(!(USR & (1<<TXC))); //wait for char to be send
   
	USR &= ~(1<<TXC || 1<<UDRE);
#else //2 UART MCU
	while(!(UCSRA & (1<<UDRE))); //transmit buffer is ready to receivce data
	
	UDR = ch;    // send character
	while(!(UCSRA & (1<<TXC))); //wait for char to be send
   
	UCSRA &= ~(1<<TXC || 1<<UDRE);
   
#endif
	
}

ISR (SPI_STC_vect)
{
   data = SPDR;
   //SPDR = data & 0x07;
   SPDR = text[textpos & 0x07];
   //SPDR = 'B';
   spicount++;
   //PORTD = data;
   
}


int main (void)
{
	/* INITIALIZE */
//	LCD_DDR |=(1<<LCD_RSDS_PIN);
//	LCD_DDR |=(1<<LCD_ENABLE_PIN);
//	LCD_DDR |=(1<<LCD_CLOCK_PIN);
	
	slaveinit();
	
   //SPI_Init();
   
   
   
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
   delay_ms(1000);
	lcd_cls();
   lcd_gotoxy(0,0);
	lcd_puts("SLAVE\0");
	
   // http://www.nerdkits.com/forum/thread/2317/
   //DDRC|=(1<<PC5);// pin c5 as output
   DDRB|=(1<<PB4);
   DDRB &= ~(1<<PB5);
   DDRB &= ~(1<<PB3);
   DDRB &= ~(1<<PB2);
   SPCR |= (1<<SPE) | (1<<SPIE);// enable SPI, enable spi interputs
   SPCR |= (1<<SPR0)|(1<<SPR1);
   //PORTB |= (1<<PB4);
   
	// DS1820 init-stuff begin
	// DS1820 init-stuff end
#pragma mark while
   sei();
   lcd_gotoxy(0,1);
	while (1) 
	{
		loopCount0 ++;
		//_delay_ms(2);
      
		if (loopCount0 >=0x00FF)
		{
						
			
			LOOPLED_PORT ^= (1<<LOOPLED_PIN);
			loopCount1++;
         

         //lcd_gotoxy(12,0);
         //lcd_puthex(loopCount1);
         
         //lcd_gotoxy(12,0);
         //lcd_puthex(spicount);
         /*
         data = SPDR;
         SPDR = 5;
         spicount++;
         PORTD = data;
*/
         
         //lcd_gotoxy(18,0);
         if (!(data==lastdata))
         {
            lastdata = data;
            lcd_gotoxy(inpos,0);
            lcd_putc(lastdata);
            inpos ++;
            if (inpos >= 19)
            {
               inpos = 0;
            }
            
            
            lcd_gotoxy(outpos,1);
            lcd_putc(text[textpos & 0x07]);
            textpos++;
            outpos ++;
            if (outpos >= 19)
            {
               outpos = 0;
            }
           
            
         }

         
         
        // lcd_gotoxy(12,0);
        // lcd_putc(text[spicount & 0x07]);
         //lcd_puthex(SPDR);
			//SPDR = 2;
			if ((loopCount1 >0x000F) )//&& (!(Programmstatus & (1<<MANUELL))))
			{
            //LOOPLED_PORT ^= (1<<LOOPLED_PIN);
				
            //LOOPLED_PORT ^= (1<<LOOPLED_PIN);
				
            
				loopCount1=0;
            
            //char incoming = SPI_get_put_char('A');
            
            
            
            
				// DS1820 loop-stuff begin
            /*
				start_temp_meas();
				delay_ms(800);
				read_temp_meas();
				
				//Sensor 1
				lcd_gotoxy(0,1);
				lcd_puts("1:\0");
				if (gTempdata[0]/10>=100)
				{
					lcd_gotoxy(1,1);
					lcd_putint((gTempdata[0]/10));
				}
				else
				{
					lcd_gotoxy(2,1);
					lcd_putint2((gTempdata[0]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[0]%10);
				
				// Sensor 2
				lcd_gotoxy(7,1);
				lcd_puts("2:\0");
				if (gTempdata[1]/10>=100)
				{
					lcd_gotoxy(8,1);
					lcd_putint((gTempdata[1]/10));
				}
				else
				{
					lcd_gotoxy(9,1);
					lcd_putint2((gTempdata[1]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[1]%10);
				
				// Sensor 3
				lcd_gotoxy(14,1);
				lcd_puts("3:\0");
				if (gTempdata[2]/10>=100)
				{
					lcd_gotoxy(15,1);
					lcd_putint((gTempdata[2]/10));
				}
				else
				{
					lcd_gotoxy(16,1);
					lcd_putint2((gTempdata[2]/10));
				}
				
				lcd_putc('.');
				lcd_putint1(gTempdata[2]%10);
				
				
				
				lcd_gotoxy(15,0);
				lcd_puts("   \0");
				lcd_gotoxy(15,0);
				lcd_puthex(gTemp_measurementstatus);

				*/
				
				// DS1820 loop-stuff end
				
				
				//lcd_putint(gTempdata[1]);
				//lcd_putint(gTempdata[2]);
				//delay_ms(1000);
			}
			
			loopCount0 =0;
		}
      
      
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P0 Down\0");
			
			if (! (TastenStatus & (1<<PB0))) //Taste 0 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PB0);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P0 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					Tastencount=0;
					TastenStatus &= ~(1<<PB0);
				}
			}//	else
			
		} // Taste 0

		
#pragma mark Tastatur 
		/* ******************** */
		continue;
      initADC(TASTATURPIN);
		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
//		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				
				 
				 //lcd_gotoxy(17,1);
				 //lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	
					{ 
					if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[0][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
						Manuellcounter=0;
						
						}
					}break;
						
					case 2://
					{ 
					
						if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[1][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
						Manuellcounter=0;
						
						
						}
						
					}break;
						
					case 3: //	Uhr aus
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gSensorIDs[2][i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}
						Manuellcounter=0;
						

						}
					}break;
						
					case 4://
					{ 
						DS18X20_read_scratchpad(&gSensorIDs[0][0], gScratchPad );
												uint8_t i=0;
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('0');
						lcd_putc(' ');
						for (i=0;i<OW_ROMCODE_SIZE;i++)
						{
						lcd_puthex(gScratchPad[i]);
						if (i==3)
						{
						lcd_gotoxy(0,1);
						}
						lcd_putc(' ');
						}

					}break;
						
					case 5://
					{ 
						Programmstatus |= (1<<MANUELL);	// MANUELL ON
						Manuellcounter=0;
						MANUELL_PORT |= (1<<MANUELLPIN);
						Programmstatus |= (1<<MANUELLNEU);
						lcd_clr_line(1);
						/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
						*/
					}break;
						
					case 6://
					{ 
					sensornummer=0xAF;
					Sensornummerlesen(0,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					
					}break;
						
					case 7:// Schalter rückwaerts
					{ 
					sensornummer=0x00;
					Sensornummerlesen(0,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('0');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					
					}break;
						
					case 8://
					{ 
					sensornummer=0x00;
					Sensornummerlesen(1,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('1');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					

					}break;
						
					case 9:// Schalter vorwaerts
					{ 
					sensornummer=0x00;
					Sensornummerlesen(2,&sensornummer);
						lcd_gotoxy(0,0);
						lcd_puts("Sens\0");
						lcd_putc('2');
						lcd_putc(' ');
						lcd_puthex(sensornummer);
					
					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
						Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
						Programmstatus &= ~(1<<MANUELLNEU);
						MANUELL_PORT &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
	}
	
	
	return 0;
}
