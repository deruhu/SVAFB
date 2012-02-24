/*
 * Laderegler.c
 *
 * Created: 06.08.2011 13:25:51
 *  Author: Weidenhocker
 *
 *	Status: funktioniert nicht im Moment...  musss dringend noch überarbeitet werden 
 *			Hardware: funktioniert nicht im Moment
 */ 

#define F_CPU 12000000

#include <avr/io.h>
#include <stdio.h>
#include <avr/iom8.h>
#include "lcd-routines.h"
#include "adc-routines.h"
//#include "adc-routines.c"
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h> 
#include <inttypes.h>
 
uint32_t Sol_PWR(void);
void PPT(void);
void PPT_p(uint32_t neu);
void PPT_m(uint32_t neu);
void TE_Schutz(void);
uint8_t Display (uint8_t, uint8_t);

#define PWM_PWR	OCR2
#define Bat_niedrig	250

volatile char tCounter=0,GIAF=0;	//GIAF= "generelles Interrupt aktivierungsFlag"
char tCounterb=0;

int main(void)
{
	
	int tempval;
	uint8_t Disp_Mode=0,Disp_Stat=1;

//	Start Initialisierung

  DDRB = DDRB & ~(1<<PB2) | (1<<PB3) & ~(1<<PB1) | (1<<PB0) | (1<<PB4) | (1<<PB5);
  DDRC |= (1<<PC5) | (1<<PC4);
  DDRD = DDRD & ~(1<<PD6) & ~(1<<PD7) ;	
	ADC_Init();
	lcd_init();
	_delay_ms(50);
 	
	lcd_command(0b00001111);	//cursor setzen
	lcd_string("Test");

/*
 // OC1A/B auf Ausgang
  //ATMega8
  //
  // Timer 1 einstellen (OC1A: Pin14; OC1B: Pin15)
  //  
  // Modus 5 (Fast-PWM, 8 bit):
  //
  //    WGM13    WGM12   WGM11    WGM10
  //      0        1       0        1
  //
  //    Timer Vorteiler: 1
  //     CS12     CS11    CS10
  //       0        0       1
  //
  //  Steuerung des Ausgangsport: Set at BOTTOM, Clear at match
  //     COM1A1   COM1A0	COM1B1   COM1B0
  //       1        0		  1	       0
 
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
  TCCR1B = (1<<WGM12) | (1<<CS10);
    */
  
  //
  //  Jetzt nochmal für den Timer2 (OC2: Pin16)
  //
  // Modus 3 (Fast-PWM, 8 bit):
  //
  //    WGM21    WGM20 
  //      1        1   
  //
  //    Timer Vorteiler: 1
  //     CS22     CS21    CS20
  //       0        0       1
  //
  //  Steuerung des Ausgangsport: Set at BOTTOM, Clear at match
  //     COM21   COM20
  //       1        0	
 
  TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<CS20) | (1<<COM21);
  
  // der Compare Wert OCRnx
  // Wenn der Zähler diesen Wert erreicht, wird mit
  // obiger Konfiguration der OC1A Ausgang abgeschaltet
  // Sobald der Zähler wieder bei 0 startet, wird der
  // Ausgang wieder auf 1 gesetzt
  //
  // Durch Verändern dieses Wertes, werden die unterschiedlichen
  // PWM Werte eingestellt.

	
		
  // Timer 0 konfigurieren
  TCCR0 = (1<<CS00)|(1<<CS02); // Prescaler 8

  // Overflow Interrupt erlauben
  TIMSK |= (1<<TOIE0);
  PWM_PWR = 200;
 	
  
	sei();


//	Ende Initialisierung

	
	
	
//	Start Programm
	
	
	
		
	//uint16_t i=0;
	uint8_t Te=0,Tz=0,LStat=0;
	PORTB&= ~(1<<PB0);
	
	
	while(1)
    {
		
	TE_Schutz();
	
	if (ADC_Read(0)>ADC_Read(2)) //Tag
	{	
		if ((PINB&(1<<PB0))==0) //Shutdown ausmachen
		{
			PORTB|= (1<<PB0);
		}
		
				if (LStat!=0)	//Lampen aus
		{
			PORTB=PORTB & ~(1<<PB4) & ~(1<<PB5);
			LStat=0;
		}
		
		PPT();
		
		if ((tCounter==45)&&(GIAF&(1<<0)))	
		{
			Disp_Stat=Display(Disp_Mode,Disp_Stat);
			GIAF&=~(1<<0);
		}		
	}
	
	else if (ADC_Read(2)>ADC_Read(0)) //Nacht
	{	
		if ((PINB&~(1<<PB0))!=0)
		{
			PORTB&= ~(1<<PB0);
		}
		
		if ((PINC&(1<<PC4))!=0) //Lüfter aus
		{
			PORTC&= ~(1<<PC4);
		}
		if ((Te==1)&&(PIND6==0))
		{	Te=0;
		}

		if ((Tz==1)&&(PIND7==0))
		{	Tz=0;
		}
		
		if ((Te==0)&&(PIND6!=0))	//Schalter gedrückt, schöner wäre über interrupt, aber PD2 und PD3 hat das Display
		{
			Te=1;
			if ((LStat&(1<<0))==0)
			{
				LStat|=(1<<0);
				PORTB|=(1<<PB4);
			}
			else if ((LStat&(1<<0))!=0)
			{
				LStat&=~(1<<0);
				PORTB&=~(1<<PB4);
			} 
		}
		
		if ((Tz==0)&&(PIND7!=0))	//Schalter gedrückt, schöner wäre über interrupt, aber PD2 und PD3 hat das Display
		{
			Tz=1;
			if ((LStat&(1<<1))==0)
			{
				LStat|=(1<<1);
				PORTB|=(1<<PB5);
			}
			else if ((LStat&(1<<1))!=0)
			{
				LStat&=~(1<<1);
				PORTB&=~(1<<PB5);
			} 
		}
		
		if ((tCounter==45)&&(GIAF&(1<<0)))	
		{	Disp_Stat=Display(Disp_Mode,Disp_Stat);
			GIAF&=~(1<<0);
		}
	}
	

	}
}
uint8_t Display(uint8_t Mode, uint8_t Status){
	
uint16_t zeig;	
char Buffer[20];

if (PINB & (1<<PB2)) //Displaybeleuchtung an
			{
				if (!Status){
				lcd_command(LCD_SET_DISPLAY | LCD_DISPLAY_ON);
				Status=1;
				}
			
	switch (Mode){
		case 0:		lcd_setcursor(0,1);
					lcd_string( "U_Sol" );
				
					lcd_setcursor(9,1);
					lcd_string( "U_Bat" );
			
					zeig=ADC_Read(0)*25;
			
					lcd_setcursor(0,2);
					itoa( (zeig/1000), Buffer, 10 );
					lcd_string( Buffer );
					lcd_data(0x2c);
					itoa( (zeig-((zeig/1000)*1000)), Buffer, 10 );
					lcd_string( Buffer );

					zeig=ADC_Read(2)*25;
			
					lcd_setcursor(9,2);
					itoa( (zeig/1000), Buffer, 10 );
					lcd_string( Buffer );
					lcd_data(0x2c);
					itoa( (zeig-((zeig/1000)*1000)), Buffer, 10 );
					lcd_string( Buffer );
				break;
				}
			}			
									
else //Displaybeleuchtung aus
			{
				if (Status){
				lcd_command(LCD_SET_DISPLAY | LCD_DISPLAY_OFF);
				Status=0;
			}
			
			}
	return Status;
	}

/*
Der Overflow Interrupt Handler
wird aufgerufen, wenn TCNT0 von
255 auf 0 wechselt (256 Schritte),
d.h. ca. alle (21.85) ms
*/
ISR(TIMER0_OVF_vect)
{
  	if (tCounterb==460) //~10 sec
	{tCounterb=0;
	}
	tCounterb++;
	tCounter=tCounterb;
	GIAF|=(1<<0)|(1<<1);
	}

/*
	Bestimmung der Eingangsleistung in Form des 32-bit-Äquivalents.
	Die Eingangsvariablen (werden aus Analogwerten gelesen) haben ein 10 bit-Format.
	Diese werden einfach multipliziert, da keine zeit für Fließkommaberechnungen 
	verschwendet werden soll. 
*/
uint32_t Sol_PWR(void){
	uint16_t sol_u,sol_i;
	uint32_t PWR;
	
	sol_u=ADC_Read(0);
	sol_i=ADC_Read(1);
	PWR=sol_u*sol_i;
	return PWR;	
	}

/*
	Funktion zur Ermittlung des Spitzenleistungswertes.
	Wenn mehr Leistung umgesetzt werden kann, indem die PWM größer gewählt wird, dann
	wird der Punkt weiter oben, ansonsten weiter unten gesucht wenn der Spitzenwert bereits eingestellt ist,
	dann wird nichts getan. 
*/	
void PPT(void){
	uint32_t PWR_alt,PWR_p,PWR_m;
	
	PWR_alt=Sol_PWR();	
	
	/* Leistung bei nem größeren PWM-Schritt berechnen */ 
	if(PWM_PWR<=253) PWM_PWR++;	
	/*else {
		PWM_PWR=254;
		PPT_m();
		return;
		}
	*/
	_delay_ms(10);
	PWR_p=Sol_PWR();
	
	/* Leistung bei nem kleineren PWM-Schritt berechnen */
	if(PWM_PWR>=3) PWM_PWR-=2;
	/*else {
		PWM_PWR=1;
		PPT_p();
		return;
		}
	*/
	_delay_ms(10);
	PWR_m=Sol_PWR();
	
	if ((PWR_p>PWR_alt)&&(PWR_p>=PWR_m))
	{
		PPT_p(PWR_p);
	}
	else if ((PWR_m>PWR_alt)&&(PWR_m>PWR_p))
	{
		PPT_m(PWR_m);
	}
	else {
		PWM_PWR++;
		return;
	}		
}

void PPT_p(uint32_t neu){
	uint32_t alt=0;

	do 
	{	
		alt=neu;
		_delay_ms(20);
		if(PWM_PWR<=253) PWM_PWR++;	
		else {
		PWM_PWR=254;
		}
		neu=Sol_PWR();
	} while (neu>=alt);
	
		PWM_PWR--;
		return;	
	}	


void PPT_m(uint32_t neu){
	uint32_t alt;

	do 
	{	
		alt=neu;
		_delay_ms(20);
		if(PWM_PWR>=2) PWM_PWR--;	
		else {
		PWM_PWR=1;
		}
		neu=Sol_PWR();
	} while (neu>=alt);
	
		PWM_PWR++;
		return;	
	}	

void TE_Schutz(void){
	uint16_t Bat;
	
	Bat=ADC_Read(2);
	if (Bat <= Bat_niedrig) {
		PORTC &= ~(1<<PC5); //PC5=Tiefentladeshutz ausschalten
	}
	else if (Bat>=(Bat_niedrig+3)) {
		PORTD |= (1<<PC5); //PC5=Tiefentladeshutz einschalten
	}
}

void UL_Schutz(void){
	
	}