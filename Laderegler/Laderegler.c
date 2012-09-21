/*
 * Laderegler.c
 *
 * Created: 06.08.2011 13:25:51
 *  Author: Weidenhocker
 *
 *	Status: funktioniert nicht im Moment...  musss dringend noch �berarbeitet werden 
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

#ifdef Ladeschaltung
uint32_t sol_pwr(void);
void PPT(void);
void PPT_p(uint32_t neu);
void PPT_m(uint32_t neu);
uint8_t UL_Schutz(void);
#endif

void TE_Schutz(void);
uint8_t Display (uint8_t, uint8_t);


#define PWM_PWR		    OCR2
#define BAT_LOW		    250
#define BAT_HIGH	    0xFFFF
#define U_SOL		    ADC_Read(0)
#define	 U_BAT		    ADC_Read(2)
#define I_SOL		    ADC_Read(1)
#define TAG			    U_SOL>U_BAT
#define NACHT		    U_BAT>U_SOL
#define TEMP_HIGH      150
#define TEMP_THR       30

#define INT_DISPLAY    (1<<0)
#define INT_TIMER      (1<<1)
#define INT_SCHALTER   (1<<2)

#define LAMPE1  (1<<PB4)
#define LAMPE2  (1<<PB5)

volatile uint8_t GIAF=0, led_stat=0;	//GIAF= "generelles Interrupt aktivierungsFlag"
volatile uint16_t tCounter=0;

int main(void)
{

    uint8_t disp_mode=0,disp_stat=1;

    //	Start Initialisierung

    DDRB = DDRB & ~(1<<PB2) | (1<<PB3) & ~(1<<PB1) | (1<<PB0) | (1<<PB4) | (1<<PB5);
    DDRC |= (1<<PC5) | (1<<PC4);
    DDRD = DDRD & ~(1<<PD6) & ~(1<<PD7) ;
    ADC_Init();
    lcd_init();
    _delay_ms(50);

    lcd_command(0b00001111);	//cursor setzen
    lcd_string("Test");

#ifdef Ladeschaltung

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
    //  Jetzt nochmal f�r den Timer2 (OC2: Pin16)
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
    // Wenn der Z�hler diesen Wert erreicht, wird mit
    // obiger Konfiguration der OC1A Ausgang abgeschaltet
    // Sobald der Z�hler wieder bei 0 startet, wird der
    // Ausgang wieder auf 1 gesetzt
    //
    // Durch Ver�ndern dieses Wertes, werden die unterschiedlichen
    // PWM Werte eingestellt.
#endif


    // Timer 0 konfigurieren
    TCCR0 = (1<<CS00)|(1<<CS02); // Prescaler 8

    // Overflow Interrupt erlauben
    TIMSK |= (1<<TOIE0);

#ifdef Ladeschaltung
    PWM_PWR = 200;
#endif

    sei();


    //	Ende Initialisierung


    //	Start Programm




    //uint16_t i=0;
    uint8_t Te=0,Tz=0;
    PORTB&= ~(1<<PB0);


    while(1)
    {

        if(GIAF & INT_TIMER){

            TE_Schutz();


            if (TAG) //Tag
            {
                if (led_stat!=0)	//Lampen aus
                {
                    led_stat=led_stat & ~LAMPE1 & ~LAMPE2;
                    PORTB=PORTB & (led_stat | ~0x03);//~LAMPE1 & ~LAMPE2;
                }

#ifdef Ladeschaltung
                if (!UL_Schutz()) //Shutdown ausmachen
                {
#endif
                    if(PINB&(1<<PB0)==0)
                        PORTB|= (1<<PB0);


                    if (GIAF & INT_DISPLAY)
                    {
                        disp_stat=Display(disp_mode,disp_stat);
                        GIAF&=~(1<<0);
                    }
#ifdef Ladeschaltung
                    PPT();
                }

                else PORTB &= ~(1<<PB0);
#endif
            }

            else if (NACHT) //Nacht
            {
#ifdef Ladeschaltung
                if ((PINB&~(1<<PB0))!=0)
                {
                    PORTB&= ~(1<<PB0);
                }
#endif
                if ((PINC&(1<<PC4))!=0) //L�fter aus
                {
                    PORTC&= ~(1<<PC4);
                }

                switch (Te) {
                case 1:
                    if (PIND6==0)
                        Te=0;
                    break;

                case 0:
                    if (PIND6!=0)    //Schalter gedrückt, schöner wäre über interrupt, aber PD2 und PD3 hat das Display
                    {
                        Te=1;
                        if ((led_stat&(1<<0))==0)
                        {
                            led_stat|=(1<<0);
                            PORTB|=(1<<PB4);
                        }
                        else if ((led_stat&(1<<0))!=0)
                        {
                            led_stat&=~(1<<0);
                            PORTB&=~(1<<PB4);
                        }
                    }
                    break;
                default:
                    break;
                }

                switch (Tz) {
                case 1:                 //gedrückt
                    if (PIND7==0)       //losgelassen
                    {                   //Aktion bei losgelassen
                        Te=0;
                    }
                    else if (PIND7==1)  //gehalten
                    {                   //Aktion bei gehalten
                        Te=1;
                    }
                    break;

                case 0:
                    if (PIND7!=0)    //Schalter gedrückt, schöner wäre über interrupt, aber PD2 und PD3 hat das Display
                    {                //Aktion sofort nach dem Drücken
                        Te=1;
                        if ((led_stat&(1<<1))==0)
                        {
                            led_stat|=(1<<1);
                            PORTB|=(1<<PB5);
                        }
                        else if ((led_stat&(1<<1))!=0)
                        {
                            led_stat&=~(1<<1);
                            PORTB&=~(1<<PB5);
                        }
                    }
                    break;
                default:
                    break;
                }

                if ((tCounter==45)&&(GIAF&(1<<0)))
                {	disp_stat=Display(disp_mode,disp_stat);
                GIAF&=~(1<<0);
                }
            }

        }


    }
}
uint8_t Display(uint8_t modus, uint8_t status){

    uint16_t zeig;
    uint8_t leds=0;
    char Buffer[20];

    if (PINB & (1<<PB2)) //Displaybeleuchtung an
    {
        if (!status){
            lcd_command(LCD_SET_DISPLAY | LCD_DISPLAY_ON);
            status=1;
        }

        switch (modus){
        case 0:
            lcd_setcursor(0,1);
            lcd_string( "U_Sol" );

            lcd_setcursor(9,1);
            lcd_string( "U_Bat" );

            zeig=U_SOL*25;

            lcd_setcursor(0,2);
            itoa( (zeig/1000), Buffer, 10 );
            lcd_string( Buffer );
            lcd_data(0x2c);
            itoa( (zeig-((zeig/1000)*1000)), Buffer, 10 );
            lcd_string( Buffer );

            zeig=U_BAT*25;

            lcd_setcursor(9,2);
            itoa( (zeig/1000), Buffer, 10 );
            lcd_string( Buffer );
            lcd_data(0x2c);
            itoa( (zeig-((zeig/1000)*1000)), Buffer, 10 );
            lcd_string( Buffer );
            break;

        case 1:

            lcd_setcursor(0,1);
            lcd_string( "U_Bat" );

            zeig=U_BAT*25;

            lcd_setcursor(0,2);
            itoa( (zeig/1000), Buffer, 10 );
            lcd_string( Buffer );
            lcd_data(0x2c);
            itoa( (zeig-((zeig/1000)*1000)), Buffer, 10 );
            lcd_string( Buffer );


            lcd_setcursor(0,9);
            lcd_string ("Lampe");
            lcd_setcursor(9,1);

            leds=(led_stat & (LAMPE1 | LAMPE2));
            switch(leds)
            {
            case 0:
                lcd_string ("keine");
                break;

            case LAMPE1:
                lcd_string ("L1");
                break;

            case LAMPE2:
                lcd_string ("L2");
                break;

            case (LAMPE1|LAMPE2):
                lcd_string ("L1 & L2");
                break;

            default:
                break;
            }


            break;
        }
    }

    else //Displaybeleuchtung aus
    {
        if (status){
            lcd_command(LCD_SET_DISPLAY | LCD_DISPLAY_OFF);
            status=0;
        }

    }
    return status;
}

/*
Der Overflow Interrupt Handler
wird aufgerufen, wenn TCNT0 von
255 auf 0 wechselt (256 Schritte),
d.h. ca. alle (21.85) ms
 */
ISR(TIMER0_OVF_vect)
{
    uint8_t tCounterb=0;
    if (tCounterb==46) //~1 sec
    {tCounterb=0;
    }
    tCounterb++;
#ifdef Ladeschaltung
    tCounter=tCounterb;
#endif
    GIAF|=INT_TIMER;
}

#ifdef Ladeschaltung
/*
	Bestimmung der Eingangsleistung in Form des 32-bit-�quivalents.
	Die Eingangsvariablen (werden aus Analogwerten gelesen) haben ein 10 bit-Format.
	Diese werden einfach multipliziert, da keine zeit f�r Flie�kommaberechnungen 
	verschwendet werden soll. 
 */
uint32_t sol_pwr(void){
    uint32_t pwr;

    pwr=(uint32_t)U_SOL*(uint32_t)I_SOL;
    return pwr;
}
#endif

#ifdef Ladeschaltung
/*
	Funktion zur Ermittlung des Spitzenleistungswertes.
	Wenn mehr Leistung umgesetzt werden kann, indem die PWM gr��er gew�hlt wird, dann
	wird der Punkt weiter oben, ansonsten weiter unten gesucht wenn der Spitzenwert bereits eingestellt ist,
	dann wird nichts getan. 
 */
void PPT(void){
    uint32_t pwr_alt,pwr_p,pwr_m;

    pwr_alt=sol_pwr();

    /* Leistung bei nem gr��eren PWM-Schritt berechnen */
    if(PWM_PWR<=250) PWM_PWR+=4;
    /*else {
		PWM_PWR=254;
		PPT_m();
		return;
		}
     */
    _delay_ms(10);
    pwr_p=sol_pwr();

    if (pwr_p>pwr_alt)
    {
        PPT_p(pwr_p);
    }

    else
    {
        /* Leistung bei nem kleineren PWM-Schritt berechnen */
        if(PWM_PWR>=9) PWM_PWR-=8;
        /*else {
			PWM_PWR=1;
			PPT_p();
			return;
			}
         */
        _delay_ms(10);
        pwr_m=sol_pwr();

        if ((pwr_m>pwr_alt))
        {
            PPT_m(pwr_m);
        }
    }
    return;
}
#endif

#ifdef Ladeschaltung
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
        neu=sol_pwr();
    } while (neu>=alt);

    PWM_PWR--;
    return;
}
#endif

#ifdef Ladeschaltung
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
        neu=sol_pwr();
    } while (neu>=alt);

    PWM_PWR++;
    return;
}
#endif


void TE_Schutz(void){
    uint16_t u_bat;

    u_bat=U_BAT;
    if (u_bat <= BAT_LOW) {
        PORTC &= ~(1<<PC5); //PC5=Tiefentladeschutz ausschalten
    }
    else if (u_bat>=(BAT_LOW+3)) {
        PORTD |= (1<<PC5); //PC5=Tiefentladeschutz einschalten
    }
}


#ifdef Ladeschaltung
uint8_t UL_Schutz(void){

    if(U_BAT>=BAT_HIGH)
        return 1;

    return 0;
}
#endif
