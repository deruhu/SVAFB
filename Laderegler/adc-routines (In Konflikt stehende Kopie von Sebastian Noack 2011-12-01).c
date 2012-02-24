#include <avr/io.h>

/* ADC initialisieren */
void ADC_Init(void) {
 
  int result;
 
//  ADMUX = (0<<REFS1) | (1<<REFS0);      // AVcc als Referenz benutzen
  ADMUX = (1<<REFS1) | (1<<REFS0);      // interne Referenzspannung nutzen
  ADCSRA =(1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);     // Frequenzvorteiler: 64
  ADCSRA |= (1<<ADEN);                  // ADC aktivieren
 
  /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
 
  ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung 
  while (ADCSRA & (1<<ADSC) ) {}        // auf Abschluss der Konvertierung warten
  /* ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten
     Wandlung nicht übernommen. */
  result = ADCW;
}
 
 void ADC_enable(void) {
 ADCSRA |= (1<<ADEN);                  // ADC aktivieren
 }
 
 void ADC_disable(void) {
 ADCSRA &=~(1<<ADEN);                  // ADC deaktivieren
 }
    
/* ADC Einzelmessung */
int ADC_Read( char channel )
{
  // Kanal waehlen, ohne andere Bits zu beeinflußen
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
  while (ADCSRA & (1<<ADSC) ) {}  // auf Abschluss der Konvertierung warten
  return ADCW;                    // ADC auslesen und zurückgeben
}
 
/* //ADC Mehrfachmessung mit Mittelwertbbildung 
int ADC_Read_Avg( char channel, char average )
{
  long result = 0;
 
  for (char i = 0; i < average; ++i )
    result += ADC_Read( channel );
 
  return (int)( result / average );
}
*/