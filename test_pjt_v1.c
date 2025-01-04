/*
 * File:   test_project_eeprom_v2.c
 * Author: Hp
 *
 * Created on 16 December, 2024, 10:29 AM
 * added eeprom to the pjt.
 */


#define F_CPU 1000000UL//unsigned long frequency range
#include <avr/io.h>//input output 
#include <util/delay.h> // delay func
#include <stdio.h>
#include <avr/eeprom.h>

// Macros for controlling RS and Enable pins
#define RS 0 // RS connected to B0
#define EN 1 // Enable connected to B1

void cmd(char a)
{
    // Send higher nibble of the command
    PORTB = (PORTB & 0x0F) | (a & 0xF0);
    PORTB &= ~(1 << RS); // RS = 0 (command mode)
    PORTB |= (1 << EN);  // Enable = 1
    _delay_ms(5);
    PORTB &= ~(1 << EN); // Enable = 0

    // Send lower nibble of the command
    PORTB = (PORTB & 0x0F) | (a << 4);
    PORTB &= ~(1 << RS); // RS = 0 (command mode)
    PORTB |= (1 << EN);  // Enable = 1
    _delay_ms(5);
    PORTB &= ~(1 << EN); // Enable = 0
}

void data(char b)
{
    // Send higher nibble of the data
    PORTB = (PORTB & 0x0F) | (b & 0xF0);
    PORTB |= (1 << RS);  // RS = 1 (data mode)
    PORTB |= (1 << EN);  // Enable = 1
    _delay_ms(5);
    PORTB &= ~(1 << EN); // Enable = 0

    // Send lower nibble of the data
    PORTB = (PORTB & 0x0F) | (b << 4);
    PORTB |= (1 << RS);  // RS = 1 (data mode)
    PORTB |= (1 << EN);  // Enable = 1
    _delay_ms(5);
    PORTB &= ~(1 << EN); // Enable = 0
}

void display(const char *p)
{

    while ((*p != 0)) {
        data(*p);
        p++;
    
    }
}


 
uint32_t map_adc_to_freq(uint16_t adc_value,uint16_t max,uint16_t min) {

    return 50000 - ((uint32_t)(adc_value - min) * (50000 - 20000) / (max - min));

}



void set_pwm_freq(uint32_t freq) {
    uint16_t prescaler_bits = 0;
    uint16_t prescaler = 1;

//    // Select appropriate prescaler
//    if (freq >= 20000) {  // Prescaler 1
//        prescaler = 1;
//        prescaler_bits = (1 << CS10);
//    } else if (freq >= 3125) {  // Prescaler 8
//        prescaler = 8;
//        prescaler_bits = (1 << CS11);
//    } else if (freq >= 390) {  // Prescaler 64
//        prescaler = 64;
//        prescaler_bits = (1 << CS11) | (1 << CS10);
//    } else if (freq >= 98) {  // Prescaler 256
//        prescaler = 256;
//        prescaler_bits = (1 << CS12);
//    } else {  // Prescaler 1024
//        prescaler = 1024;
//        prescaler_bits = (1 << CS12) | (1 << CS10);
//    }

    // Calculate ICR1
    uint16_t icr = (uint16_t)((F_CPU / (freq * prescaler)) - 1);

    // Configure Timer1 for Fast PWM mode
    TCCR1A = (1 << WGM11)|(1 << COM1B1)|(1 << COM1B0);                // Fast PWM mode, non-inverted
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = icr;                           // Set TOP value for desired frequency
    OCR1B =(uint16_t) icr *.5;                      // 50% duty cycle
}
 
int main(void) {
    uint16_t  eeprom_address_MIN = 1;  
    uint16_t  eeprom_address_MAX = 4;
    uint16_t a;
    uint16_t max_val = 31; // by default.
    uint16_t min_val = 3; // by default.
    min_val = eeprom_read_word(&eeprom_address_MIN);
    max_val = eeprom_read_word(&eeprom_address_MAX);
    
    uint8_t max_flag=0;
    uint8_t min_flag=0;
    
    uint32_t freq;
    char buffer[150];
    

    DDRD=0x00;
    DDRB = 0xFF; // Set PORTB as output
    DDRE |= (1 << PE1);// set pwm pin timer 0
    DDRC|=(1<<PC1); // set pwm pin timer 1
    

    cmd(0x02); // Initialize in 4-bit mode
    cmd(0x28); // 4-bit mode, 2-line display
    cmd(0x01); // Clear display
    cmd(0x06); // Increment cursor
    cmd(0x0E); // Display ON, Cursor ON

    

    while(1){
        
//    // ADC Initialization with internal opam
//    ADMUX = 0xCF; //select Internal 2.56V reference voltage with external capacitorq connected on the AREF pin and AMP1 
//    AMP1CSR=0b10110000;//40 gain
//    ADCSRA = 0x84; // Enable ADC and set prescaler to 16
//    ADCSRB=0x20;
//        
    // ADC Initialization with external opam
    ADMUX = 0xC2;  
    ADCSRA = 0x87; // Enable ADC and set prescaler to 128 
    ADCSRB=0x20;
    
        ADCSRA |= (1 << ADSC); 
        while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete

        while((ADCSRA&0x40));
        a=ADC;
       
      
           
               if(((PIND&(0x80))==0)) {
                   
                   
                    ADCSRA=0XC0;
                    while((ADCSRA&0x40)==0);
                    a=ADCL;
                    a=a+(ADCH<<8);
                    cmd(0x01);
                    sprintf(buffer, "-Ve min=%d", a);
                    display(buffer);
                 
                    eeprom_update_word(&eeprom_address_MIN,a);
                    min_val = eeprom_read_word(&eeprom_address_MIN);
                    
                   
               }// -ve
               
               if((PIND&(0x40))==0) 
               {    
                    cmd(0x01);
                    sprintf(buffer, "+Ve max=%d", a );
                    display(buffer);
                    eeprom_update_word(&eeprom_address_MAX,a);
                    max_val = eeprom_read_word(&eeprom_address_MAX);
                   
               }// +ve
//                  min_val = a;
                if(a>=min_val && a<=max_val)
                  freq = map_adc_to_freq(a,max_val,min_val);

        // Set frequency
        
       set_pwm_freq(freq);
        
        
        // Send ADC and frequency via UART
//       
//        cmd(0x80);
//        sprintf(buffer, "ADC=% 4u F% 5lu Hz", a, freq);
//        display(buffer);
//        cmd(0xC0);
//        sprintf(buffer, "max% 4u min% 4u",max_val,min_val);
//        display(buffer);  
////      
       _delay_ms(10);
       
  
   
       
       
       
       
    }
    
}
