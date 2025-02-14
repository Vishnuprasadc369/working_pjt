/*
 * File:   test_pjt_v2.c
 * Author: Hp
 *
 * Created on 14 February, 2025, 2:21 PM
 */

#define F_CPU 8000000UL//unsigned long frequency range
#include <avr/io.h>//input output 
#include <util/delay.h> // delay func
#include <stdio.h>
#include <avr/eeprom.h>
// Macros for controlling RS and Enable pins
#define RS 0 // RS connected to B0
#define EN 1 // Enable connected to B1

// Initialize USART with LINBTR-based baud rate calculation
void USART_Init() {
    // Read LINBTR register and apply masking
    DDRD = DDRD | 0x08;
    //LINCR=0X80;
    //while((LINCR&(0X80)));
    LINDLR=0x00; 
    LINBRRL = 25 ;        // Set low byte of UBRR 25 for 8mhz 51 for 16mhz
    // Enable LIN/USART in UART mode
    LINCR = ((1 << LENA) | (1 << LCMD0) | (1 << LCMD2));

}

// Transmit a single character using LIN/USART
void USART_Transmit(char data) {
    LINDAT = data;                              // Load data into LIN data register
    while (!(LINSIR & (1 << LTXOK)));           // Wait for transmission to complete
    LINSIR |= (1 << LTXOK);                     // Clear the transmit complete flag
}
void display(const char *ptr){
    while(*ptr){
    USART_Transmit(*ptr);
    ptr++;
    }
            
}

uint32_t map_adc_to_freq(uint16_t adc_value,uint16_t max,uint16_t min) {
    return 50000 - ((uint32_t)(adc_value - min) * (50000 - 20000) / (max - min));
}


void set_pwm_freq_phase(uint32_t freq) {
    uint16_t prescaler = 1;
    uint16_t prescaler_bits = 0;

    // Select appropriate prescaler
    if (freq >= 3906) { // Prescaler = 1
        prescaler = 1;
        prescaler_bits = (1 << CS10);
    } else if (freq >= 488) { // Prescaler = 8
        prescaler = 8;
        prescaler_bits = (1 << CS11);
    } else if (freq >= 61) { // Prescaler = 64
        prescaler = 64;
        prescaler_bits = (1 << CS11) | (1 << CS10);
    } else if (freq >= 15) { // Prescaler = 256
        prescaler = 256;
        prescaler_bits = (1 << CS12);
    } else { // Prescaler = 1024
        prescaler = 1024;
        prescaler_bits = (1 << CS12) | (1 << CS10);
    }

    // Calculate ICR1
    uint16_t icr = (uint16_t)((F_CPU) / (2 * prescaler * freq));

    // Configure Timer1 for Phase-Correct PWM
    TCCR1A = (1 << WGM11) | (1 << COM1B1);       // Phase-correct PWM, clear OC1B on compare match
    TCCR1B = (1 << WGM13) | prescaler_bits;      // Phase-correct PWM, selected prescaler
    ICR1 = icr;                                  // Set TOP value for desired frequency
    OCR1B = icr / 2;                             // 50% duty cycle
}
int main(void) {
    uint16_t  eeprom_address_MIN = 1;  
    uint16_t  eeprom_address_MAX = 4;
    uint16_t  eeprom_address_threshold = 8;
    uint16_t a=0;
    uint16_t b=0;
    uint16_t max_val = 31; // by default.
    uint16_t min_val = 3; // by default.
    uint16_t threshold = 920; // by default.
    
    min_val = eeprom_read_word(&eeprom_address_MIN);
    max_val = eeprom_read_word(&eeprom_address_MAX);
    threshold = eeprom_read_word(&eeprom_address_threshold);
    
    uint8_t max_flag=0;
    uint8_t min_flag=0;
    uint32_t sum = 0;
    uint32_t freq;
    char buffer[150];
     // Configure Timer1 for Fast PWM mode
    TCCR1A = (1 << WGM11)|(1 << COM1B1)|(1 << COM1B0);        // Prescaler 1        // Fast PWM mode, non-inverted
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
   
    DDRD=0x00;
    
    DDRE |= (1 << PE1);// set pwm pin timer 0
    DDRC|=(1<<PC1); // set pwm pin timer 1
    DDRC&= ~(1 << PC3); 
  
    USART_Init();

    while(1){
        
//    // ADC Initialization with internal opam
//    ADMUX = 0xCF; //select Internal 2.56V reference voltage with external capacitorq connected on the AREF pin and AMP1 
//    AMP1CSR=0b10110000;//40 gain
//    ADCSRA = 0x84; // Enable ADC and set prescaler to 16
//    ADCSRB=0x20;
//        
    // ADC Initialization with external opam
    ADMUX = 0x42;  
    ADCSRA = 0x87; // Enable ADC and set prescaler to 128 
    ADCSRB=0x00;
    sum=0;
    for(int i=1;i<100;i++){
        ADCSRA |= (1 << ADSC); //01
      //  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
        while((ADCSRA&0x40));
        a=ADC;
        sum+=a;  
    }
        a = sum / 100; // Compute average
//        
//        
//        
//             ADCSRA |= (1 << ADSC); //01
//      //  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
//        while((ADCSRA&0x40));
//        a=ADC;
        
        
               if(((PIND&(0x80))==0)) {

                    
                    sprintf(buffer, "-Ve min=%d   \r\n", a);
                    display(buffer);
                    eeprom_update_word(&eeprom_address_MIN,a);
                    min_val = eeprom_read_word(&eeprom_address_MIN);
                   
               }// -ve
               
               if((PIND&(0x40))==0) 
               {    
                    
                    sprintf(buffer, "+Ve max=%d   \r\n", a );
                    display(buffer);
                    eeprom_update_word(&eeprom_address_MAX,a);
                    max_val = eeprom_read_word(&eeprom_address_MAX);
                   
               }// +ve
//                  min_val = a;
                if(a>=min_val && a<=max_val)
                  freq = map_adc_to_freq(a,max_val,min_val);

        // Set frequency
        
       set_pwm_freq_phase(freq);
       
       
 ///////////////////ad10 reading for bjt
       
       
    ADMUX = 0xCA;  
    ADCSRA = 0x87; // Enable ADC and set prescaler to 128 
    ADCSRB=0x20;

        ADCSRA |= (1 << ADSC); //01
      //  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
       sum=0;
    for(int i=1;i<100;i++){
        ADCSRA |= (1 << ADSC); //01
      //  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
        while((ADCSRA&0x40));
        b=ADC;
        sum+=b;  
    }
        b = sum / 100;
        
         if((PINC&(1<<PC3))==0x08)
               {    
                
                    sprintf(buffer, "threshold max=%d   \r\n", b);
                    display(buffer);
                    eeprom_update_word(&eeprom_address_threshold,b);
                    threshold = eeprom_read_word(&eeprom_address_threshold);
                   
               }// +ve
        
        if(b>=threshold){
            PORTB=(1<<PB2);
            
        }else{
            PORTB &= ~(1 << PB2); // Clear PB2
        }
   
        
        // Send ADC and frequency via UART
       
        
        sprintf(buffer, "ADC10=% 4u ADC2=% 4u \r\n", b,a);
        display(buffer);

//        sprintf(buffer, "max% 4u min% 4u \r\n",max_val,min_val);
//        display(buffer);  
//      
   //    _delay_ms(10);
         
       
    }
    
}
