/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
� Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 7/11/2023
Author  : 
Company : 
Comments: 


Chip type               : ATtiny13A
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Tiny
External RAM size       : 0
Data Stack size         : 16
*******************************************************/

#include <tiny13a.h>

// Declare your global variables here                  

#define TIM_FREQ2 // Timer freq 9600/1200/120kHz

#define TIME_STARTUP_MAX 1100L                   //  1000ms +10% max startup value
#define TIME_STARTUP_MIN 900L                   //  1000ms -10% min startup value
                            
#define TIME_ERROR_PULSE 2400L       //in us //error if the value is greater than zerro to out
#define TIME_ERROR_PAUSE 2250L       //60*9600/2^8 = 2250    // in timer period 60ms


#ifdef TIM_FREQ1
#define TIM_FREQ 9600L
#elif defined (TIM_FREQ2)
#define TIM_FREQ 1200L
#elif defined (TIM_FREQ3)
#define TIM_FREQ 150L
#endif


// char middle = 0;
unsigned int startup_imp_min = TIME_STARTUP_MAX;

struct timer_watch {
      unsigned char tcnt_t1;      // calc impulse front timer value
      unsigned char tcnt_t2;      // calc ipmulse decline timer value
      unsigned int timer;          // length impulse from timer overflow
      unsigned char exPin_st;      // status position extern pin input
      unsigned char en;             // (1) enable / (0) disable output
};

#define DOUBLE_PIN

struct timer_watch pin3 = {0,0,0,0,0};

#ifdef DOUBLE_PIN
struct timer_watch pin4 = {0,0,0,0,0};
#endif

#define tcc_out_en  TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00)
#define tcc_out_dis  TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00)

#ifdef DOUBLE_PIN
#define tcc_out_update  TCCR0A=(pin3.en<<COM0A1) | (0<<COM0A0) | (pin4.en<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00)       // with COM0B
#else
#define tcc_out_update  TCCR0A=(pin3.en<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00)
#endif

// inline 
void PinChanged(struct timer_watch * pin, unsigned char cnt, unsigned char in){

      // char err_fl = ((middle == 1) && (cnt <= 255/4)) ? 1 : 0;
      if (in == 0){                             // decline input impulse
            if (pin->exPin_st == 2) 
            {
                  // if (err_fl) pin->timer +=1;
                  pin->tcnt_t2 = cnt;   
                  pin->exPin_st = 3;
            }
      }
      else if (pin->exPin_st == 0)              //front input impulse
      {
            pin->tcnt_t1 = cnt;   
            pin->timer=0;  
            // if (err_fl) pin->timer -=1;
            pin->exPin_st = 1;
      }
}

// Pin change interrupt service routine
interrupt [PC_INT0] void pin_change_isr(void)
{
// Place your code here
      unsigned char tmp = TCNT0;    // read timer

      PinChanged(&pin3, tmp, PINB.3);
#ifdef DOUBLE_PIN
      PinChanged(&pin4, tmp, PINB.4);
#endif
}

#if !defined(DOUBLE_PIN)
inline 
#endif
void ovf_action (struct timer_watch * pin, unsigned char new_cnt){
      if (pin->timer > TIME_ERROR_PAUSE){  
            pin->en=0;
            tcc_out_update;
      }       
      if (pin->exPin_st < 3) pin->timer++;

      if (pin->exPin_st == 1){
            if (new_cnt >= pin->tcnt_t1){
                  pin->timer -= 1;
            }
            pin->exPin_st = 2;
      }
      else if (pin->exPin_st == 3){
            if (new_cnt >= pin->tcnt_t2){
                  pin->timer += 1;
            }
            pin->exPin_st = 4;
      }
}

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)         //28.12us
{
// Place your code here

      unsigned char tmp = TCNT0;    // read timer 
      
      ovf_action(&pin3, tmp);
#ifdef DOUBLE_PIN
      ovf_action(&pin4, tmp);
#endif
      // middle = 0;     
}      

// // Timer 0 output compare B interrupt service routine
// interrupt [TIM0_COMPB] void timer0_compb_isr(void)
// {
// // Place your code here
//       middle = 1;
// }

void loop (struct timer_watch *pin){
      // unsigned int PWM_out = 0x00;  
      unsigned int time_ms = 0;
      
      if ((pin->exPin_st < 4)) return;
       
      time_ms =  (((((pin->timer)<<8) + pin->tcnt_t2 - pin->tcnt_t1)*1000L)/TIM_FREQ); //8400L);

      if (pin->timer == 0) time_ms=0;   
      if (time_ms>TIME_ERROR_PULSE) time_ms=0;   
      else if (time_ms<TIME_STARTUP_MIN) time_ms=0;
      else if (time_ms < startup_imp_min) startup_imp_min = time_ms;   

      // if (time_ms>2000L) time_ms=2000L;
      #define  PWM_out time_ms

      PWM_out = (((time_ms-startup_imp_min)*255L)/startup_imp_min);    

      if (PWM_out>0xFF) PWM_out = 0xFF;   
      // if (time_ms < pin->startup_imp_min) PWM_out = 0;

      if (PWM_out <= 25) pin->en=0; 
      else pin->en=1;
      tcc_out_update;

#ifdef DOUBLE_PIN
      if (pin == &pin3){
            OCR0A = PWM_out&0xFF;   
      }
      else{
            OCR0B = PWM_out&0xFF; 
      }
#else 
      OCR0A = PWM_out&0xFF;   
#endif

      pin->exPin_st = 0;
            
}

void main(void)
{
// Declare your local variables here

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port B initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=Out 
DDRB=(0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit5=T Bit4=P Bit3=P Bit2=T Bit1=1 Bit0=0 
PORTB=(0<<PORTB5) | (1<<PORTB4) | (1<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 8000.000 kHz              changed
// Mode: Fast PWM top=0xFF
// OC0A output: Non-Inverted PWM
// OC0B output: Non-Inverted PWM
// Timer Period: 0.032 ms                 changed
// Output Pulse(s):
// OC0A Period: 0.032 ms Width: 0 us      
// OC0B Period: 0.032 ms Width: 0 us
TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
// TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
#ifdef TIM_FREQ1
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (1<<CS00);
#elif defined (TIM_FREQ2)
TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
#elif defined (TIM_FREQ3)
TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
#endif
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 0 Interrupt(s) initialization
// TIMSK0=(1<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

// External Interrupt(s) initialization
// INT0: Off
// Interrupt on any change on pins PCINT0-5: On
GIMSK=(0<<INT0) | (1<<PCIE);
MCUCR=(0<<ISC01) | (0<<ISC00);
#ifdef DOUBLE_PIN
PCMSK=(0<<PCINT5) | (1<<PCINT4) | (1<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);        // add pin4 for irq
#else
PCMSK=(0<<PCINT5) | (0<<PCINT4) | (1<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);        // add pin4 for irq
#endif
GIFR=(0<<INTF0) | (1<<PCIF);

// // Analog Comparator initialization
// // Analog Comparator: Off
// // The Analog Comparator's positive input is
// // connected to the AIN0 pin
// // The Analog Comparator's negative input is
// // connected to the AIN1 pin
// ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIS1) | (0<<ACIS0);
// ADCSRB=(0<<ACME);
// // Digital input buffer on AIN0: On
// // Digital input buffer on AIN1: On
// DIDR0=(0<<AIN0D) | (0<<AIN1D);

// // ADC initialization
// // ADC disabled
// ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);


// Global enable interrupts
#asm("sei")

while (1)
      {
      // Place your code here
            loop(&pin3);
      #ifdef DOUBLE_PIN
            loop(&pin4);
      #endif
      }
}
