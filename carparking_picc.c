#include <16F886.h>
#device ADC=10 *=16
#include <VL53L0X.h>


#FUSES NOWDT                    //No Watch Dog Timer
#FUSES PUT                      //Power Up Timer
#FUSES NOMCLR                   //Master Clear pin not enabled
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOCPD                    //No EE protection
#FUSES BROWNOUT                 //Brownout reset
#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES FCMEN                    //Fail-safe clock monitor enabled
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES NOWRT                    //Program memory not write protected
#FUSES BORV40                   //Brownout reset at 4.0V
#FUSES RESERVED                 //Used to set the reserved FUSE bits
#FUSES INTRC_IO 

#use delay(clock=8M)

#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)
#use i2c(MASTER, I2C1, FORCE_HW)


#define millis()  (msTimer)

int state = 0;
int16 duty = 1200;
int32 msTimer=0;

#INT_TIMER1
void timer1_isr() {
   if(state == 0){
      output_high(PIN_B5);
      state = 1;
      set_timer1(65535-duty);
   }
   else if(state == 1){
      output_low(PIN_B5);
      state = 0;
      set_timer1(65535-(5000-duty));
   }
   msTimer++;
}


void pic_setup() {
  #use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);
   set_timer1(65286); // keep period at 1 ms (at 8 MHz)
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
}

#define LONG_RANGE
#define HIGH_ACCURACY

void setup()
{
  
   output_high(PIN_B6);
   //delay_ms(1000);
   output_low(PIN_B6);
   printf("Starting\r\n");
 //  pic_setup();

  init();
 // setTimeout(200);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  setVcselPulsePeriod(VcselPeriodPreRange, 18);
  setVcselPulsePeriod(VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  setMeasurementTimingBudget(200000);
#endif
}
void loop(int8 times)
{
  printf("%d\r\n", times);
  printf("%lu\r\n", readRangeSingleMillimeters());
  int16 distance = readRangeSingleMillimeters();
  
  //car1
  if(times == 3){
      if(distance > 220){
         output_high(PIN_B0); 
      }
      else{
         if(distance < 100){
         }
         else{
            output_low(PIN_B0);
         }
      }
  }
   //car2
  if(times == 6){
      if((distance > 210) && (distance < 300)){
         output_low(PIN_B1); 
      }else{
         if(distance < 211){
         }
         else{
            output_high(PIN_B1);
         }
      }
  }
  
  //car3
  if(times == 9) {
      if((distance > 270) && (distance < 330)){
         output_low(PIN_B2); 
      }else{
         if(distance < 271){
         }
         else{
            output_high(PIN_B2);
         }
      }
  }
  
  //car4
  if(times == 12){
      if((distance > 420) && (distance < 445)){
         output_low(PIN_B3); 
      }
      else{
      //output_low(PIN_B3);
         if(distance < 421){
         }
         else{
            output_high(PIN_B3);
         }
      }
  }
  
  //car5
  if(times == 20){
      if((distance > 430) && (distance < 460)){
         output_low(PIN_B4); 
      }
      else{
      //output_low(PIN_B4);
         if(distance < 431){
         }
         else{
            output_high(PIN_B4);
         }
      }
  }
  
  //car6
  if(times == 23){
      if((distance > 350) && (distance < 380)){
         output_low(PIN_B6); 
      }else{
         if(distance < 350){
         }
         else{
            output_high(PIN_B6);
         }
      }
  }
  
  //car7
  if(times == 26){
      if((distance > 210) && (distance < 300)){
         output_low(PIN_C0); 
      }else{
         if(distance < 211){
         }
         else{
            output_high(PIN_C0);
         }
      }
  }
  
  //car8
  if(times == 30){
      if(distance > 220){
         output_high(PIN_C5); 
      }else{
      //output_low(PIN_C5);
         if(distance < 100){
         }
         else{
            output_low(PIN_C5);
            }
      }
  }

delay_ms(500);
}

void main()
{
   output_low(PIN_B5);
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_4);
   set_timer1(65035);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);
   
   setup();
   
   while (1){
      loop(0);
      printf("-----------\n");
      duty = 280;
      
      for(int8 i=0;i<32;i++){
         loop(i+1);
         duty += 28.75;
         
      }
   }
}

#include <VL53L0X.c>
