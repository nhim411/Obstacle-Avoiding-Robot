#include <main.h>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atol */
// control loop constants 
float Kp=1;   // proportional gain 
float Ki=0;   // integral gain 
float Kd=0;   // differential gain 

// terms 
float Tp;   // proportional term 
float Ti;   // integral term 
float Td;   // differential term 

// circular queue vars 
signed long error_history[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; 
int8 queue_pos = 0; 
int8 temp_pos; 
signed long prev_ave; 
signed long cur_ave; 

signed long error; 
signed long DeDt; // change in error over time 
signed int32 error_sum = 0; // sum of errors, for integral term 
signed long desired_power; 
unsigned int16 power; 
unsigned int16 setpoint=180; 
float temp_float; 
unsigned int16 left_speed;
unsigned int16 right_speed;
short turn_left=0;
short turn_right=0;
short temp_finish=0;
int8 temp_delay=0;
short button_left=1;
short button_right=0;

#define thresh_infrared 500
int8 zone = 0;
short sensor_bot = 0;
short status = 0;

unsigned int16 EN1,EN2;
unsigned int16 stt,i,stt1;
//char str;
unsigned int16 yaw=0;

//unsigned int16 a,buffer=0,x=0,b=0;
//#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 9, stream = RS232, errors)
#use rs232 (baud = 9600, xmit = PIN_C6, rcv = PIN_C7)

volatile char arr[7];
unsigned int8 j;
short start = 0;
#INT_RDA
void RDA_isr (void) 
{ 
 char c = getc();
 if (start == 1)
 {
  arr[j++] = c;
  if (c == '\r')
  {
   start = 0;
   j = 0;
   yaw = atol(arr);
   printf ("The yaw is: %ld.\n",yaw);
   }
  }
 if (c=='g') {
 start = 1;
 }
}
#use rtos(timer=1,minor_cycle=10ms)
#task(rate=100ms,max=1ms)
void live()
{
   output_toggle(PIN_E0);
}

#task(rate=10ms,max=10ms) // task dong co 1 chay
void DC1()
{
   if(turn_right==0){
      output_high(PIN_D0);
      output_low(PIN_C3);
   }
   else{
      output_low(PIN_D0);
      output_high(PIN_C3);
   }
   set_pwm2_duty(left_speed);
}


#task(rate=10ms,max=10ms) // task dong co 2 chay
void DC2()
{
   if(turn_left==0){
   output_high(PIN_C4);
   output_low(PIN_C5);
   }
   else{
   output_low(PIN_C4);
   output_high(PIN_C5);   
   }
   set_pwm1_duty(right_speed);
}


#task (rate=100ms,max=10ms) 
   void uart_use () 
   { 
   printf("%d\n\r", zone); 
   } 
#task (rate=10ms,max=10ms) 
void zone_dectected () 
{ 
   if (read_adc()<thresh_infrared) sensor_bot = 1;
   else sensor_bot = 0;
   if(sensor_bot!=status)
   {
      if(sensor_bot==0) zone++;
   }
   if(zone>=8) zone=0;
   status = sensor_bot;
}
#task(rate=10ms,max=10ms)
void pid_compute()
{
// calculate the raw error 
// negative = disc too low 
error = yaw - setpoint; 
if(error>0) error=-error;
// calculate the proportional term 
Tp = -Kp * error; 

// calculate the integral term 
error_sum = error_sum + (signed int32)error; 
temp_float = error_sum; 
Ti = Ki * temp_float; 

// use a circular queue to save a history of the last 8 samples 
// this will be used to calculate the differential term 
error_history[queue_pos] = error; 
queue_pos++; 
queue_pos &= 0x07;   // keep in 0..7 range 
temp_pos = queue_pos; 

// calculate the average for the 4 oldest samples 
for (i = 0, prev_ave = 0; i < 4; i++) 
{ 
prev_ave += error_history[temp_pos]; 
temp_pos++; 
temp_pos &= 0x07; 
} 

// calculate the average for the 4 most recent samples 
for (i = 0, cur_ave = 0; i < 4; i++) 
{ 
cur_ave += error_history[temp_pos]; 
temp_pos++; 
temp_pos &= 0x07; 
} 

// calculate the differential term 
DeDt = prev_ave - cur_ave; 
Td = Kd * DeDt; 

// calculate the desired power 
desired_power = (signed long)(Tp + Td + Ti); 

// set the correct power 
if (desired_power < 0) 
power = 0; 
else if (desired_power > 255) 
power = 255; 
else 
power = desired_power; 
//set_output_power(power);   // this could be pwm duty, etc 

// wait between samples 
delay_ms(2);  
}
#task(rate=10ms,max=10ms)
void control()
{
if (zone <3){
if (yaw>=180){
          left_speed= 200-power;
          right_speed= 200;   
        }
        else {
          left_speed= 200;
          right_speed= 200-power;    
        }
        }
if (zone == 3 && temp_finish==0){
//if (zone == 3 && temp_finish==0 && input(PIN_A1 == 0){
        right_speed=0;
        left_speed=0;
        if(button_left==1) setpoint=90;
        if(button_right==1) setpoint=270;
        if((yaw>=90 && button_left==1) || (yaw>=270 && button_right==1)){
        turn_left=1;
        turn_right=0;
        }
        else{
        turn_left=0;
        turn_right=1;
        }
        right_speed=power*0.7+90;
        left_speed=power*0.7+90;
}
if ((zone > 3 && zone <5) || temp_finish==1){
if(button_left==1){       
if (yaw>=90){
          left_speed= 200-power;
          right_speed= 200;   
        }
        else {
          left_speed= 200;
          right_speed= 200-power;    
        }
        }
if(button_right==1){       
if (yaw>=270){
          left_speed= 200-power;
          right_speed= 200;   
        }
        else {
          left_speed= 200;
          right_speed= 200-power;    
        }
        }
}
if (zone>=5){
         temp_finish=0;
         right_speed=0;
         left_speed=0;

}
//!        right_speed=right_speed*3;
//!        left_speed=left_speed*3;
}
#task(rate=2000ms,max=10ms)
void finish()
{
if (zone==3) temp_delay++;
if (temp_delay==2){
turn_left=0;
turn_right=0;
temp_finish=1;
}
}
#task(rate=10ms,max=10ms)
void button()
{
if(input(pin_A4)==0)
{
button_left=0;
button_right=1;
}
else{
button_left=1;
button_right=0; 
}
}
void main()
{
   setup_timer_2(T2_DIV_BY_16,255,1);      //819 us overflow, 819 us interrupt
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   set_tris_b(0xff);
   set_tris_d(0x00);
   set_tris_e(0);
   set_tris_a(1);
   port_b_pullups(1);
   setup_adc(ADC_CLOCK_DIV_32);
   setup_adc_ports(AN0);
   set_adc_channel(0);
  
   EN1=0;
   EN2=0;
   stt=1;
   stt1=1;
   enable_interrupts (INT_RDA); 
   enable_interrupts (GLOBAL);
   while(true){
   if(input(pin_A5)==0) rtos_run();
   }
}
