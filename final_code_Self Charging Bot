/*
 * projectCS101.c
 *
 * Created: 4/11/2015 4:53:05 PM
 *  Author: Cosmos
 */ 

/*
Concepts Implemented :
   (1). Buzzer ON/OFF
	     Buzzer is connected to PORTC.3 of ATMEGAM2560
		 Output operation, generating exact delay
   (2). Simple motion control

         There are two components to the motion control:
         1. Direction control using pins PORTA0 to PORTA3
         2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B of timer 5.

         In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
         Pins for PWM are kept at logic 1.
  
         Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1;
    (3). Robot Velocity Control Using PWM 
	       Use of timer to generate PWM for velocity control

           There are two components to the motion control:
             1. Direction control using pins PORTA0 to PORTA3
             2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 
  
             Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 

	(4). Servo motor control using 10 bit fast PWM mode
	        Use of timer to generate PWM for servo motor control

           Fire Bird V ATMEGA2560 microcontroller board has connection for 3 servo motors (S1, S2, S3).
           Servo motors move between 0 to 180 degrees proportional to the pulse train 
           with the on time of 0.6 to 2 ms with the frequency between 40 to 60 Hz. 50Hz is most recommended.

           We are using Timer 1 at 10 bit fast PWM mode to generate servo control waveform.
           In this mode servo motors can be controlled with the angular resolution of 1.86 degrees.
           Although angular resolution is less this is very simple method.
 
          There are better ways to produce very high resolution PWM but it involves interrupts at the frequency of the PWM.
          High resolution PWM is used for servo control in the Hexapod robot.
  
          Connection Details:	PORTB 5 OC1A --> Servo 1: Camera pod pan servo
 						PORTB 6 OC1B --> Servo 2: Camera pod tile servo
						PORTB 7 OC1C --> Servo 3: Reserved 
	(5). LCD interfacing in 4 bit mode
	         LCD interfacing

                LCD Connections:
 			              LCD	  Microcontroller Pins
 			              RS  --> PC0
			              RW  --> PC1
			              EN  --> PC2
			              DB7 --> PC7
			              DB6 --> PC6
			              DB5 --> PC5
			              DB4 --> PC4
	(6). ADC captures the analog sensor values and displays it on the LCD
	          ADC, LCD interfacing

              LCD Connections:
 			            LCD	  Microcontroller Pins
 			             RS  --> PC0
			             RW  --> PC1
			             EN  --> PC2
			             DB7 --> PC7
			             DB6 --> PC6
			             DB5 --> PC5
			             DB4 --> PC4

              ADC Connection:
 			             ACD CH.	PORT	Sensor
			             0			PF0		Battery Voltage
			             1			PF1		White line sensor 3
			             2			PF2		White line sensor 2
			             3			PF3		White line sensor 1
			             4			PF4		IR Proximity analog sensor 1*****
	(7). the application of a simple line follower robot. The robot follows a white line over a black backround
             ADC, LCD interfacing, motion control based on sensor data
*/

/*
 LCD Display interpretation:
 ****************************************************************************
 * LEFT WL SENSOR	        CENTER WL SENSOR	                  RIGHT WL SENSOR		        Batt_Volt			                *
 * Volt_bot_movement 		Voltage_align_bot(at max) 			  Volt_servo_movement   		Volt_Align_Servo(at max)	    	*
 ****************************************************************************
*/

/*
NOTES:
     (1). Make sure that in the configuration options following settings are 
          done for proper operation of the code

          Microcontroller: atmega2560
          Frequency: 14745600
          Optimization: -O0
		  
	 (2).  Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
 	       2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
	       it produces large current surge. When robot is powered by Auxiliary power which can supply
	       only 1 Ampere of current, sudden direction change in both the motors will cause current 
	       surge which can reset the microcontroller because of sudden fall in voltage. 
	       It is a good practice to stop the motors for at least 0.5seconds before changing 
	       the direction. This will also increase the useable time of the fully charged battery.
	       the life of the motor.
		   
	 (3). It is observed that external interrupts does not work with the optimization level -Os
	 
	 (4). 5V supply to these motors is provided by separate low drop voltage regulator "5V Servo" which can
 	      supply maximum of 800mA current. It is a good practice to move one servo at a time to reduce power surge 
	      in the robot's supply lines. Also preferably take ADC readings while servo motor is not moving or stopped
	      moving after giving desired position.
		  
	 (5). Make sure that you copy the lcd.c file in your folder
*/

/*************************************************************************************************************************************************************/


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"

// few public variables accessible from any part of the code
int max_angle_of_bot=0;
const float rot_time=61.8;
const float rot_time2=23.3;
const int delay_time=400; // in milliseconds

//Function to initialize Buzzer
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void port_init_buzzer (void)
{
	buzzer_pin_config();
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void init_devices_buzzer (void)
{
	cli(); //Clears the global interrupts
	port_init_buzzer();
	sei(); //Enables the global interrupts
}

//Main Function
void buzzer(void)
{
	init_devices_buzzer();
	for(int i=0;i<1;i++)
	{
		buzzer_on();
		_delay_ms(1000);		//delay
		buzzer_off();
		_delay_ms(1000);		//delay
	}
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	unsigned char ADC_Value;
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
} 

// Motion pin configuration for the movement of wheels of the bot
void motion_pin_config (void)
{
     DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
     PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
     DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
     PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale :256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//{
//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
//}
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

//Function to initialize ports
void servo_port_init(void)
{
 servo1_pin_config();  //Configure PORTB 5 pin for servo motor 1 operation
}
void port_init_motion(void)
{
	motion_pin_config(); // configure PORTA 0,1,2,3 for motor movement
}
void port_init_lcd(void)
{
   lcd_port_config();  // configure PORT C 1,2,3,4,5,6,7
}
void port_init_adc(void)
{
	adc_pin_config();  //Configure PORT F for ADC conversion
}


// for rotation of wheels of the bot
void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	PORTA=0x02;
}
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{  
	PORTA=0x01;
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	PORTA=0x08;
}
void forward (void) //both wheels forward
{
	PORTA=0x06;
}
// Stoping the wheels of the bot
void stop (void) //hard stop
{
	PORTA=0x00;
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}



// Initialising devices

void init_devices_adc (void)
{
	cli(); //Clears the global interrupts
	port_init_adc(); // adc pin configuration
	adc_init();
	timer5_init();
	sei();   //Enables the global interrupts
}

void init_devices_motion(void)
{
 cli(); //disable all interrupts
 port_init_motion(); // motion pin config&& lcd pig config
 port_init_lcd();   // lcd initialisation
 timer5_init();
 sei(); //re-enable interrupts 
}

void init_devices_servo (void)
{
	cli(); //disable all interrupts
	servo_port_init();
	timer1_init();
	sei(); //re-enable interrupts 
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
 PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}
// aligning the panel on servo at 45 degree
void servo_panel_45(void)
{  
   init_devices_servo();
  for (int i = 0; i <45; i++)   //FIRST ALINGING THE PANEL AT 45      DEGREE FROM GROUND TO HAVE THE   LOCAL MAXIMUM INTENSITY IN ONE PLANE                                  
  {
  servo_1(i);
  _delay_ms(100);
  }
  _delay_ms(1000);
  servo_1_free();  
}

// Bringing the panel at 0 angle.
void servo_panel_0(void)
{
	init_devices_servo();
	_delay_ms(500);
	servo_1_free();
	
}

/*
Since the ADC conversion is 8 bits so maximum type of binary available is 256
Panel voltage varies from 0 min to 12V max
SO Similar conversion is done
Since values available is low so they are magnified by 100 times in order for better comparison.
*/
float value_in_volt(unsigned char x)
{  float value=x;
   float max_voltage_of_panel=12;
   int bit_resolution=256;
   int magnification =100;
   return (((value*max_voltage_of_panel)/bit_resolution)*magnification);
}

/*
Since the ADC conversion is 8 bits so maximum type of binary available is 256
Battery voltage varies form 0 minimum to 12V maximum
12V correspond to 256
similarly xV=given below.
*/
float batt_volt(unsigned char x)
{
  float bat_vat=x;
  return ((bat_vat*12)/256);
}


/*
This function first rotates the bot by 5 degree till 360 
Stores the voltage at each angle in array
Find maximum value of voltage from the array
Aligns the bot at that value of voltage.
*/
void bot_rotation360()
{    
     init_devices_motion();
	 init_devices_adc();
	 int bot_ang=0;
	 float panel_voltage[72];
	 float max_panel_volt=0;
	 float bat_voltage = 0.0;
	 int j;
	
	
	 for(j=0;j<72;j++)
	 {
		 
		 soft_right();
		                           // halting right wheel and moving only the left wheel for specified time to get the desired angle of rotation(according to calculation)
		 _delay_ms(rot_time);      // rot_time is calculated according to power from the battery
		 
		  stop();
		  _delay_ms(delay_time);
		
		 panel_voltage[j]=value_in_volt(ADC_Conversion(10)); // channel 10 contains voltage reading 
		 lcd_print(2,1,panel_voltage[j],3);
		 bat_voltage=batt_volt(ADC_Conversion(0));
		 lcd_print(1,13,bat_voltage,4);      // Printing battery voltage.
		
	 }
	  _delay_ms(1000);      // Stopping the bot momentarily.
	  
	  // for getting the angle of bot corresponding to maximum voltage and maximum voltage also.
	  for( j=0;j<72;j++)
	  {      
		  if(panel_voltage[j]>max_panel_volt)
		  {
			  max_panel_volt=panel_voltage[j];
			  bot_ang=j;
		  }
	  }
     lcd_print(2,1,panel_voltage[bot_ang],3);      // Printing maximum voltage.
	 
	                                // Realigning the bot at the maximum angle of intensity in circular plane
	   j=1;         
	  while(j!=bot_ang)            // Since the bot_ang variable contains the angle for which the intensity was maximum
	  {
		  
		  soft_right();       //left wheel forward leaving the right wheel at rest to get
		                     //soft rotation at the axis of right wheel
		  _delay_ms(rot_time2);
		  float servo_volt=value_in_volt(ADC_Conversion(10));
		  lcd_print(2,5,servo_volt,3);
		  j++;
		  stop();
		  _delay_ms(delay_time);
		  
	  }
	   lcd_print(2,5,panel_voltage[bot_ang],3);
	 	
	  max_angle_of_bot=bot_ang;
	 
}

 
/* Rotating the panel in the plane of servo hinge by one degree each till 165 
 Here we have rotated the servo only upto 165 not 180 due to large size of panel, that is when rotated by more than 
 165 it start touching the upper plate of bot.
 Here first the servo is rotated upto 165 each by one degree and the value corresponding to voltage is noted in the array
 then maximum value is calculated from the array and the corresponding angle.
 Then the panel is aligned to that angle.
 Delay time for each rotation is 400 milli-seconds.
*/
void servo_rotation_165()
{ 
  init_devices_servo();
  unsigned int  i = 0;
  float max_panel_volt=0.0;
  int counter=0;
  float panel_voltage_s[165];
	
  for (i = 0; i <165; i++)
 {
   servo_1(i);
   _delay_ms(delay_time);
   
   panel_voltage_s[i]=value_in_volt(ADC_Conversion(10));   // ADC_Conversion gives analog value of voltage through channel 10
   lcd_print(1,13,batt_volt(ADC_Conversion(0)),4);	// Printing the battery voltage.
   lcd_print(2,9,panel_voltage_s[i],3);       // Printing voltage of panel at specific angles.
                                             // _delay_ms(50);	
 }

 // finding the maximum intensity of value and corresponding angle
   for(int j=0;j<165;j++)
  {
    if(panel_voltage_s[j]> max_panel_volt)
    {
     max_panel_volt= panel_voltage_s[j];
     counter=j;       // identifier counter contains the angle for which the intensity is maximum
    }
  }
   lcd_print(2,9,panel_voltage_s[counter],3);   // Printing maximum voltage.
   
           
// setting the panel at that angle of maximum intensity
   
   servo_panel_0();
  
  for (int j = 0; j<counter;j++)             // Setting the panel at maximum voltage.
  {  
	  servo_1(j);
	  _delay_ms(delay_time);
	  float panel_volt=value_in_volt(ADC_Conversion(10));
	  lcd_print(2,13,panel_volt,3);
	  
  }
  
  lcd_print(2,13,panel_voltage_s[counter],3);      // Checking the value to be exact by printing again.

  _delay_ms(1000);
  servo_1_free();
  
 
} 


void bot_charging_process(void)
{

//  Keeping the panels at 45 degree to find maximum intensity in the plane of bot.
       servo_panel_45();
// checking maximum intensity by rotating the bot in its plane by 5 degrees till 360.

   bot_rotation360();
// checking for the angle of maximum intensity in the plane perpendicular to the plane of bot by moving the servo motors
  servo_panel_0();
  servo_rotation_165();
  

// charging the bot at that intensity for constant time t(predefined)
  _delay_ms(20000);        // Time of Charging the bot.
}


// MAIN FUNCTION 
int main()
{ 
	
	//init_devices2();
	 
	
	// White Line Following
	while(1) 
	{
		init_devices_motion();
		init_devices_adc();
		lcd_set_4bit();
		lcd_init();
		
		unsigned char flag = 0;
		unsigned char Left_white_line = 0;
		unsigned char Center_white_line = 0;
		unsigned char Right_white_line = 0;
		unsigned char lwl=0;
		unsigned char cwl=0;
		unsigned char rwl=0;
		
		while(1)
		{
            
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			flag=0;

			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3
			
			

			if(Center_white_line<0x10)
			{
				flag=1;
				forward();
				velocity(100,100);
				//_delay_ms(10000);
			}

			if((Left_white_line>0x10) && (flag==0))
			{   
				
				//bot_charging_process();
				flag=1;
				forward();
				velocity(130,30);
			}

			if((Right_white_line>0x10) && (flag==0))
			{
				//bot_charging_process();
				flag=1;
				forward();
				velocity(30,130);
			}

			if(Center_white_line>0x10 && Left_white_line>0x10 && Right_white_line>0x10)
			{
				forward();
				velocity(0,0);
			}
			
			if((((Center_white_line-Right_white_line>0)&&(Center_white_line-Right_white_line<=10))||((Right_white_line-Center_white_line>0)&&(Right_white_line-Center_white_line<=10)))&&
			   (((Center_white_line-Left_white_line>0)&&(Center_white_line-Left_white_line<=10))||((Left_white_line-Center_white_line>0)&&(Left_white_line-Center_white_line<=10)))&&
			   (((Left_white_line-Right_white_line>0)&&(Left_white_line-Right_white_line<=10))||((Right_white_line-Left_white_line>0)&&(Right_white_line-Left_white_line<=10)))
		
		          ) // recognising node position that is if the reading at all three white line sensors differ maximum by 10 then that point will be node
		       {	
			     lwl=Left_white_line;
				 cwl=Center_white_line;
				 rwl=Right_white_line;
				 stop();
			     _delay_ms(500);
			      break;		   
			  }
		  }	
	  bot_charging_process();
	 //buzzer();
	 
	 
	for(int t=max_angle_of_bot;t>=0;t--)
	 {
		  soft_left_2();       //left wheel forward leaving the right wheel at rest to get
		                       //soft rotation at the axis of right wheel
		  _delay_ms(rot_time);
		  
		  stop();
		  _delay_ms(100);
	 }	
	 
	 forward();
	 _delay_ms(100);  	 
   }
}


