//Preprocessor Directives ->
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
//#include "lcd.c"
#include "lcd.h"

/* --------------------------------------------------------------*/

//Wiring ->
//
//Buzzer -- PORT C3
//Direction Control of Motor Driver -- PORT A0, PORT A1, PORT A2, PORT A3
//
//L-1---->PA0;		L-2---->PA1;
//R-1---->PA2;		R-2---->PA3;
//
//PWM Control of Motor Driver -- PORT L3, PORT L4
//
//PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1;
//
//External interrupt for left motor position encoder -- PE4 (INT4)
//External interrupt for the right position encoder -- PE5 (INT5)
//
//Servo(Base)(1) -- PORT B5
//Servo(Bulk)(2) -- PORT B6
//Demux Data (3,4) -- PORT B7
//Demux Selection -- PORT H0
//
//LCD --
//     LCD   Microcontroller Pins
//     RS  -- PORTC0
//     RW  -- PORTC1
//     EN  -- PORTC2
//     DB7 -- PORTC7
//     DB6 -- PORTC6
//     DB5 -- PORTC5
//     DB4 -- PORTC4
//
//Black Line Sensor --
//  ACD CH.	PORT    	Sensor
//    1		PORTF1		White line sensor 3
//    2		PORTF2		White line sensor 2
//    3		PORTF3		White line sensor 1
//    9		PORTK1		Sharp IR range sensor 1
//    10	PORTK2		Sharp IR range sensor 2
/* --------------------------------------------------------------*/

//Global Variables ->
//unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
//unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
//unsigned int Degrees; //to accept angle in degrees for turning
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading, wall;
unsigned int value;
unsigned char base = 255;
unsigned char turn = 185;
unsigned char soft = 205;
unsigned char ls, ms, rs;
unsigned char check;
/* --------------------------------------------------------------*/

//Timers ->
void timer1_init(void) {
  TCCR1B = 0x00; //stop
  TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
  TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
  OCR1AH = 0x03; //Output compare Register high value for servo 1
  OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
  OCR1BH = 0x03; //Output compare Register high value for servo 2
  OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
  OCR1CH = 0x03; //Output compare Register high value for servo 3
  OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
  ICR1H = 0x03;
  ICR1L = 0xFF;
  TCCR1A = 0xAB;
  /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
  				For Overriding normal port functionality to OCRnA outputs.
  			  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
  TCCR1C = 0x00;
  TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void timer5_init() {
  TCCR5B = 0x00; //Stop
  TCNT5H = 0xFF; //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01; //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00; //Output compare register high value for Left Motor
  OCR5AL = 0xFF; //Output compare register low value for Left Motor
  OCR5BH = 0x00; //Output compare register high value for Right Motor
  OCR5BL = 0xFF; //Output compare register low value for Right Motor
  OCR5CH = 0x00; //Output compare register high value for Motor C1
  OCR5CL = 0xFF; //Output compare register low value for Motor C1
  TCCR5A = 0xA9;
  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
  				  For Overriding normal port functionality to OCRnA outputs.
  			  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

  TCCR5B = 0x0B; //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
/* --------------------------------------------------------------*/

//Configuration Functions ->
//Function to initialize Buzzer
void buzzer_pin_config(void) {
  DDRC = DDRC | 0x08; //Setting PORTC 3 as output
  PORTC = PORTC & 0xF7; //Setting PORTC 3 logic low to turnoff buzzer
}

void motion_pin_config(void) {
  DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
  PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
  DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
  PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

/*//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config(void) {
  DDRE = DDRE & 0xEF; //Set the direction of the PORTE 4 pin as input
  PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config(void) {
  DDRE = DDRE & 0xDF; //Set the direction of the PORTE 5 pin as input
  PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 5 pin
}

void left_position_encoder_interrupt_config(void) //Interrupt 4 enable
{
  cli(); //Clears the global interrupt
  EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
  EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
  sei(); // Enables the global interrupt
}

void right_position_encoder_interrupt_config(void) //Interrupt 5 enable
{
  cli(); //Clears the global interrupt
  EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
  EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
  sei(); // Enables the global interrupt
}*/

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config(void) {
  DDRB = DDRB | 0x20; //making PORTB 5 pin output
  PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config(void) {
  DDRB = DDRB | 0x40; //making PORTB 6 pin output
  PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config(void) {
  DDRB = DDRB | 0x80; //making PORTB 7 pin output
  PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Configure PORTH 0 pin for demux operation
void demux_pin_config(void) {
  DDRH = DDRH | 0xFF; //making PORTH pin output
  PORTH &= ~(1 << 0); //Master
}

//ADC pin configuration
void adc_pin_config(void) {
  DDRF = 0x00; //set PORTF direction as input
  PORTF = 0x00; //set PORTF pins floating
  DDRK = 0x00; //set PORTK direction as input
  PORTK = 0x00; //set PORTK pins floating
}
/* --------------------------------------------------------------*/

//Port Initialization Function ->
void port_init(void) {
  //buzzer_pin_config();
  motion_pin_config();
  //left_encoder_pin_config();
  //right_encoder_pin_config();
  //left_position_encoder_interrupt_config();
  //right_position_encoder_interrupt_config();
  servo1_pin_config();
  servo2_pin_config();
  servo3_pin_config();
  demux_pin_config();
  lcd_port_config();
  adc_pin_config();
}
/* --------------------------------------------------------------

//Interrupt Service Routines ->
//ISR for right position encoder
ISR(INT5_vect) {
  ShaftCountRight++; //increment right shaft position count
}

//ISR for left position encoder
ISR(INT4_vect) {
  ShaftCountLeft++; //increment left shaft position count
}
 --------------------------------------------------------------

//Other Functions ->
void buzzer_off(void)
{
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore | 0x08;
    PORTC = port_restore;
}

void buzzer_on(void)
{
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore & 0xF7;
    PORTC = port_restore;
    _delay_ms(5000);
    buzzer_off();
}
--------------------------------------------------------------*/

//Function used for setting motor's direction
void motion_set(unsigned char Direction) {
  unsigned char PortARestore = 0;

  Direction &= 0x0F; // removing upper nibbel as it is not needed
  PortARestore = PORTA; // reading the PORTA's original status
  PortARestore &= 0xF0; // setting lower direction nibbel to 0
  PortARestore |= Direction; // adding lower nibbel for direction command and restoring the PORTA status
  PORTA = PortARestore; // setting the command to the port
}

void back(void) //both wheels forward
{
  motion_set(0x06);
}

void forward(void) //both wheels backward
{
  motion_set(0x09);
}

void right(void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void left(void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_right(void) //Left wheel stationary, Right wheel forward
{
  motion_set(0x04);
}

void soft_left(void) //Left wheel forward, Right wheel is stationary
{
  motion_set(0x02);
}

void soft_right_2(void) //Left wheel backward, right wheel stationary
{
  motion_set(0x01);
}

void soft_left_2(void) //Left wheel stationary, Right wheel backward
{
  motion_set(0x08);
}

void stop(void) //hard stop
{
  motion_set(0x00);
}
/* --------------------------------------------------------------*/

// Encoder Functions

/* --------------------------------------------------------------*/

// Function for robot velocity control
void velocity(unsigned char left_motor, unsigned char right_motor) {
  OCR5AL = (unsigned char) left_motor;
  OCR5BL = (unsigned char) right_motor;
}
/* --------------------------------------------------------------*/

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees) {
  float Pos = 0;
  Pos = ((float) degrees / 1.86) + 35.0;
  OCR1AH = 0x00;
  OCR1AL = (unsigned char) Pos;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees) {
  float Pos = 0;
  Pos = ((float) degrees / 1.86) + 35.0;
  OCR1BH = 0x00;
  OCR1BL = (unsigned char) Pos;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees) {
  PORTH &= ~(1 << 0); //Master
  float Pos = 0;
  Pos = ((float) degrees / 1.86) + 35.0;
  OCR1CH = 0x00;
  OCR1CL = (unsigned char) Pos;
}

void servo_4(unsigned char degrees) {
  PORTH |= (1 << 0); //Slave
  float Pos = 0;
  Pos = ((float) degrees / 1.86) + 35.0;
  OCR1CH = 0x00;
  OCR1CL = (unsigned char) Pos;
}

void servo_1_free(void) //makes servo 1 free rotating
{
  OCR1AH = 0x03;
  OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free(void) //makes servo 2 free rotating
{
  OCR1BH = 0x03;
  OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free(void) //makes servo 3 free rotating
{
  OCR1CH = 0x03;
  OCR1CL = 0xFF; //Servo 3 off
}

void servo_4_free(void) //makes servo 4 free rotating
{
  OCR1CH = 0x03;
  OCR1CL = 0xFF; //Servo 4 off
}
/* --------------------------------------------------------------*/
//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch) {
  unsigned char a;
  if (Ch > 7) {
    ADCSRB = 0x08;
  }
  Ch = Ch & 0x07;
  ADMUX = 0x20 | Ch;
  ADCSRA = ADCSRA | 0x40; //Set start conversion bit
  while ((ADCSRA & 0x10) == 0); //Wait for ADC conversion to complete
  a = ADCH;
  ADCSRA = ADCSRA | 0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location.
void print_sensor(char row, char coloumn, unsigned char channel) {
  ADC_Value = ADC_Conversion(channel);
  lcd_print(row, coloumn, ADC_Value, 3);
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading) {
  float distance;
  unsigned int distanceInt;
  distance = (int)(10.00 * (2799.6 * (1.00 / (pow(adc_reading, 1.1546)))));
  distanceInt = (int) distance;
  if (distanceInt > 800) {
    distanceInt = 800;
  }
  return distanceInt;
}

void LCD_ON(void) {
  print_sensor(1, 1, 1); //Prints value of White Line Sensor Left
  print_sensor(1, 7, 2); //Prints Value of White Line Sensor Center
  print_sensor(1, 14, 3); //Prints Value of White Line Sensor Right

  wall = ADC_Conversion(11); //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
  lcd_print(2, 1, wall, 3); //Prints Value Of Distance in MM measured by Sharp Sensor.

  sharp = ADC_Conversion(13); //Stores the Analog value of front sharp connected to ADC channel 10 into variable "wall"
  lcd_print(2, 14, sharp, 3); //Prints Value Of Distance in MM measured by Sharp Sensor.
}

void LCD_Function(int a) {
  lcd_init();

  switch (a) {
  case 1:
    lcd_cursor(1, 3);
    lcd_string("Line-Follower");
    break;

  case 2:
    lcd_cursor(1, 3);
    lcd_string("Wall-Follower");
    break;

  case 3:
    lcd_cursor(1, 3);
    lcd_string("Zig-Zag");

  }
}
/* --------------------------------------------------------------*/
void m_pick(void) {
    servo_1(95);
    _delay_ms(750);
    servo_1_free();
    servo_3(115);
    _delay_ms(750);
    servo_3_free();
    servo_2(102);
    _delay_ms(1000);
 //   servo_2_free();
    servo_3(30); //35,105
    _delay_ms(750);
    servo_2(0);
    _delay_ms(1000);

}

void s_pick(void) {
  servo_1(70);
  _delay_ms(2000);
  servo_1_free();
  servo_4(60);
  _delay_ms(2000);
  servo_4_free();
  servo_2(165);
  _delay_ms(2000);
  servo_2_free();
  servo_4(120); //60,120
  _delay_ms(2000);
  servo_2(130);
  _delay_ms(2000);
  servo_2_free();
  servo_1(95);
  _delay_ms(1000);
  servo_1_free();
}

void m_place_lr(void) {
  servo_1(95);
  _delay_ms(750);
  servo_1_free();
  servo_3(30);
  _delay_ms(750);
  servo_2(80);
  _delay_ms(1000);
  servo_3(105); //35,105
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);
}

void s_place_lr(void) {
  servo_1(70);
  _delay_ms(2000);
  servo_4(120);
  _delay_ms(1000);
  servo_2(165);
  _delay_ms(1000);
  servo_4(60); //60,120
  _delay_ms(2000);
  servo_2(130);
  _delay_ms(1000);
  servo_1(95);
}
/* --------------------------------------------------------------*/

void forward_wls(int a);

void forward_walls() {
  LCD_Function(2);
  forward();
  OCR5AL = 220;
  OCR5BL = 220;

  while (1) {
    wall = ADC_Conversion(11);

    forward();

    if ((wall >= 105) && (wall <= 135)) {
      OCR5AL = 220;
      OCR5BL = 220;
      _delay_ms(100);

    } else if (wall > 135) {
      OCR5AL = 220;
      OCR5BL = 220;
      right();
      _delay_ms(1);

    } else if (wall < 105 && wall > 70) {
      OCR5AL = 220;
      OCR5BL = 220;
      left();
      _delay_ms(1);

    } else if (wall < 70) {
      _delay_ms(50);
      break;

    }

  }
  forward_wls(0);
}

void forward_wls(int a)

{ LCD_Function(1);
  check = 0;
  //lcd_print(2,7,check,1);
  forward();
  velocity(base, base);
  while (1) {
    ls = ADC_Conversion(1);
    ms = ADC_Conversion(2);
    rs = ADC_Conversion(3);

    if ((a == 2) && (ls + ms + rs > 300)) {
      stop();
      _delay_ms(500);
      break;
    } else if ((a == 0) && (ls + ms + rs > 400)) {
      stop();
      _delay_ms(500);
      break;
    } else if ((ls < 60 && ms >= 125 && rs < 60)) {

      OCR5AL = base;
      OCR5BL = base;
    } else if ((ls < 40 && ms < 60 && rs > 140)) {
      OCR5AL = base;
      OCR5BL = turn;

    } else if (rs < 40 && ms < 60 && ls > 140) {
      OCR5AL = turn;
      OCR5BL = base;

    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls > rs) {
      OCR5AL = soft;
      OCR5BL = base;

    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls < rs) {
      OCR5AL = base;
      OCR5BL = soft;
    } else if (a == 1) {
      wall = ADC_Conversion(11);

      if ((wall > 60) && (ls < 80) && (ms < 80) && (rs < 80))

      {

        stop();
        _delay_ms(500);
        break;

      }
    }

  }
  if (a == 1) {
    forward_walls();
  }

}
void static_reorientation() {
  while (1) {
    stop();
    ls = ADC_Conversion(1);
    ms = ADC_Conversion(2);
    rs = ADC_Conversion(3);
    OCR5AL = base;
    OCR5BL = base;
    if (ls < 60 && ms >= 125 && rs < 60) {
      break;
    } else if (ls < 40 && ms < 60 && rs > 140) {
      OCR5BL = 115;
      soft_right_2();
    } else if (rs < 40 && ms < 60 && ls > 140) {
      OCR5AL = 120;
      soft_left_2();
    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls > rs) {
      OCR5AL = 120;
      soft_left_2();
    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls < rs) {
      OCR5BL = 120;
      soft_right_2();
    } else if (ls < 40 && ms < 40 && rs < 40) {
      stop();
    }
  }
}

void right_turn_wls(void) {
	
  forward();
  _delay_ms(300);
  right(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
  _delay_ms(200);
  stop();
  _delay_ms(50);
  right();
  OCR5AL = 100;
  OCR5BL = 100;
  while (1) //while loop which detects black line using middle sensor so that the robot stops turning
  {
    rs = ADC_Conversion(3);
    if (rs >= 80) {
      PORTA = 0x00;
      break;
    }
  }
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;

}

void left_turn_wls(void) {
//
  //forward();
  //_delay_ms(250);
  left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
  _delay_ms(200);
  stop();
  _delay_ms(50);
  left();
  OCR5AL = 100;
  OCR5BL = 100;
  while (1) //while loop which detects black line using middle sensor so that the robot stops turning
  {
    ms = ADC_Conversion(2);
    if (ms >= 80) {
      PORTA = 0x00;
      break;
    }
  }
  left();
  _delay_ms(10);
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;

}
int forward_untw() {
  forward();
  _delay_ms(50);
  velocity(180, 180);
  int n = 0;
  while (1) {
    ls = ADC_Conversion(1);
    ms = ADC_Conversion(2);
    rs = ADC_Conversion(3);

    /* if(ls + ms + rs > 360 )
    {
    	stop();
    	_delay_ms(500);
    	break;
    }
    */
    if ((ls < 60 && ms >= 125 && rs < 60)) {
      OCR5AL = 210;
      OCR5BL = 210;
    } else if ((ls < 40 && ms < 60 && rs > 140)) {
      OCR5AL = 210; //check
      OCR5BL = 180; //check
    } else if (rs < 40 && ms < 60 && ls > 140) {
      OCR5AL = 180; //check
      OCR5BL = 210; //check

    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls > rs) {
      OCR5AL = 180;
      OCR5BL = 210;
    } else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls < rs) {
      OCR5AL = 210;
      OCR5BL = 180;
    } else if (ls < 60 && ms < 40 && rs < 60) {
      stop();
      break;
    } else if ((ls + ms + rs) > 450) {
      stop();
      n = 1;
      break;
    }
  }
  if (n == 0) {
    forward();
    _delay_ms(75);
  }
  return n;
}

void forward_zigzag() {
  LCD_Function(3);
  int delay = 350;
  int temp;
  while (1) {
    temp = forward_untw();
    if (temp == 1)
      break;
    left();
    _delay_ms(delay);
    stop();
    right_turn_wls();
  }
  forward();
  velocity(base, base);
  _delay_ms(250);
  stop();
  _delay_ms(1000);

}

/* --------------------------------------------------------------*/

//Devices Initialization Function ->
//Function to Initialize ADC
void adc_init() {
  ADCSRA = 0x00;
  ADCSRB = 0x00; //MUX5 = 0
  ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void init_devices(void) {
  cli(); //Clears the global interrupts
  port_init();
  adc_init();
  timer5_init();
  timer1_init();
  sei(); //Enables the global interrupts
  lcd_set_4bit(); //These functions need not to be inside interrupt blocked code
  lcd_init();
}
/* --------------------------------------------------------------*/

//Main Function ->
int main() {
  init_devices();
  //buzzer_on();

 /* forward_wls(0);
  right_turn_wls();

  forward_wls(2);
  right_turn_wls();

  forward_wls(0);
  right_turn_wls();
  back();
  _delay_ms(120);
  stop();

  //m_pick();

  left_turn_wls();
  forward_wls(0);
  right_turn_wls();

  forward_wls(1);
  */
 //_delay_ms(1000);
 m_pick();
 left_turn_wls();
 forward_wls(0);
 forward();
 _delay_ms(350);
 left_turn_wls();
back();
_delay_ms(300);
stop();
 m_place_lr();
 //_delay_ms(10000);
  //forward();
  //_delay_ms(250);
  //left_turn_wls();
//
  //forward_wls(0);
  //forward_wls(0);
  //
  //forward();
  //_delay_ms(300);
  //left_turn_wls();
//
  //forward_zigzag();
  //forward();
  //_delay_ms(250);
  //left_turn_wls();
//
  //forward_wls(0);
  //forward_wls(0);
  //forward_wls(0);
//
  //forward_wls(2);
  //forward();
  //_delay_ms(250);
  //left_turn_wls();
//
  //forward_wls(2);
  //forward();
  //_delay_ms(250);
  //left_turn_wls();
//
  //stop();
  //_delay_ms(10000);
//while(1)
//{
	//servo_2(0);
	//_delay_ms(2000);
	//servo_2(180);
	//_delay_ms(2000);
	//servo_2_free();
//}
}
/* --------------------------------------------------------------*/