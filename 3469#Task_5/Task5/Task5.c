
/*
 *
 * Team Id: 3469
 * Author List: Abhay Maurya, Ratnesh Mohan, Shubhankar Jain, Saumya Gupta
 * Filename: CB_Task_4.c
 * Theme: Construct-O-Bot - Specific to eYRC
 * Functions: timer1_init, timer5_init, buzzer_pin_config, motion_pin_config, servo1_pin_config, servo2_pin_config, buzzer_off, buzzer_on,
 * servo3_pin_config, demux_pin_config, adc_pin_config, port_init, motion_set, back, forward, right, left, soft_right,
 * soft_left, soft_right_2, soft_left_2, stop, velocity, servo_1, servo_2, servo_3, servo_4, servo_1_free, servo_2_free, servo_3_free,
 * servo_4_free, ADC_Conversion, print_sensor, Sharp_GP2D12_estimation, LCD_ON, LCD_Function, m_pick, s_pick, m_place_lr, s_place_lr,
 * s_place_hr, forward_walls, forward_wls, left_turn_wls, static_reorientation, right_turn_wls, right_turn_wls_bwall, forward_untw,
 * forward_zigzag, Wall_run, adc_init, init_devices
 * Global Variables: ADC_Value, adc_reading, sharp, distance, wall, value, base, turn, soft, ls, rs, ms;
 *
 */

//Preprocessor Directives ->
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h"
#define inf 0
#define max 15
#define infi 9999

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
//
//Buzzer --
//GND -- PORTG9
//VCC -- PORTG8
//Data -- PORTG3
/* --------------------------------------------------------------*/

//Global Variables ->

/* ----------------------------CHANGE THESE CONFIGURATION----------------------------------*/
int n = 15;							//total node count and initial node
int u = 0;
char face = 'w';					//direction variables which include directions w,n,e,s
char fdir = 'n';
int house_config[5] = {1,0,0,1,0};
int house_req[5] = {2,2,2,1,2};
int which_material[10] = {3,11,9,5,4,7,1,0,2,8};
int dis[15];
int pre[15];
int b1n = 1;
int b2n = 1;
int b3n = 13;
int b4n = 13;
int b5n = 4;
int b6n = 4;
int b7n = 11;
int b8n = 11;
int b9n = 6;
int b10n = 6;
int b11n = 9;
int b12n = 9;

int h1 = 3;
int h2 = 12;
int h3 = 5;
int h4 = 10;

int block_placed[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

unsigned char ADC_Value, adc_reading;
unsigned char sharp, distance, wall; //ADC Output from Sharp sensor
unsigned int value;
unsigned char base = 255; //base velocity of motor
unsigned char turn = 185; //turn velocity of motor
unsigned char soft = 205; //soft turn velocity of motor
unsigned char ls, ms, rs; //ADC Output from line sensors
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning

/* ----------------------------DIJKSTRA----------------------------------*/

#define wf 8
#define zz 6
#define in 8
void reverse(int a[],int n)
{
	int l=n-1;
	int i=0;
	while(i<=l)
	{
		int temp;
		temp=a[i];
		a[i]=a[l];
		a[l]=temp;
		i++;
		l--;
	}
}
int G[15][15] = {
	// 0  1	  2   3   4	  5	  6	  7	  8	  9	 10	 11	 12	 13	 14
	{inf,2,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,2},//0
	{2,inf,1,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf},//1
	{inf,1,inf,1,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf},//2
	{inf,inf,1,inf,1,inf,inf,inf,inf,inf,inf,inf,wf,inf,inf},//3
	{inf,inf,inf,1,inf,1,inf,inf,inf,inf,inf,inf,inf,inf,inf},//4
	{inf,inf,inf,inf,1,inf,1,inf,inf,inf,zz,inf,inf,inf,inf},//5
	{inf,inf,inf,inf,inf,1,inf,1,inf,inf,inf,inf,inf,inf,inf},//6
	{inf,inf,inf,inf,inf,inf,1,inf,in,inf,inf,inf,inf,inf,inf},//7
	{inf,inf,inf,inf,inf,inf,inf,in,inf,1,inf,inf,inf,inf,inf},//8
	{inf,inf,inf,inf,inf,inf,inf,inf,1,inf,1,inf,inf,inf,inf},//9
	{inf,inf,inf,inf,inf,zz,inf,inf,inf,1,inf,1,inf,inf,inf},//10
	{inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,1,inf,1,inf,inf},//11
	{inf,inf,inf,wf,inf,inf,inf,inf,inf,inf,inf,1,inf,1,inf},//12
	{inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,1,inf,1},//13
	{2,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,1,inf}//14
};

//n,e,s,w
int movement_array[15][4] = {
	{inf,14,inf,1},
	{2,0,inf,inf},
	{3,inf,1,inf},
	{4,12,2,inf},
	{5,inf,3,inf},
	{6,10,4,inf},
	{7,inf,5,inf},
	{inf,8,6,inf},
	{inf,inf,9,7},
	{8,inf,10,inf},
	{9,inf,11,5},
	{10,inf,12,inf},
	{11,inf,13,3},
	{12,inf,14,inf},
	{13,inf,inf,0}
};

/*
*
* Function Name: dijkstra
* Input: adjacency matrix, total nodes, initial node
* Output: void
* Logic: calculates the minimum distance between the initial node and every other node using weights stored in adjacency matrix
* Example Call: dijkstra(G, 32, 4); //calculates minimum distance between the 4th node and every other node based on the weights given in adjacency matrix
*
*/
void dijkstra(int G[15][15], int n, int startnode) {
	int cost[15][15], distance[15], pred[15];					//initialising cost, distance and perd arrays
	int visited[15], count, mindistance, nextnode, i, j;
	for (i = 0; i < n; i++)										//generating cost matrix
	for (j = 0; j < n; j++)
	if (G[i][j] == 0)
	cost[i][j] = infi;
	else
	cost[i][j] = G[i][j];
	for (i = 0; i < n; i++) {									//for loop to initialise visited array
		distance[i] = cost[startnode][i];
		pred[i] = startnode;
		visited[i] = 0;
	}
	distance[startnode] = 0;
	visited[startnode] = 1;
	count = 1;
	while (count < n - 1) {										//main while loop which calculates the distance using dijkstra algorithm
		mindistance = infi;
		for (i = 0; i < n; i++)
		if (distance[i] < mindistance && !visited[i]) {
			mindistance = distance[i];
			nextnode = i;
		}
		visited[nextnode] = 1;
		for (i = 0; i < n; i++)
		if (!visited[i])
		if (mindistance + cost[nextnode][i] < distance[i]) {
			distance[i] = mindistance + cost[nextnode][i];
			pred[i] = nextnode;
		}
		count++;
	}
	for (i = 0; i < n; i++)									//generating dis and pre arrays from distance and pred
	{
		dis[i] = distance[i];
	}
	for (i = 0; i < n; i++)
	{
		pre[i] = pred[i];
	}
}

void dist_comp(int x, int y, int path[], int *index)
{
	if (dis[x] <= dis[y])
	{
		if (u != x)
		{
			int j = x;
			path[*index] = x;
			*index = *index + 1;
			do {
				j = pre[j];
				path[*index] = j;
				*index = *index + 1;
			} while (j != u);
		}
		reverse(path, *index);
		u = x;

	}
	else
	{
		if (u != y)
		{
			int j = y;
			path[*index] = y;
			*index = *index + 1;
			do {
				j = pre[j];
				path[*index] = j;
				*index = *index + 1;
			} while (j != u);
		}
		reverse(path, *index);
		u = y;

	}
}

/* --------------------------------------------------------------*/

//Function Prototypes ->
unsigned char ADC_Conversion(unsigned char);
void forward_wls(int a, int node);
void left_turn_wls();
void forward_inv();
void static_reorientation();
void static_reorientation_inv();
void forward_mm(unsigned int DistanceInMM);

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
/* --------------------------------------------------------------ELECTRONIC SECTION -----------------------------------------------------------------------*/

//Configuration Functions ->
//Function to initialize Buzzer
void buzzer_pin_config(void) {
  DDRG = DDRG | 0x08; //Setting PORTG 3 as output
  PORTG = PORTG & 0xF7; //Setting PORTG 3 logic low to turnoff buzzer
}

void motion_pin_config(void) {
  DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
  PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
  DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
  PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
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
}

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

/*
 *
 * Function Name: demux_pin_config
 * Input: void
 * Output: void
 * Logic: initializes pin configuration for proper working of demux
 * We have used a demux to control two gripper servo motors
 *
 */
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
  buzzer_pin_config();
  motion_pin_config();
  left_encoder_pin_config();
  right_encoder_pin_config();
  left_position_encoder_interrupt_config();
  right_position_encoder_interrupt_config();
  servo1_pin_config();
  servo2_pin_config();
  servo3_pin_config();
  demux_pin_config();
  lcd_port_config();
  adc_pin_config();
}
/* --------------------------------------------------------------*/

//Interrupt Service Routines ->
//ISR for right position encoder
ISR(INT5_vect) {
  ShaftCountRight++; //increment right shaft position count
}

//ISR for left position encoder
ISR(INT4_vect) {
  ShaftCountLeft++; //increment left shaft position count
}
/* --------------------------------------------------------------*/

//Other Functions ->
void buzzer_off(void) {
  unsigned char port_restore = 0;
  port_restore = PING;
  port_restore = port_restore | 0x08;
  PORTG = port_restore;
}

void buzzer_on(void) {
  unsigned char port_restore = 0;
  port_restore = PING;
  port_restore = port_restore & 0xF7;
  PORTG = port_restore;
  _delay_ms(5000);
  buzzer_off();
}

/*--------------------------------------------------------------*/

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
	PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
	_delay_ms(20);
	float Pos = 0;
	Pos = ((float) degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) Pos;
}

void servo_4(unsigned char degrees) {
	PORTH &= ~(1 << 0);//Slave Servo motor demux pin set to 1
	_delay_ms(20);
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
/* --------------------------------------------------------------SENSOR SECTION------------------------------------------------------------------------*/

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
/*---------------------------------------------------------------------------LCD Section--------------------------------------------------------------------*/
/*
 *
 * Function Name: LCD_ON
 * Input: void
 * Output: void
 * Logic: prints the given ADC sensors onto the LCD
 *
 */
void LCD_ON(void) {
  print_sensor(1, 1, 1); //Prints value of White Line Sensor Left
  print_sensor(1, 7, 2); //Prints Value of White Line Sensor Center
  print_sensor(1, 14, 3); //Prints Value of White Line Sensor Right

  wall = ADC_Conversion(11); //Stores the Analog value of front sharp connected to ADC channel 11 into variable "wall"
  lcd_print(2, 1, wall, 3); //Prints Value Of Distance in MM measured by Sharp Sensor.

  sharp = ADC_Conversion(13); //Stores the Analog value of front sharp connected to ADC channel 10 into variable "sharp"
  lcd_print(2, 14, sharp, 3); //Prints Value Of Distance in MM measured by Sharp Sensor.

}

/*
 *
 * Function Name: LCD_Function
 * Input: integer a
 * Output: void
 * Logic: prints the given text onto the LCD, was used to determine which function was called at what moment
 *
 */
void LCD_Function(int a) 
{
  lcd_init();

  switch (a) 
  {
  case 0:
    lcd_cursor(1, 3);
    lcd_string("Line-Follower");
    break;

  case 1:
    lcd_cursor(1, 3);
    lcd_string("Wall-Follower");
    break;

  case 5:
    lcd_cursor(1, 3);
    lcd_string("Zig-Zag");
    break;

  
  case 6:
    lcd_cursor(1, 6);
    lcd_string(" END ");
    lcd_cursor(2, 1);
    lcd_string("-----****-----");
    break;

  case 4:
    lcd_cursor(1, 6);
    lcd_string("Static");
    lcd_cursor(2, 3);
    lcd_string("Reorientation");
    break;
  
  case 3:
    lcd_cursor(1, 3);
    lcd_string("Inversion");
	break;

  }
}
/* --------------------------------------------------------------ARM MECHANISM----------------------------------------------------------------------*/

/*
 *
 * Function Name: m_pick
 * Input: void
 * Output: void
 * Logic: uses Servo 3 (Master) to pick the CM
 *
 */
void m_pick(void) {
  servo_1(90);
  _delay_ms(750);
  servo_1_free();
  servo_3(120);
  _delay_ms(750);
  servo_3_free();
  servo_2(102);
  _delay_ms(1000);
  servo_3(20);
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);

}

/*
 *
 * Function Name: s_pick
 * Input: void
 * Output: void
 * Logic: uses Servo 4 (Slave) to pick the CM
 *
 */
void s_pick(void) {
  servo_1(67);
  _delay_ms(750);
  servo_1_free();
  servo_4(40);
  _delay_ms(1000);
  servo_4_free();
  servo_2(100);
  _delay_ms(1500);
  servo_4(160);
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);
  servo_2_free();
  servo_1(90);
  _delay_ms(750);
  servo_1_free();
}

void inv_place() {
  _delay_ms(500);
  servo_1(0);
  _delay_ms(1000);
  servo_1_free();
  servo_3(30);
  _delay_ms(750);
  servo_2(45);
  _delay_ms(1000);
  servo_3(105);
  _delay_ms(750);
  servo_3_free();
  servo_2(0);
  _delay_ms(1000);
  servo_2_free();
  servo_1(95);
  _delay_ms(1000);
  servo_1_free();
}

/*
 *
 * Function Name: m_place_lr
 * Input: void
 * Output: void
 * Logic: uses Servo 3 (Master) to place the CM to low-rise house
 *
 */
void m_place_lr(void) {
  servo_1(95);
  _delay_ms(750);
  servo_1_free();
  servo_3(30);
  _delay_ms(750);
  servo_2(65);
  _delay_ms(750);
  servo_3(105);
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);
}

/*
 *
 * Function Name: s_place_lr
 * Input: void
 * Output: void
 * Logic: uses Servo 4 (Slave) to place the CM to low-rise house
 *
 */
void s_place_lr(void) {
  servo_1(72);
  _delay_ms(750);
  servo_4(150);
  _delay_ms(750);
  servo_2(65);
  _delay_ms(1000);
  servo_4(40);
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);
  servo_1(95);
}

/*
 *
 * Function Name: s_place_hr
 * Input: void
 * Output: void
 * Logic: uses Servo 4 (Slave) to place the CM to high-rise house
 *
 */
void s_place_hr(void) {
  servo_1(72);
  _delay_ms(750);
  servo_4(150);
  _delay_ms(750);
  servo_2(35);
  _delay_ms(1000);
  servo_4(40);
  _delay_ms(750);
  servo_2(0);
  _delay_ms(1000);
  servo_1(95);
}

void m_place_hr()
{
	//write your code here
}
/* --------------------------------------------------------------NAVIGATION SECTION---------------------------------------------------------------------------*/

/*
 *
 * Function Name: forward_walls
 * Input: void
 * Output: void
 * Logic: used to navigate through the walls using sharp sensor, is called automatically by line follower " forward_wls(int a)"
 *
 */
void forward_walls() {
  LCD_Function(1);
  forward();
  velocity(base, base);
  _delay_ms(500);
  while (1) {
    wall = ADC_Conversion(11);
    if ((wall >= 115) && (wall <= 130)) {
      forward();
      OCR5AL = 220;
      OCR5BL = 220;
      _delay_ms(45);
    } else if (wall > 130) {

      soft_right();
      OCR5AL = 117;
      OCR5BL = 111;
      _delay_ms(2);
    } else if (wall < 115 && wall > 80) {
      soft_left();
      OCR5AL = 114;
      OCR5BL = 117;
      _delay_ms(2);

    } else if (wall < 80) 
	{
		stop();
      break;
    }
  }
  forward_wls(0, 1);
}

/*
 *
 * Function Name: forward_wls
 * Input: integer a
 * Output: void
 * Logic: used to navigate through the black line using line sensor, entered input determines if the function is to serve as node to node navigation and black line to wall follower
 *
 */
void forward_wls(int a, int node) {
  int n = 1;
  while (n <= node) {
    LCD_Function(0);
    forward();
    velocity(base, base);
    while (1) {
      ls = ADC_Conversion(1);
      ms = ADC_Conversion(2);
      rs = ADC_Conversion(3);
      if ((a == 2) && (ls + ms + rs > 280)) // Certain nodes on the edge of the arena allow only two sensor to stand on them and hence have a different threshold than standard nodes
      {
	      _delay_ms(80);
	      break;
      }
	  else if ( a == 3)
      {
	      if (ls > 120 && rs > 120)
	      {
		      stop();
		      _delay_ms(100);
		      break;
	      }
      } 
	  else if (a == 1) // Condition to invoke forward_walls
	  {
		  wall = ADC_Conversion(11);

		  if ((wall > 75) && (ls < 80) && (rs < 80))

		  {

			  stop();
			  _delay_ms(100);
			  break;

		  }
	  }
	  else if ((a == 0) && (ls + ms + rs > 400)) // Standard nodes threshold
      { 
        break;
      } 
	  else if ((ls < 60 && ms >= 125 && rs < 60)) // Motor speed is changed directly to adjust the robot rather than calling left or right to increase smoothness in motion
      { // Velocity function was not called and values were configured directly as function calling was increasing bot response time in while(1) loop

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
      } 

    }
    if (a == 1) {
      forward_walls();
    }
    if (a == 3) {
      forward_inv();

    }
    n++;
  }
  PORTA = 0x00;
  stop();
}

void forward_inv()
{
	unsigned char w = 0;
	
	LCD_Function(3);
	lcd_print(2,7,w,2);
	
	
	while (1)
	{
		forward();
		velocity(base, base);
		
		ls = ADC_Conversion(1);
		ms = ADC_Conversion(2);
		rs = ADC_Conversion(3);
		if ( ls+ms+rs < 330 && ms <90 && w==0)
		{
			
			_delay_ms(100);
			++w;
			lcd_print(2,7,w,2);
			back();
			_delay_ms(250);
			stop();
			inv_place();
			forward();
			_delay_ms(100);
			continue;
			
			
		}
		else if ((ls > 180 && ms < 130 && rs > 180)) // Motor speed is changed directly to adjust the robot rather than calling left or right to increase smoothness in motion
		{                                           // Velocity function was not called and values were configured directly as function calling was increasing bot response time in while(1) loop
			
			OCR5AL = base;
			OCR5BL = base;
		}
		else if ((ls < 80 && ms > 130 && rs > 150))
		{   
			soft_left();
			OCR5AL = turn-50;
			OCR5BL = base;
			_delay_ms(1);
			
		}
		else if (rs < 150 && ms > 130 && ls > 80)
		{
			soft_right();
			OCR5AL = base;
			OCR5BL = turn-50;
			_delay_ms(1);
			
		}
		else if( rs< 100 && ms > 100 && ls < 100)
		{
			break;
		}
		
		
	}
	
	forward_wls(2,1);
}


/*
 *
 * Function Name: static_reorientation
 * Input: void
 * Output: void
 * Logic: used to aling the robot to the black line
 *
 */
void static_reorientation() {
  LCD_Function(4);
  ls = ADC_Conversion(1);
  ms = ADC_Conversion(2);
  rs = ADC_Conversion(3);
  if ((ls < 70 && ms >= 110 && rs < 70)) {
    PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function 
  } else {
    OCR5AL = 150;
    OCR5BL = 150;
    stop();
    right();
    _delay_ms(280);
    stop();
    left_turn_wls();
  }
}

void static_reorientation_inv() {
  LCD_Function(4);
  ls = ADC_Conversion(1);
  ms = ADC_Conversion(2);
  rs = ADC_Conversion(3);
  if ((ls > 150 && ms <= 70 && rs < 150)) {
    PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
  } else {
    OCR5AL = 155;
    OCR5BL = 155;
    stop();
    _delay_ms(10);
    right();
    _delay_ms(250);
    stop();
    left();
    OCR5AL = 100;
    OCR5BL = 100;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {
      ms = ADC_Conversion(1);
      if (ms <= 70) {
        PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
        break;
      }
    }
    left();
    _delay_ms(30);
    stop();
    _delay_ms(200);
    OCR5AL = base;
    OCR5BL = base;
  }
}
void forward_zigzag()
{
	LCD_Function(5);
	
	
	while (1)
	{
		ls = ADC_Conversion(1);
		ms = ADC_Conversion(2);
		rs = ADC_Conversion(3);
		forward();
		velocity(250, 250);
		
		if ((ls + ms + rs) > 400)
		{
			stop();
			//_delay_ms(100);
			break;
		}
		
		else	if ( ms > 100)
		{
			forward();
			OCR5AL = 250;
			OCR5BL = 250;
			_delay_ms(5);
		}
		else if (ls < 20 && ms < 20 && rs < 20)
		{
			
			static_reorientation();
			
		}
		else if (ls > ms)
		{
			left();
			OCR5AL = 180;
			OCR5BL = 160;
			_delay_ms(1);
			
		}
		else if (ms < rs)
		{
			right();
			OCR5AL = 160;
			OCR5BL = 180;
			_delay_ms(1);
			

		}
		
		
		
	}
	
	stop();
	_delay_ms(100);
}

/*----------------------------------------------------------TURNING MECHANISM ---------------------------------------------------------------*/
/*
 *
 * Function Name: right_turn_wls
 * Input: void
 * Output: void
 * Logic: used to turn the robot to the right using the line sensor
 *
 */
void right_turn_wls(void) {

  forward();
  _delay_ms(250);
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
      PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function 
      break;
    }
  }
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;

}

void right_turn_inv(void) {

  forward();
  _delay_ms(200);
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
    ms = ADC_Conversion(2);
    ls = ADC_Conversion(1);
    if (rs >= 80 && ms < 80) {
      PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
      break;
    }
  }
  right();
  _delay_ms(200);
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;
  static_reorientation_inv();

}

void right_turn_wls_bwall(void) {

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
      PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function 
      break;
    }
  }
  right();
  _delay_ms(5);
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;

}

void right_turn(void)
{
	forward_mm(80);
	stop();
	right();
	velocity(120,120);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees /4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			ls = ADC_Conversion(1);
			if (ls > 80)
			{
				break;
			}
		}
		
	}
	stop(); //Stop robot
}
/*
 *
 * Function Name: left_turn_wls
 * Input: void
 * Output: void
 * Logic: used to turn the robot to the left using the line sensor
 *
 */
void left_turn_wls(void) {
	forward();
	_delay_ms(250);
  left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
  _delay_ms(200);
  stop();
  _delay_ms(50);
  left();
  OCR5AL = 100;
  OCR5BL = 100;
  while (1) //while loop which detects black line using middle sensor so that the robot stops turning
  {
    ms = ADC_Conversion(1);
    if (ms >= 80) {
      PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function 
      break;
    }
  }
  stop();
  _delay_ms(200);
  OCR5AL = base;
  OCR5BL = base;

}

void left_turn_inv(void) {

	forward();
	_delay_ms(200);
	left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
	_delay_ms(200);
	stop();
	_delay_ms(50);
	left();
	OCR5AL = 100;
	OCR5BL = 100;
	while (1) //while loop which detects black line using middle sensor so that the robot stops turning
	{
		rs = ADC_Conversion(3);
		ms = ADC_Conversion(2);
		ls = ADC_Conversion(1);
		if (ls >= 80 && ms < 80) {
			PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
			break;
		}
	}
	left();
	_delay_ms(200);
	stop();
	_delay_ms(200);
	OCR5AL = base;
	OCR5BL = base;
	static_reorientation_inv();

}


void left_turn(void)
{
	forward_mm(80);
	stop();
	left();
	velocity(120,120);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees /4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			ls = ADC_Conversion(1);
			if (ls > 80)
			{
				break;
			}
		}
		
	}
	stop(); //Stop robot

}

void left_turn_wls_bwall(void)
{
	left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
	_delay_ms(200);
	stop();
	_delay_ms(50);
	left();
	OCR5AL = 100;
	OCR5BL = 100;
	while (1) //while loop which detects black line using middle sensor so that the robot stops turning
	{
		ls = ADC_Conversion(1);
		if (ls >= 80) {
			PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
			break;
		}
	}
	left();
	_delay_ms(5);
	stop();
	_delay_ms(200);
	OCR5AL = base;
	OCR5BL = base;
	
}






/* --------------------------------------------------------------POSITION CONTROL INTERRUPTS----------------------------------------------*/



//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees * 3.8033; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM * 2.1888; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}




/* -------------------------------------------------------------INITIALIZATION--------------------------------------------------*/

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
int which_node(int block)
{
	if (block == 1) return b1n;
	else if (block == 2) return b2n;
	else if (block == 3) return b3n;
	else if (block == 4) return b4n;
	else if (block == 5) return b5n;
	else if (block == 6) return b6n;
	else if (block == 7) return b7n;
	else if (block == 8) return b8n;
	else if (block == 9) return b9n;
	else if (block == 10) return b10n;
	else if (block == 11) return b11n;
	else if (block == 12) return b12n;
	return -1;
}

/*
*
* Function Name: traverse
* Input: array, current face direction, initial node
* Output: void
* Logic: uses direction knowledge and the distance array generated by dijkstra to traverse to any node from the current node through shortest path
* Example Call: traverse(path,n,12); //uses direction knowledge and the distance array generated by dijkstra to traverse to any node from the current node through shortest path
*
*/
void traverse(int path[], char face, int u, int *size)
{
	int ps = *size;
	for (int i = 0; i < ps - 1; i++)
	{
		if ((path[i] == 0 && path[i + 1] == 1) || (path[i] == 0 && path[i + 1] == 14))
		{
			for (int a = 0; a < 4; a++)
			{
				if (movement_array[path[i]][a] == path[i + 1])
				{
					if (a == 0)
					{
						fdir = 'n';
					}
					else if (a == 1)
					{
						fdir = 'e';
					}
					else if (a == 2)
					{
						fdir = 's';
					}
					else if (a == 3)
					{
						fdir = 'w';
					}
				}
			}
			if (face == 'n' && fdir == 'n')
			{
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 'e')
			{
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 's')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 'w')
			{
				left_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'e')
			{
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 's')
			{
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'w')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'n')
			{
				left_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 's')
			{
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'w')
			{
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'n')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'e')
			{
				left_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'w')
			{
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'n')
			{
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'e')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 's')
			{
				left_turn_wls();
				forward_wls(2,1);
				face = fdir;
			}
		}
		else
		{
			for (int a = 0; a < 4; a++)
			{
				if (movement_array[path[i]][a] == path[i + 1])
				{
					if (a == 0)
					{
						fdir = 'n';
					}
					else if (a == 1)
					{
						fdir = 'e';
					}
					else if (a == 2)
					{
						fdir = 's';
					}
					else if (a == 3)
					{
						fdir = 'w';
					}
				}
			}
			if (face == 'n' && fdir == 'n')
			{
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 'e')
			{
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 's')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'n' && fdir == 'w')
			{
				left_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'e')
			{
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 's')
			{
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'w')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'e' && fdir == 'n')
			{
				left_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 's')
			{
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'w')
			{
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'n')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 's' && fdir == 'e')
			{
				left_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'w')
			{
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'n')
			{
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 'e')
			{
				right_turn_wls();
				right_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			else if (face == 'w' && fdir == 's')
			{
				left_turn_wls();
				forward_wls(0,1);
				face = fdir;
			}
			
		}
	}
}

void block_traverse(char face,char fdir)
{
	if (face == 'n' && fdir == 'n')
	{
		face = fdir;
	}
	else if (face == 'n' && fdir == 'e')
	{
		right_turn_wls();
		face = fdir;
	}
	else if (face == 'n' && fdir == 's')
	{
		right_turn_wls();
		right_turn_wls_bwall();
		face = fdir;
	}
	else if (face == 'n' && fdir == 'w')
	{
		left_turn_wls();
		face = fdir;
	}
	else if (face == 'e' && fdir == 'e')
	{
		face = fdir;
	}
	else if (face == 'e' && fdir == 's')
	{
		right_turn_wls();
		face = fdir;
	}
	else if (face == 'e' && fdir == 'w')
	{
		right_turn_wls();
		right_turn_wls_bwall();
		face = fdir;
	}
	else if (face == 'e' && fdir == 'n')
	{
		left_turn_wls();
		face = fdir;
	}
	else if (face == 's' && fdir == 's')
	{
		face = fdir;
	}
	else if (face == 's' && fdir == 'w')
	{
		right_turn_wls();
		face = fdir;
	}
	else if (face == 's' && fdir == 'n')
	{
		right_turn_wls();
		right_turn_wls_bwall();
		face = fdir;
	}
	else if (face == 's' && fdir == 'e')
	{
		left_turn_wls();
		face = fdir;
	}
	else if (face == 'w' && fdir == 'w')
	{
		face = fdir;
	}
	else if (face == 'w' && fdir == 'n')
	{
		right_turn_wls();
		face = fdir;
	}
	else if (face == 'w' && fdir == 'e')
	{
		right_turn_wls();
		right_turn_wls_bwall();
		face = fdir;
	}
	else if (face == 'w' && fdir == 's')
	{
		left_turn_wls();
		face = fdir;
	}
}

void block_choose(int bnode,int block_placed[],char face,char fdir)
{
	if (bnode == 2)
	{
		if (block_placed[0] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[0] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[1] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[1] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
	else if (bnode == 13)
	{
		if (block_placed[2] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[2] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[3] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[3] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
	else if (bnode == 4)
	{
		if (block_placed[4] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[4] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[5] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[5] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
	else if (bnode == 11)
	{
		if (block_placed[6] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[6] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[7] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[7] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
	else if (bnode == 6)
	{
		if (block_placed[8] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[8] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[9] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[9] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
	else if (bnode == 9)
	{
		if (block_placed[10] == 0)
		{
			fdir = 'w';
			block_traverse(face,fdir);
			block_placed[10] = 1;
			back();
			_delay_ms(200);
			stop();
		}
		else if (block_placed[11] == 0)
		{
			fdir = 'e';
			block_traverse(face,fdir);
			block_placed[11] = 1;
			back();
			_delay_ms(200);
			stop();
		}
	}
}
/* --------------------------------------------------------------MAIN-----------------------------------------------------------*/

int main() 
{
  init_devices();
  //int house_no;
  int block;
  int bnode;
  int len = 0;
  char apfdir = 'w';
  u = 0;
  face = 's';
  
  block = which_material[8];
  bnode = which_node(block);
  dijkstra(G,n,u);
  int *path = (int*)malloc(80);
  dist_comp(bnode,bnode,path,&len);
  traverse(path,face,u,&len);
  free(path);
  apfdir=face;
  block_choose(bnode,block_placed,face,fdir);
  m_pick();
  block_traverse(face,apfdir);
  u=bnode;
  
  block = which_material[9];
  bnode = which_node(block);
  dijkstra(G,n,u);
  //int *path = (int*)malloc(80);
  dist_comp(bnode,bnode,path,&len);
  traverse(path,face,u,&len);
  free(path);
  apfdir=face;
  block_choose(bnode,block_placed,face,fdir);
  m_pick();
  block_traverse(face,apfdir);
  u=bnode;
  
  dijkstra(G,n,u);
  //int *path = (int*)malloc(80);
  dist_comp(7,8,path,&len);
  //right_turn_wls();
  //u = 0;
  //face = 'w';
  //forward_wls(2,1);
  //u = 1;
  //face = 'w';
  //right_turn_wls();
  //u = 1;
  //face = 'n';
  //forward_wls(0,1);
  //u = 2;
  //face = 'n';
  //left_turn_wls();
  //u = 2;
  //face = 'e';
  //for (int i = 0; i < 10; i++)
  //{
	  //if (1 == which_material[i])
	  //{
		  //house_no = i;
	  //}
  //}
}
/* --------------------------------------------------------------*/