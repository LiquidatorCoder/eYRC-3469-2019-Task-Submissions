
/*
*
* Team Id: 3469
* Author List: Abhay Maurya, Ratnesh Mohan, Shubhankar Jain, Saumya Gupta
* Filename: task5-main.c
* Theme: Construct-O-Bot - Specific to eYRC
* Functions: block_choose2, timer1_init, timer5_init, buzzer_pin_config, motion_pin_config, servo1_pin_config, servo2_pin_config, buzzer_off, buzzer_on,
* servo3_pin_config, demux_pin_config, adc_pin_config, port_init, motion_set, back, forward, right, left, soft_right,
* soft_left, soft_right_2, soft_left_2, stop, velocity, servo_1, servo_2, servo_3, servo_4, servo_1_free, servo_2_free, servo_3_free,
* servo_4_free, ADC_Conversion, print_sensor, Sharp_GP2D12_estimation, LCD_ON, LCD_Function, m_pick, s_pick, m_place_lr, s_place_lr,
* s_place_hr, forward_walls, forward_wls, left_turn_wls, left_turn_wls_bwall, static_reorientation, right_turn_wls, right_turn_wls_bwall, forward_untw,
* forward_zigzag, Wall_run, adc_init, init_devices, block_traverse, check_block, dijkstra, dist_comp, inv_place1, inv_place2, place_hr, place_lr,
* reverse, static_reorientation_inv, traverse, which_block, which_house, which_node;
* Global Variables: ADC_Value, adc_reading, sharp, distance, wall, value, base, turn, soft, ls, rs, ms, is_block1, is_block2, is_block3, is_block4,
* is_block5, is_block6, house_no, block, bnode, b1n, b2n, b3n, b4n, b5n, b6n, b7n, b8n, b9n, b10n, b11n, b12n, n, u, face, fdir,
* house_config, house_req, which_material, dis, pre;
*
*/

//Preprocessor Directives ->
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#define inf 0
#define max 15
#define infi 9999

/* --------------------------------------------------------------*/

//Function Prototypes ->
unsigned char ADC_Conversion(unsigned char);
void forward_wls(int a, int node);
void forward (void);
void left_turn_wls();
void right_turn_wls();
void right_turn_wls_bwall();
void left_turn_wls_bwall();
void forward_inv();
void forward_zigzag();
void back();
void stop();
void static_reorientation();
void static_reorientation_inv();
void forward_mm(unsigned int DistanceInMM);
void m_pick();
void s_pick();
void object_detect(void);
void velocity(unsigned char left_motor, unsigned char right_motor);
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
/* ------------------------------------------------------Global Variables----------------------------------------------------------*/



/* --------------------------------------------------------CHANGE THESE CONFIGURATION----------------------------------*/
int n = 15;							//total node count and initial node
int u = 0;
char face = 'w';					//direction variables which include directions w,n,e,s
char fdir = 'n';
int house_config[5] = {0,1,1,0,0};
int house_req[5] = {2,2,2,2,2};
int which_material[10] = {9,11,5,4,12,7,1,3,2,8};
int dis[15];
int pre[15];
int b1n = 2;
int b2n = 2;
int b3n = 13;
int b4n = 13;			// storing nodes for each block
int b5n = 4;
int b6n = 4;
int b7n = 11;
int b8n = 11;
int b9n = 6;
int b10n = 6;
int b11n = 9;
int b12n = 9;

int h1n = 3;			//storing nodes for each house
int h2n = 12;
int h3n = 5;
int h4n = 10;
int h51 = 7;
int h52 = 8;

int block_placed[12] = {0,0,0,0,0,0,0,0,0,0,0,0};				//array to check if block is placed or not

//Global Variables ->
int is_block1 = 0;
int is_block2 = 0;
int is_block3 = 0;
int is_block4 = 0;
int is_block5 = 0;
int is_block6 = 0;
int house_no = 0;
int block = 0;
int bnode;
unsigned char ADC_Value, adc_reading;
unsigned char sharp, distance, wall; //ADC Output from Sharp sensor
unsigned int value;
unsigned char base = 250; //base velocity of motor
unsigned char turn = 185; //turn velocity of motor
unsigned char soft = 205; //soft turn velocity of motor
unsigned char ls, ms, rs; //ADC Output from line sensors
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
int inv = 0; //Used to call place function for F5

/* --------------------------------------------------------------------DIJKSTRA---------------------------------------------------------------------------*/
// Macros for different weights of branches of graph
#define wf 6
#define zz 6
#define in 8

/*
*
* Function Name: reverse
* Input: array,length of array
* Output: void
* Logic: swapping of elements
* Example Call: reverse(a,n) //name of array is a and length is n
*
*/
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
/*
*
* Function Name: check_block
* Input: no of block(w1,w2.......)
* Output: int
* logic: checks if the block is to be place or not
*
*/
int check_block(int block)
{
    int i;
    for(i = 0; i <10; i++)
    {
        if(which_material[i] == block)
            return 1;
    }
    return 0;
}
// Non-Directional weighted adjacency matrix for the given graph
int G[15][15] =
{
    // 0  1	  2   3   4	  5	  6	  7	  8	  9	 10	 11	 12	 13	 14
    {inf,4,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,4},//0
    {4,inf,1,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf},//1
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
    {4,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,1,inf}//14
};

//contains the node which is in north, east, south, west of the given index node
int movement_array[15][4] =
{
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
void dijkstra(int G[15][15], int n, int startnode)
{
    int cost[15][15], distance[15], pred[15];					//initialising cost, distance and perd arrays
    int visited[15], count, mindistance, nextnode, i, j;
    for (i = 0; i < n; i++)										//generating cost matrix
        for (j = 0; j < n; j++)
            if (G[i][j] == 0)
                cost[i][j] = infi;
            else
                cost[i][j] = G[i][j];
    for (i = 0; i < n; i++)  									//for loop to initialise visited array
    {
        distance[i] = cost[startnode][i];
        pred[i] = startnode;
        visited[i] = 0;
    }
    distance[startnode] = 0;
    visited[startnode] = 1;
    count = 1;
    while (count < n - 1)  										//main while loop which calculates the distance using dijkstra algorithm
    {
        mindistance = infi;
        for (i = 0; i < n; i++)
            if (distance[i] < mindistance && !visited[i])
            {
                mindistance = distance[i];
                nextnode = i;
            }
        visited[nextnode] = 1;
        for (i = 0; i < n; i++)
            if (!visited[i])
                if (mindistance + cost[nextnode][i] < distance[i])
                {
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

/*
*
* Function Name: dist_comp
* Input: integer which stores the value of first node,int which stores the value of second node,integer array,int
* Output: int
* logic: checks if the block is to be place or not
*
*/
void dist_comp(int x, int y, int path[], int *index)
{
    if (dis[x] <= dis[y])
    {
        if (u != x)
        {
            int j = x;
            path[*index] = x;
            *index = *index + 1;
            do
            {
                j = pre[j];
                path[*index] = j;
                *index = *index + 1;
            }
            while (j != u);
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
            do
            {
                j = pre[j];
                path[*index] = j;
                *index = *index + 1;
            }
            while (j != u);
        }
        reverse(path, *index);
        u = y;

    }
}
/*
*
* Function Name: which_node
* Input: block number
* Output: int
* logic: returns the nearest node of the block
*
*/s
int which_node(int block)
{
    if (block == 1)
        return b1n;
    else if (block == 2)
        return b2n;
    else if (block == 3)
        return b3n;
    else if (block == 4)
        return b4n;
    else if (block == 5)
        return b5n;
    else if (block == 6)
        return b6n;
    else if (block == 7)
        return b7n;
    else if (block == 8)
        return b8n;
    else if (block == 9)
        return b9n;
    else if (block == 10)
        return b10n;
    else if (block == 11)
        return b11n;
    else if (block == 12)
        return b12n;
    return -1;
}
/*
*
* Function Name: which_block
* Input: node number
* Output: int
* logic: returns the block number using the direction of robot
*
*/
int which_block(int node)
{
    if ((node == b1n) && (face == 'w'))
        return 1;
    else if ((node == b2n) && (face == 'e'))
        return 2;
    else if ((node == b3n) && (face == 'w'))
        return 3;
    else if ((node == b4n) && (face == 'e'))
        return 4;
    else if ((node == b5n) && (face == 'w'))
        return 5;
    else if ((node == b6n) && (face == 'e'))
        return 6;
    else if ((node == b7n) && (face == 'w'))
        return 7;
    else if ((node == b8n) && (face == 'e'))
        return 8;
    else if ((node == b9n) && (face == 'w'))
        return 9;
    else if ((node == b10n) && (face == 'e'))
        return 10;
    else if ((node == b11n) && (face == 'w'))
        return 11;
    else if ((node == b12n) && (face == 'e'))
        return 12;
    return -1;
}
/*
*
* Function Name: which_house
* Input: house number
* Output: int
* logic: returns the node number of the house
*
*/
int which_house(int house)
{
    if (house == 1)
        return h1n;
    else if (house == 2)
        return h2n;
    else if (house == 3)
        return h3n;
    else if (house == 4)
        return h4n;
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
void traverse(int path[], int u, int *size)
{
    int ps = *size;
    for (int i = 0; i < ps - 1; i++)            //condition for corner nodes
    {
        if (((path[i] == 0 && path[i + 1] == 1) || (path[i] == 0 && path[i + 1] == 14)) || ((path[i] == 9 && path[i + 1] == 8) || (path[i] == 6 && path[i + 1] == 7)) || ((path[i] == 13 && path[i + 1] == 14) || (path[i] == 2 && path[i + 1] == 1)) || ((path[i] == 14 && path[i + 1] == 0) || (path[i] == 1 && path[i + 1] == 0)))
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
            if (face == 'n' && fdir == 'n')     //further else if clauses try to align the robot to the final direction fdir from the face direction
            {
                ////printf("t 1\n");
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'e')
            {
                ////printf("t 2\n");
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 's')
            {
                ////printf("t 3\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'w')
            {
                ////printf("t 4\n");
                left_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'e')
            {
                ////printf("t 5\n");
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 's')
            {
                ////printf("t 6\n");
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'w')
            {
                ////printf("t 7\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'n')
            {
                ////printf("t 8\n");
                left_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 's')
            {
                ////printf("t 9\n");
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'w')
            {
                ////printf("t 10\n");
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'n')
            {
                ////printf("t 11\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'e')
            {
                ////printf("t 12\n");
                left_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'w')
            {
                ////printf("t 13\n");
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'n')
            {
                ////printf("t 14\n");
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'e')
            {
                ////printf("t 15\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 's')
            {
                ////printf("t 16\n");
                left_turn_wls();
                forward_wls(2,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
        }            //condition for wall following
        else if ((path[i] == 3 && path[i + 1] == 12) || (path[i] == 12 && path[i + 1] == 3))
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
                ////printf("wf 1\n");
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'e')
            {
                ////printf("wf 2\n");
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 's')
            {
                ////printf("wf 3\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'w')
            {
                ////printf("wf 4\n");
                left_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'e')
            {
                ////printf("wf 5\n");
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 's')
            {
                ////printf("wf 6\n");
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'w')
            {
                ////printf("wf 7\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'n')
            {
                ////printf("wf 8\n");
                left_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 's')
            {
                ////printf("wf 9\n");
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'w')
            {
                ////printf("wf 10\n");
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'n')
            {
                ////printf("wf 11\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'e')
            {
                ////printf("wf 12\n");
                left_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'w')
            {
                ////printf("wf 13\n");
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'n')
            {
                ////printf("wf 14\n");
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'e')
            {
                ////printf("wf 15\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 's')
            {
                ////printf("wf 16\n");
                left_turn_wls();
                forward_wls(1,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
        }                   //condition for zigzag path
        else if ((path[i] == 10 && path[i + 1] == 5) || (path[i] == 5 && path[i + 1] == 10))
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
                ////printf("wf 1\n");
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'e')
            {
                ////printf("wf 2\n");
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 's')
            {
                ////printf("wf 3\n");
                right_turn_wls();
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'w')
            {
                ////printf("wf 4\n");
                left_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'e')
            {
                ////printf("wf 5\n");
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 's')
            {
                ////printf("wf 6\n");
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'w')
            {
                ////printf("wf 7\n");
                right_turn_wls();
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'n')
            {
                ////printf("wf 8\n");
                left_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 's')
            {
                ////printf("wf 9\n");
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'w')
            {
                ////printf("wf 10\n");
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'n')
            {
                ////printf("wf 11\n");
                right_turn_wls();
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'e')
            {
                ////printf("wf 12\n");
                left_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'w')
            {
                ////printf("wf 13\n");
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'n')
            {
                ////printf("wf 14\n");
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'e')
            {
                ////printf("wf 15\n");
                right_turn_wls();
                right_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 's')
            {
                ////printf("wf 16\n");
                left_turn_wls();
                forward_zigzag();
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
        }

        else            //condition for standard line following
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
                ////printf("t 1\n");
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'e')
            {
                ////printf("t 2\n");
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 's')
            {
                ////printf("t 3\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'n' && fdir == 'w')
            {
                ////printf("t 4\n");
                left_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'e')
            {
                ////printf("t 5\n");
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 's')
            {
                ////printf("t 6\n");
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'w')
            {
                ////printf("t 7\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'e' && fdir == 'n')
            {
                ////printf("t 8\n");
                left_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 's')
            {
                ////printf("t 9\n");
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'w')
            {
                ////printf("t 10\n");
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'n')
            {
                ////printf("t 11\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 's' && fdir == 'e')
            {
                ////printf("t 12\n");
                left_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'w')
            {
                ////printf("t 13\n");
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'n')
            {
                ////printf("t 14\n");
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 'e')
            {
                ////printf("t 15\n");
                right_turn_wls();
                right_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }
            else if (face == 'w' && fdir == 's')
            {
                ////printf("t 16\n");
                left_turn_wls();
                forward_wls(0,1);
                face = fdir;
                lcd_cursor(2, 7);
                lcd_wr_char(face);
            }

        }
    }
}

/*
*
* Function Name: block_traverse
* Input: void
* Output: void
* Logic: to face in the direction of the block which is to be picked and to call object detect
*
*/
void block_traverse()
{
    if (face == 'n' && fdir == 'n')
    {
        ////printf("bt 1\n");
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'n' && fdir == 'e')
    {
        ////printf("bt 2\n");
        right_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'n' && fdir == 's')
    {
        ////printf("bt 3\n");
        right_turn_wls();
        right_turn_wls_bwall();
        face = fdir;
    }
    else if (face == 'n' && fdir == 'w')
    {
        ////printf("bt 4\n");
        left_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'e' && fdir == 'e')
    {
        ////printf("bt 5\n");
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'e' && fdir == 's')
    {
        ////printf("bt 6\n");
        right_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'e' && fdir == 'w')
    {
        ////printf("bt 7\n");
        right_turn_wls();
        right_turn_wls_bwall();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'e' && fdir == 'n')
    {
        ////printf("bt 8\n");
        left_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 's' && fdir == 's')
    {
        ////printf("bt 9\n");
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 's' && fdir == 'w')
    {
        ////printf("bt 10\n");
        right_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 's' && fdir == 'n')
    {
        ////printf("bt 11\n");
        right_turn_wls();
        right_turn_wls_bwall();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 's' && fdir == 'e')
    {
        ////printf("bt 12\n");
        left_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'w' && fdir == 'w')
    {
        ////printf("bt 13\n");
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'w' && fdir == 'n')
    {
        ////printf("bt 14\n");
        right_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'w' && fdir == 'e')
    {
        ////printf("bt 15\n");
        right_turn_wls();
        right_turn_wls_bwall();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    else if (face == 'w' && fdir == 's')
    {
        ////printf("bt 16\n");
        left_turn_wls();
        face = fdir;
        lcd_cursor(2, 7);
        lcd_wr_char(face);
    }
    stop();
    _delay_ms(50);
}

/*
*
* Function Name: block_choose2
* Input: block number to pick and block placed array
* Output: void
* Logic: Positions the robot so as to pick the given block
*
*/
void block_choose2(int block,int block_placed[])
{
    if (block%2 == 0)
    {
        fdir = 'e';
        block_traverse();
        block_placed[block-1] = 1;
        velocity(base,base);
        stop();
        _delay_ms(100);
        back();
        _delay_ms(200);
        stop();
        _delay_ms(100);
        static_reorientation(); // Aligning with black line
        velocity(base,base);

    }
    else if (block%2 == 1)
    {
        fdir = 'w';
        block_traverse();
        block_placed[block-1] = 1;
        velocity(base,base);
        stop();
        _delay_ms(100);
        back();
        _delay_ms(200);
        stop();
        _delay_ms(100);
        if (block == 1)
        {
            back();
            _delay_ms(50);
            stop();
            _delay_ms(100);
        }
       static_reorientation();// Aligning with black line
        velocity(base,base);
    }
}
/* -------------------------------------------Timers--------------------------------------------------*/

void timer1_init(void)
{
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

void timer5_init()
{
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
void buzzer_pin_config(void)
{
    DDRG = DDRG | 0x08; //Setting PORTG 3 as output
    PORTG = PORTG & 0xF7; //Setting PORTG 3 logic low to turnoff buzzer
}

void motion_pin_config(void)
{
    DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
    PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
    DDRL = DDRL | 0x18; //Setting PL3 and PL4 pins as output for PWM generation
    PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config(void)
{
    DDRB = DDRB | 0x20; //making PORTB 5 pin output
    PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config(void)
{
    DDRB = DDRB | 0x40; //making PORTB 6 pin output
    PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config(void)
{
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
//Configure PORTH 3 pin for demux operation
void demux_pin_config(void)
{
    DDRH = DDRH | 0xFF; //making PORTH pin output
    PORTH &= ~(1 << 0);
}

//ADC pin configuration
void adc_pin_config(void)
{
    DDRF = 0x00; //set PORTF direction as input
    PORTF = 0x00; //set PORTF pins floating
    DDRK = 0x00; //set PORTK direction as input
    PORTK = 0x00; //set PORTK pins floating
}
/* --------------------------------------------------------------*/

//Port Initialization Function ->
void port_init(void)
{
    buzzer_pin_config();
    motion_pin_config();
    servo1_pin_config();
    servo2_pin_config();
    servo3_pin_config();
    demux_pin_config();
    lcd_port_config();
    adc_pin_config();
}
/* --------------------------------------------------------------*/

//Other Functions ->
void buzzer_off(void)
{
    unsigned char port_restore = 0;
    port_restore = PING;
    port_restore = port_restore | 0x08;
    PORTG = port_restore;
}

void buzzer_on(void)
{
    unsigned char port_restore = 0;
    port_restore = PING;
    port_restore = port_restore & 0xF7;
    PORTG = port_restore;
    _delay_ms(5000);
    buzzer_off();
}

/*---------------------------------MOVEMENT FUNCTIONS SECTION-----------------------------------------------*/

//Function used for setting motor's direction
void motion_set(unsigned char Direction)
{
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

// Function for robot velocity control
void velocity(unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char) left_motor;
    OCR5BL = (unsigned char) right_motor;
}
/* -------------------------------------------------------------SERVO CONFIGURATION--------------------------------------------------------------------*/

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
    float Pos = 0;
    Pos = ((float) degrees / 1.86) + 35.0;
    OCR1AH = 0x00;
    OCR1AL = (unsigned char) Pos;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
    float Pos = 0;
    Pos = ((float) degrees / 1.86) + 35.0;
    OCR1BH = 0x00;
    OCR1BL = (unsigned char) Pos;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
    float Pos = 0;
    Pos = ((float) degrees / 1.86) + 35.0;
    OCR1CH = 0x00;
    OCR1CL = (unsigned char) Pos;
}
//Function to rotate Servo 4 by a specified angle in the multiples of 1.86 degrees
void servo_4(unsigned char degrees)
{
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
/* --------------------------------------------------------------SENSOR SECTION------------------------------------------------------------------------------------*/

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
    unsigned char a;
    if (Ch > 7)
    {
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
void print_sensor(char row, char coloumn, unsigned char channel)
{
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, coloumn, ADC_Value, 3);
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
    float distance;
    unsigned int distanceInt;
    distance = (int)(10.00 * (2799.6 * (1.00 / (pow(adc_reading, 1.1546)))));
    distanceInt = (int) distance;
    if (distanceInt > 800)
    {
        distanceInt = 800;
    }
    return distanceInt;
}
/*---------------------------------------------------------------------------LCD SECTION--------------------------------------------------------------------*/
/*
*
* Function Name: LCD_ON
* Input: void
* Output: void
* Logic: prints the given ADC sensors onto the LCD
*
*/
void LCD_ON(void)
{
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
* Example : m_pick()
*/

void m_pick(void)
{
    unsigned char i = 0;
	PORTH &= 0x00;
    PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
    _delay_ms(100);
    _delay_ms(100);
    servo_1(95);							//Base Motor set for m_pick
    _delay_ms(750);
    servo_1_free();
    _delay_ms(100);
    servo_3(120);							//master gripper open for m_pick
    _delay_ms(750);
    for (i = 0; i <137; i++)							//Arm Servo moved down for m_pick
    {
        servo_2(i);
        _delay_ms(6);
    }
    _delay_ms(100);
    servo_3(0);									//Master Gripper closed for m_pick
    _delay_ms(750);
    servo_2(12);							//Arm Servo moved up after m_pick
    _delay_ms(1000);
velocity(base,base);             //Restoring Velocity before movement

}
/*
*
* Function Name: s_pick
* Input: void
* Output: void
* Logic: uses Servo 4 (Slave) to pick the CM
* Example : s_pick()
*/
void s_pick(void)
{
    unsigned char i = 0;
    PORTH |= (1 << 3);  //Master Servo motor demux pin set to 0
    _delay_ms(100);
    _delay_ms(100);
    servo_1(70);							//Base Motor set for s_pick
    _delay_ms(750);
    servo_1_free();
    _delay_ms(100);
    servo_3(90);							//master gripper open for s_pick
    _delay_ms(750);
    for (i = 0; i <137; i++)							//Arm Servo moved down for s_pick
    {
        servo_2(i);
        _delay_ms(6);
    }
    _delay_ms(100);
    servo_3(180);									//Master Gripper closed for s_pick
    _delay_ms(750);
    servo_2(12);							//Arm Servo moved up after s_pick
    _delay_ms(1000);
    servo_1(90);
    _delay_ms(750);
    servo_1_free();
	velocity(base,base);     //Restoring Velocity before movement
}

/*
*
* Function Name: inv_place_1
* Input: void
* Output: void
* Logic: uses Servo 3 (Master) and servo 4(slave) to place the CM to special low-rise house in inversion path
*
*/
void inv_place_1()
{
    velocity(base,base);       //Restoring Velocity before movement
    right();
    _delay_ms(65);
    stop();
	_delay_ms(150);
    servo_1(5);
    _delay_ms(1000);
    servo_1_free();
    servo_2(90);
    _delay_ms(1000);

	PORTH &= 0x00;
    PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
    _delay_ms(100);

    servo_3(140);
    _delay_ms(1000);
    servo_3_free();

    servo_1(0);
    _delay_ms(1000);
    servo_1_free();

	velocity(base,base);        //Restoring Velocity before movement
	left();
	_delay_ms(47);
	stop();
	_delay_ms(750);
    PORTH |= (1 << 3); //Slave Servo motor demux pin set to 1
    _delay_ms(100);
    servo_4(50);
    _delay_ms(1000);
    servo_3_free();

    servo_2(12);
    _delay_ms(1000);
    servo_1(95);
    _delay_ms(1000);
    servo_1_free();
	velocity(base,base);       //Restoring Velocity before movement
    left();
    _delay_ms(60);
    stop();
}

/*
*
* Function Name: inv_place_2
* Input: void
* Output: void
* Logic: uses Servo 3 (Master) and servo 4(slave) to place the CM to special low-rise house in inversion path
*
*/
void inv_place_2()
{
    velocity(base,base);            //Restoring Velocity before movement
    left();
    _delay_ms(50);
    stop();
	_delay_ms(150);
    servo_1(180);
    _delay_ms(1000);
    servo_1_free();
    servo_2(85);
    _delay_ms(1000);

	PORTH &= 0x00;
    PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
    _delay_ms(100);

    servo_3(140);
    _delay_ms(1000);
    servo_3_free();
    servo_1(175);
    _delay_ms(1000);
    servo_1_free();
	_delay_ms(750);
    PORTH |= (1 << 3);//Slave Servo motor demux pin set to 1
    _delay_ms(100);
    servo_4(50);
    _delay_ms(1000);
    servo_3_free();
	servo_2(12);
    _delay_ms(1000);
    servo_1(95);
    _delay_ms(1000);
    servo_1_free();
	velocity(base,base);   //Restoring Velocity before movement
    right();
    _delay_ms(70);
    stop();
}

/*
*
* Function Name: place_lr
* Input: void
* Output: void
* Logic: uses Servo 3 (Master) and servo 4(slave) to place the CM to low-rise house
*
*/
void place_lr(void)
{
    PORTH |= (1 << 3);//Slave Servo motor demux pin set to 1
    _delay_ms(100);
    servo_1(65);
    _delay_ms(750);
	servo_2(100);
	_delay_ms(750);
    servo_3(75);
    _delay_ms(1000);

	servo_1(90);
	_delay_ms(1000);
	servo_1_free();
	PORTH &= 0x00;
	PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
	_delay_ms(100);
	servo_3(140);
	_delay_ms(750);

    servo_2(12);
    _delay_ms(1000);
    servo_1(95);
	_delay_ms(750);
	servo_1_free();
	velocity(base,base);     //Restoring Velocity before movement
}

/*
*
* Function Name: place_hr
* Input: void
* Output: void
* Logic: uses Servo 3 (Master) and servo 4(slave) to place the CM to high-rise house
*
*/
void place_hr(void)
{
    PORTH |= (1 << 3);//Slave Servo motor demux pin set to 1
    _delay_ms(100);
    servo_1(65);
    _delay_ms(750);
	servo_2(70);
	_delay_ms(1000);
    servo_3(110);
    _delay_ms(1000);
	servo_1(90);
	_delay_ms(1000);
	servo_1_free();
	PORTH &= 0x00;
	PORTH |= (1 << 0);  //Master Servo motor demux pin set to 0
	_delay_ms(100);
	servo_3(120);
	_delay_ms(750);
    servo_2(12);
    _delay_ms(1000);
    servo_1(95);
    _delay_ms(750);
    servo_1_free();
	velocity(base,base);  //Restoring Velocity before movement
}

/* --------------------------------------------------------------NAVIGATION SECTION---------------------------------------------------------------------------*/

/*
*
* Function Name: forward_walls
* Input: void
* Output: void
* Logic: used to navigate through the walls using sharp sensor, is called automatically by line follower " forward_wls(int a)"
* Example: forward_wls(1,1)
*
*/
void forward_walls()
{
    LCD_Function(1);                  // Function to print "Wall-Follower"
    forward();                        // Forcing Bot to Enter the region between walls
    velocity(base, base);
    _delay_ms(500);
    int correction = 0;                      //error counter
    while (1)
    {
        forward();
        wall = ADC_Conversion(11);
        rs = ADC_Conversion(3);
        ms = ADC_Conversion(2);

        if (((rs > 80  ) || ( ms > 80) )&&( wall < 100)) // Condition to break te loop and call line follower
        {
            stop();
			_delay_ms(150);
            break;
        }
        if ((wall >= 120) && (wall <= 135)) // condition for the bot to move straight
        {
            ++correction ;
            if (correction>3)            // Measure of bot to see if its moving in a straight line and to boost its speed if the condition is met
            {
                OCR5AL = base;
                OCR5BL = base;
                _delay_ms(3);
            }
            else
            {
                OCR5AL = 220;
                OCR5BL = 220;
                _delay_ms(5);
            }


        }
        else if (wall > 135)  // Condition to move away from wall
        {
            correction = 0;
            right();
            OCR5AL = 130;
            OCR5BL = 138;
            _delay_ms(2);
        }
        else if (wall < 120 && wall > 50) // Condition to move towards wall
        {
            correction = 0;
            left();
            OCR5AL = 110;
            OCR5BL = 118;
            _delay_ms(1);
        }
    }
    forward_wls(0, 1);       // Calling Line Follower
}

/*
*
* Function Name: forward_wls
* Input: integer a, integer node ( fixed to 1)
* Output: void
* Logic: used to navigate through the black line using line sensor, entered input determines if the function is to serve as node to node navigation and black line to wall follower
* Example: forward_wls(0,1) ----------> invokes bllack line follower for one node  
*/
void forward_wls(int a, int node)
{
    LCD_Function(0);    // Function to print string "Line-Follower" on LCD
    unsigned int error = 0;   // variable to measure deflection of robot from the path
    while (1)
    {
        ls = ADC_Conversion(1);
        ms = ADC_Conversion(2);
        rs = ADC_Conversion(3);
        wall =ADC_Conversion(11);

        forward();
        OCR5AL = base;
        OCR5BL = base;


        if ((a == 2) && (ls + ms + rs > 310)) // Certain nodes on the edge of the arena allow only two sensor to stand on them and hence have a different threshold than standard nodes
        {
            stop();
            _delay_ms(150);
            velocity(base,base);
            forward();
            _delay_ms(280);
            stop();
            break;
        }

        else if ( a == 3) // Condition to call Inversion Navigation, "forward_inv()"
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

            if ((ms < 100) && (wall >= 85))
            {
                stop();
                break;
            }
        }
        else if ((a == 0) && (ls + ms + rs > 400)) // Standard nodes threshold
        {
            stop();
            _delay_ms(50);
            velocity(base,base);
            forward();
            _delay_ms(250);
            stop();
            break;
        }
        if ((ls < 60 && ms >= 125 && rs < 60)) // Motor speed is changed directly to adjust the robot rather than calling left or right to increase smoothness in motion
        {
            // Velocity function was not called and values were configured directly as function calling was increasing bot response time in while(1) loop

            OCR5AL = base;
            OCR5BL = base;
            _delay_ms(1);
        }
        else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls > rs)
        {
            OCR5AL = soft;
            OCR5BL = base;
            _delay_ms(2);

        }
        else if ((ls + ms + rs) < 200 && (ms > 50 && ms < 100) && ls < rs)
        {
            OCR5AL = base;
            OCR5BL = soft;
            _delay_ms(2);
        }
        else if ((ls < 40 && ms < 60 && rs > 140))  // soft movement functions called to correct major deflection that only velocity change could not
        {
            soft_right();
            OCR5AL = 120;
            OCR5BL = 120;
            _delay_ms(3);

        }
        else if (rs < 40 && ms < 60 && ls > 140)
        {
            soft_left();
            OCR5AL = 120;
            OCR5BL = 120;
            _delay_ms(3);

        }
        if (ms < 60 && (ls + rs < 40)&& (a ==  0))
        {
            ++error;
            if ((error>=250)) // When error reaches significant magnitude "static_reorientation" is called to correct the robot's path
            {
                PORTA = 0x00;
                stop();
                _delay_ms(100);
                velocity(150,150);
                back();
                _delay_ms(150);
                static_reorientation();
                error = 0;
            }


        }

    }
    if (a == 1)
    {
        forward_walls();
    }
    else if (a == 3)
    {
        forward_inv();
    }
velocity(base,base); // Restoring velocity values
}

/*
*
* Function Name: forward_inv
* Input: void
* Output: void
* Logic: used to navigate through the white line using line sensor
* Example: forward_wls(3,1)
*
*/
void forward_inv()
{
    unsigned char w = 0;  // White node counter

    LCD_Function(3);  // Function to print string "Inversion " on LCD



    while (1)
    {
        forward();
        velocity(base, base);

        ls = ADC_Conversion(1);
        ms = ADC_Conversion(2);
        rs = ADC_Conversion(3);

        if ( ls+ms+rs < 290 && ms <70 && w==0) // Condition for white node
        {

            _delay_ms(100);
            ++w;
            back();             // Configuration to place CM in house H5 based on robot's direction
            _delay_ms(450);
            stop();
            if (inv == 1)
                inv_place_1();
            else if (inv == 0)
                inv_place_2();
            forward();
            _delay_ms(250);
            continue;


        }
        if( rs< 100 && ls < 100)  // Condition to break loop and call wall follower
        {
            PORTA =0x00; // partial condition to invoke "stop()"
            break;
        }

        else if ((ls > 180 && ms < 130 && rs > 180)) // Motor speed is changed directly to adjust the robot rather than calling left or right to increase smoothness in motion
        {
            // Velocity function was not called and values were configured directly as function calling was increasing bot response time in while(1) loop

            OCR5AL = base;
            OCR5BL = base;
        }
        else if ((ls < 80 && ms > 130 && rs > 150)) // soft movement functions called to correct major deflection that only velocity change could not
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
    }
    stop();      // configuration to orient bot when it exits inversion
    _delay_ms(100);
    static_reorientation();
    forward_wls(2,1);
}


/*
*
* Function Name: static_reorientation
* Input: void
* Output: void
* Logic: used to align the robot to the black line
*
*/
void static_reorientation()
{
    stop();
    ms = ADC_Conversion(2);
    if (ms < 110)
    {
        OCR5AL = 150;
        OCR5BL = 150;
        right();
        _delay_ms(300);
        stop();
        OCR5AL = 125;
        OCR5BL = 125;
        left();
        while (1)
        {

            ms = ADC_Conversion(2);
            if (ms >= 80)
            {
                PORTA = 0x00;
                break;
            }
        }
        stop();
        OCR5AL = base;
        OCR5BL = base;
    }

}

/*
*
* Function Name: static_reorientation_inv
* Input: void
* Output: void
* Logic: used to align the robot to the white line
*
*/
void static_reorientation_inv()
{
    stop();
	_delay_ms(150);
    ls = ADC_Conversion(1);
    ms = ADC_Conversion(2);
    rs = ADC_Conversion(3);

    if (ms > 110)
    {
        OCR5AL = 150;
        OCR5BL = 150;
        right();
        _delay_ms(300);
        stop();
        OCR5AL = 125;
        OCR5BL = 125;
        left();
        _delay_ms(50);
        while (1)
        {
            ls = ADC_Conversion(1);
            ms = ADC_Conversion(2);
            rs = ADC_Conversion(3);
            if ((ls + ms <= 250)&&(rs>= 130))
            {
                PORTA = 0x00;
                break;
            }
        }
        stop();
        OCR5AL = base;
        OCR5BL = base;
    }
}

/*
*
* Function Name: forward_zigzag
* Input: void
* Output: void
* Logic: used to navigate through the zig-zag region using line sensor
* Example: forward_zigzag()
*
*/
void forward_zigzag()
{
    LCD_Function(5); // Function to print string "Zig-Zag" on LCD
    int error = 0; // variable to measure white node or deflection


    while (1)
    {
        ls = ADC_Conversion(1);
        ms = ADC_Conversion(2);
        rs = ADC_Conversion(3);
        forward();
        velocity(250, 250);


        if ((ls + ms + rs) > 400)  // Condition to detect node at the end of path
        {
              stop();
              _delay_ms(50);
              velocity(base,base);
              forward();
              _delay_ms(250);
              stop();
              break;
        }
        if ( ms > 100)   // Condition to move forward
        {
            forward();
            OCR5AL = 250;
            OCR5BL = 250;
            _delay_ms(5);
        }

        if (ls < 20 && ms < 20 && rs < 20) // White node or error detection
        {


            ++error;
            if (error>=130)
            {
                static_reorientation(); // Orienting bot with black line
                error = 0;
            }
        }


        if (ls > ms) // Quick adjust left
        {
            left();
            OCR5AL = 180;
            OCR5BL = 160;
            _delay_ms(1);

        }
        else if (ms < rs) // Quick adjust right
        {
            right();
            OCR5AL = 160;
            OCR5BL = 180;
            _delay_ms(1);
        }
    }
    stop();
    _delay_ms(100);
    velocity(base,base);
    forward();
    _delay_ms(80);
    stop();
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
void right_turn_wls(void)
{
    OCR5AL = 150;
    OCR5BL = 150;
    right(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(200);
    OCR5AL = 130;
    OCR5BL = 130;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {
        rs = ADC_Conversion(3);

        if (rs>80)
        {
            PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
            break;
        }
    }
    stop();
    OCR5AL = base;
    OCR5BL = base;

}

/*
*
* Function Name: right_turn_inv
* Input: void
* Output: void
* Logic: used to turn the robot to the right using the line sensor in inverse region
* Example : right_turn_inv()
*/
void right_turn_inv(void)
{
    OCR5AL = 130;
    OCR5BL = 130;
    right(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(200);
    OCR5AL = 115;
    OCR5BL = 115;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {

        ms = ADC_Conversion(2);
        ls = ADC_Conversion(1);
        rs = ADC_Conversion(3);
        if ((rs + ms <= 250)&&(ls>= 130))
        {
            PORTA = 0x00;//Stops the robot mediately, partial condition to invoke "stop()" function
            stop();
            break;
        }
    }
	velocity(base,base);

    static_reorientation_inv(); // Make sure robot is aligned with white line


}

/*
*
* Function Name: right_turn_bwall
* Input: void
* Output: void
* Logic: used to turn the robot to the right using the line sensor
*
*/
void right_turn_wls_bwall(void)
{

    OCR5AL = 150;
    OCR5BL = 150;
    right(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(200);
    OCR5AL = 130;
    OCR5BL = 130;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {
        rs = ADC_Conversion(3);

        if (rs>80)
        {
            PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
            break;
        }
    }
    stop();
    OCR5AL = base;
    OCR5BL = base;

}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: used to turn the robot to the left using the line sensor
* Example : left_turn_wls()
*/
void left_turn_wls(void)
{
    OCR5AL = 150;
    OCR5BL = 150;
    left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(400);
    OCR5AL = 130;
    OCR5BL = 130;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {

        ls = ADC_Conversion(1);
        if (ls >= 80)
        {

            PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
            break;
        }
    }
    stop();
    OCR5AL = base;
    OCR5BL = base;
}

/*
*
* Function Name: left_turn_inv
* Input: void
* Output: void
* Logic: used to turn the robot to the left using the line sensor in inverse region
* Example : left_tun_inv()
*/
void left_turn_inv(void)
{
    OCR5AL = 130;
    OCR5BL = 130;
    left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(200);
    OCR5AL = 115;
    OCR5BL = 115;
    while (1) //while loop which detects white line using middle sensor so that the robot stops turning
    {

        ms = ADC_Conversion(2);
        ls = ADC_Conversion(1);
        rs = ADC_Conversion(3);
        if ((ls + ms <= 250)&&(rs>= 130))
        {
            PORTA = 0x00;//Stops the robot mediately, partial condition to invoke "stop()" function
            stop();
            break;
        }
    }
	velocity(base,base);

    static_reorientation_inv(); // Make sure robot is aligned with white line

}

/*
*
* Function Name: left_turn_bwall
* Input: void
* Output: void
* Logic: used to turn the robot to the left using the line sensor
*
*/
void left_turn_wls_bwall(void)
{
    OCR5AL = 150;
    OCR5BL = 150;
    left(); //code which help the robot to ignore the black line which is going straight so that it can focus on line which is going to the right
    _delay_ms(400);
    OCR5AL = 125;
    OCR5BL = 125;
    while (1) //while loop which detects black line using middle sensor so that the robot stops turning
    {

        ls = ADC_Conversion(2);
        if (ls >= 80)
        {
            PORTA = 0x00; //Stops the robot mediately, partial condition to invoke "stop()" function
            break;
        }
    }
    stop();
    OCR5AL = base;
    OCR5BL = base;
}


/* -------------------------------------------------------------INITIALIZATION--------------------------------------------------*/

//Function to Initialize ADC
void adc_init()
{
    ADCSRA = 0x00;
    ADCSRB = 0x00; //MUX5 = 0
    ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
    ACSR = 0x80;
    ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void init_devices(void)
{
    cli(); //Clears the global interrupts
    port_init();
    adc_init();
    timer5_init();
    timer1_init();
    sei(); //Enables the global interrupts
    lcd_set_4bit(); //These functions need not to be inside interrupt blocked code
    lcd_init();
}


/*--------------------------------------------------------------MAIN-----------------------------------------------------------*/
int main()
{
    init_devices();             //init devices
    int len = 0;
    int *path = (int*)malloc(80);             //main path array
    u = 0;                       //current node
    face = 's';                       //current direction
    forward_wls(0,1);
    block = which_material[8];          //getting block for house 5
    if (block==0)               // if block is not to be picked
    {
        block = which_material[9];     //getting another block for house 5

        bnode = which_node(block);
        dijkstra(G,n,u);                //calculating distance of every other node from u
        dist_comp(bnode,bnode,path,&len);          //getting path for reaching the block
        traverse(path,u,&len);              //calling navigation functions according to the path

        free(path);                     // path variable freed from memory
        len = 0;
        block_choose2(block,block_placed); // positioning the robot to pick the block


        s_pick();                       //calling s_pick
        forward();
        _delay_ms(150);
        stop();

        for(int p = 0; p<=12; p++)
        {
            if (path[p] == 1)
                lcd_print(1,1,p,2);
        }
        u=bnode;            //updating current node

        dijkstra(G,n,u);  // calculating distance to every other node

        dist_comp(h51,h52,path,&len); //comparing house 5 node 7 or 8 from u
        traverse(path,u,&len); //reaching house 5

        u = path[len-1];   //updating current node
        free(path);
        len = 0;
    }
    else            //if block is to be picked
    {                   //reaching for the block and picking it
        bnode = which_node(block);
        dijkstra(G,n,u);
        dist_comp(bnode,bnode,path,&len);
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        m_pick();
        forward();
        _delay_ms(150);
        stop();

        u=bnode;        //further functions work as explained in above lines

        block = which_material[9];
        bnode = which_node(block);


        dijkstra(G,n,u);

        dist_comp(bnode,bnode,path,&len);
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        s_pick();
        forward();
        _delay_ms(150);
        stop();

        for(int p = 0; p<=12; p++)
        {
            if (path[p] == 1)
                lcd_print(1,1,p,2);
        }
        u=bnode;

        dijkstra(G,n,u);

        dist_comp(h51,h52,path,&len);
        traverse(path,u,&len);

        u = path[len-1];
        free(path);
        len = 0;
    }
    if (u==7)               //calling forward inverse
    {
        inv = 0;
        right_turn_inv();
        forward_wls(3,1);
        face = 'e';
        u = 8;                  //finding new block to pick
        is_block1 = check_block(11);
        is_block2 = check_block(12);
        is_block3 = check_block(7);
        is_block4 = check_block(8);
        if (block_placed[10] == 0 && is_block1==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block2==1)
            block = 12;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 8;
    }
    else if (u==8)              //calling forward inverse
    {
        inv = 1;
        left_turn_inv();
        forward_wls(3,1);
        face = 'w';
        u = 7;                      //finding new block to pick
        is_block1 = check_block(9);
        is_block2 = check_block(10);
        is_block3 = check_block(5);
        is_block4 = check_block(6);
        if (block_placed[8] == 0 && is_block1==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block2==1)
            block = 10;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 5;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 6;
    }

    bnode = which_node(block);
    dijkstra(G,n,u);
    dist_comp(bnode,bnode,path,&len);
    traverse(path,u,&len);
    free(path);
    len = 0;
    block_choose2(block,block_placed);

    u = bnode;
    for (int i = 0; i < 10; i++)        //finding house according to the picked block
    {
        if (which_material[i] == block)
        {
            if (i==0)
            {
                house_no = 1;
                break;
            }
            else if (i==1)
            {
                house_no = 1;
                break;
            }
            else if (i==2)
            {
                house_no = 2;
                break;
            }
            else if (i==3)
            {
                house_no = 2;
                break;
            }
            else if (i==4)
            {
                house_no = 3;
                break;
            }
            else if (i==5)
            {
                house_no = 3;
                break;
            }
            else if (i==6)
            {
                house_no = 4;
                break;
            }
            else if (i==7)
            {
                house_no = 4;
                break;
            }
        }
        else
        {
            house_no = -1;
        }
    }
    if (house_no != -1)        //if house is found
    {
        m_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    if (block == which_material[2*house_no - 1])        //finding another block to be placed in the house
    {
        block = which_material[2*house_no - 2];
    }
    else
    {
        block = which_material[2*house_no - 1];
    }
    if (block != 0)
    {
        dijkstra(G,n,u);

        bnode = which_node(block);
        dist_comp(bnode,bnode,path,&len);

        bnode = path[len-1];
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        s_pick();
        forward();
        _delay_ms(150);
        stop();
    }

    dijkstra(G,n,u);

    int hnode = 0;
    hnode = which_house(house_no);
    dist_comp(hnode,hnode,path,&len);
    traverse(path,u,&len);

    free(path);
    len = 0;
    if (house_no == 1)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)        //checking if house is high-rise or low-rise
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }                                       // Further functions work similar to the explained above
        is_block1 = check_block(1);             //comments are not written in further steps as they are self-explanatory
        is_block2 = check_block(2);
        is_block3 = check_block(5);
        is_block4 = check_block(6);
        is_block5 = check_block(9);
        is_block6 = check_block(10);
        if (block_placed[0] == 0 && is_block1==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block2==1)
            block = 2;
        else if (block_placed[4] == 0 && is_block3==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block4==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block5==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block6==1)
            block = 10;
    }
    else if (house_no == 2)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(3);
        is_block2 = check_block(4);
        is_block3 = check_block(7);
        is_block4 = check_block(8);
        is_block5 = check_block(11);
        is_block6 = check_block(12);
        if (block_placed[2] == 0 && is_block1==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block2==1)
            block = 4;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block5==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block6==1)
            block = 12;
    }
    else if (house_no == 3)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(5);
        is_block2 = check_block(6);
        is_block3 = check_block(9);
        is_block4 = check_block(10);
        is_block5 = check_block(1);
        is_block6 = check_block(2);
        if (block_placed[4] == 0 && is_block1==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block2==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block3==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block4==1)
            block = 10;
        else if (block_placed[0] == 0 && is_block5==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block6==1)
            block = 2;
    }
    else if (house_no == 4)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(7);
        is_block2 = check_block(8);
        is_block3 = check_block(11);
        is_block4 = check_block(12);
        is_block5 = check_block(3);
        is_block6 = check_block(4);
        if (block_placed[6] == 0 && is_block1==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block2==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block3==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block4==1)
            block = 12;
        else if (block_placed[2] == 0 && is_block5==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block6==1)
            block = 4;
    }

    bnode = which_node(block);
    dijkstra(G,n,u);



    dist_comp(bnode,bnode,path,&len);
    traverse(path,u,&len);

    free(path);
    len = 0;
    block_choose2(block,block_placed);


    u = bnode;
    for (int i = 0; i < 10; i++)
    {
        if (which_material[i] == block)
        {
            if (i==0)
            {
                house_no = 1;
                break;
            }
            else if (i==1)
            {
                house_no = 1;
                break;
            }
            else if (i==2)
            {
                house_no = 2;
                break;
            }
            else if (i==3)
            {
                house_no = 2;
                break;
            }
            else if (i==4)
            {
                house_no = 3;
                break;
            }
            else if (i==5)
            {
                house_no = 3;
                break;
            }
            else if (i==6)
            {
                house_no = 4;
                break;
            }
            else if (i==7)
            {
                house_no = 4;
                break;
            }
        }
        else
        {
            house_no = -1;
        }
    }
    if (house_no != -1)
    {
        m_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    if (block == which_material[2*house_no - 1])
    {
        block = which_material[2*house_no - 2];
    }
    else
    {
        block = which_material[2*house_no - 1];
    }
    if (block != 0)
    {
        dijkstra(G,n,u);

        bnode = which_node(block);
        dist_comp(bnode,bnode,path,&len);
        bnode = path[len-1];
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        s_pick();
        forward();
        _delay_ms(150);
        stop();
    }

    dijkstra(G,n,u);

    hnode = 0;
    hnode = which_house(house_no);
    dist_comp(hnode,hnode,path,&len);

    traverse(path,u,&len);

    free(path);
    len = 0;
    if (house_no == 1)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(1);
        is_block2 = check_block(2);
        is_block3 = check_block(5);
        is_block4 = check_block(6);
        is_block5 = check_block(9);
        is_block6 = check_block(10);
        if (block_placed[0] == 0 && is_block1==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block2==1)
            block = 2;
        else if (block_placed[4] == 0 && is_block3==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block4==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block5==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block6==1)
            block = 10;
    }
    else if (house_no == 2)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(3);
        is_block2 = check_block(4);
        is_block3 = check_block(7);
        is_block4 = check_block(8);
        is_block5 = check_block(11);
        is_block6 = check_block(12);
        if (block_placed[2] == 0 && is_block1==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block2==1)
            block = 4;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block5==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block6==1)
            block = 12;
    }
    else if (house_no == 3)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(5);
        is_block2 = check_block(6);
        is_block3 = check_block(9);
        is_block4 = check_block(10);
        is_block5 = check_block(1);
        is_block6 = check_block(2);
        if (block_placed[4] == 0 && is_block1==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block2==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block3==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block4==1)
            block = 10;
        else if (block_placed[0] == 0 && is_block5==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block6==1)
            block = 2;
    }
    else if (house_no == 4)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(7);
        is_block2 = check_block(8);
        is_block3 = check_block(11);
        is_block4 = check_block(12);
        is_block5 = check_block(3);
        is_block6 = check_block(4);
        if (block_placed[6] == 0 && is_block1==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block2==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block3==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block4==1)
            block = 12;
        else if (block_placed[2] == 0 && is_block5==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block6==1)
            block = 4;
    }
    free(path);
    len = 0;
    bnode = which_node(block);
    dijkstra(G,n,u);

    dist_comp(bnode,bnode,path,&len);
    traverse(path,u,&len);

    free(path);
    len = 0;
    block_choose2(block,block_placed);

    u = bnode;
    for (int i = 0; i < 10; i++)
    {
        if (which_material[i] == block)
        {
            if (i==0)
            {
                house_no = 1;
                break;
            }
            else if (i==1)
            {
                house_no = 1;
                break;
            }
            else if (i==2)
            {
                house_no = 2;
                break;
            }
            else if (i==3)
            {
                house_no = 2;
                break;
            }
            else if (i==4)
            {
                house_no = 3;
                break;
            }
            else if (i==5)
            {
                house_no = 3;
                break;
            }
            else if (i==6)
            {
                house_no = 4;
                break;
            }
            else if (i==7)
            {
                house_no = 4;
                break;
            }
        }
        else
        {
            house_no = -1;
        }
    }
    if (house_no != -1)
    {
        m_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    if (block == which_material[2*house_no - 1])
    {
        block = which_material[2*house_no - 2];
    }
    else
    {
        block = which_material[2*house_no - 1];
    }
    if (block != 0)
    {
        dijkstra(G,n,u);

        bnode = which_node(block);
        dist_comp(bnode,bnode,path,&len);



        for(int p = 0; p<len; p++)
        {
            //printf("%d\n",path[p]);
            //lcd_print(1,2*p,path[p],2);
        }
        bnode = path[len-1];
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        s_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    dijkstra(G,n,u);

    hnode = 0;
    hnode = which_house(house_no);
    dist_comp(hnode,hnode,path,&len);

    for(int p = 0; p<len; p++)
    {
        //printf("%d\n",path[p]);
        //lcd_print(1,2*p,path[p],2);
    }
    traverse(path,u,&len);

    free(path);
    len = 0;
    if (house_no == 1)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(1);
        is_block2 = check_block(2);
        is_block3 = check_block(5);
        is_block4 = check_block(6);
        is_block5 = check_block(9);
        is_block6 = check_block(10);
        if (block_placed[0] == 0 && is_block1==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block2==1)
            block = 2;
        else if (block_placed[4] == 0 && is_block3==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block4==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block5==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block6==1)
            block = 10;
    }
    else if (house_no == 2)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(3);
        is_block2 = check_block(4);
        is_block3 = check_block(7);
        is_block4 = check_block(8);
        is_block5 = check_block(11);
        is_block6 = check_block(12);
        if (block_placed[2] == 0 && is_block1==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block2==1)
            block = 4;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block5==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block6==1)
            block = 12;
    }
    else if (house_no == 3)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(5);
        is_block2 = check_block(6);
        is_block3 = check_block(9);
        is_block4 = check_block(10);
        is_block5 = check_block(1);
        is_block6 = check_block(2);
        if (block_placed[4] == 0 && is_block1==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block2==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block3==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block4==1)
            block = 10;
        else if (block_placed[0] == 0 && is_block5==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block6==1)
            block = 2;
    }
    else if (house_no == 4)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(7);
        is_block2 = check_block(8);
        is_block3 = check_block(11);
        is_block4 = check_block(12);
        is_block5 = check_block(3);
        is_block6 = check_block(4);
        if (block_placed[6] == 0 && is_block1==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block2==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block3==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block4==1)
            block = 12;
        else if (block_placed[2] == 0 && is_block5==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block6==1)
            block = 4;
    }

    free(path);
    len = 0;
    bnode = which_node(block);
    dijkstra(G,n,u);



    dist_comp(bnode,bnode,path,&len);
    traverse(path,u,&len);

    free(path);
    len = 0;
    block_choose2(block,block_placed);

    u = bnode;
    for (int i = 0; i < 10; i++)
    {
        if (which_material[i] == block)
        {
            if (i==0)
            {
                house_no = 1;
                break;
            }
            else if (i==1)
            {
                house_no = 1;
                break;
            }
            else if (i==2)
            {
                house_no = 2;
                break;
            }
            else if (i==3)
            {
                house_no = 2;
                break;
            }
            else if (i==4)
            {
                house_no = 3;
                break;
            }
            else if (i==5)
            {
                house_no = 3;
                break;
            }
            else if (i==6)
            {
                house_no = 4;
                break;
            }
            else if (i==7)
            {
                house_no = 4;
                break;
            }
        }
        else
        {
            house_no = -1;
        }
    }
    if (house_no != -1)
    {
        m_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    if (block == which_material[2*house_no - 1])
    {
        block = which_material[2*house_no - 2];
    }
    else
    {
        block = which_material[2*house_no - 1];
    }
    if (block != 0)
    {
        dijkstra(G,n,u);

        bnode = which_node(block);
        dist_comp(bnode,bnode,path,&len);

        bnode = path[len-1];
        traverse(path,u,&len);

        free(path);
        len = 0;
        block_choose2(block,block_placed);

        s_pick();
        forward();
        _delay_ms(150);
        stop();
    }
    dijkstra(G,n,u);

    hnode = 0;
    hnode = which_house(house_no);
    dist_comp(hnode,hnode,path,&len);

    traverse(path,u,&len);

    free(path);
    len = 0;
    if (house_no == 1)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(1);
        is_block2 = check_block(2);
        is_block3 = check_block(5);
        is_block4 = check_block(6);
        is_block5 = check_block(9);
        is_block6 = check_block(10);
        if (block_placed[0] == 0 && is_block1==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block2==1)
            block = 2;
        else if (block_placed[4] == 0 && is_block3==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block4==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block5==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block6==1)
            block = 10;
    }
    else if (house_no == 2)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(3);
        is_block2 = check_block(4);
        is_block3 = check_block(7);
        is_block4 = check_block(8);
        is_block5 = check_block(11);
        is_block6 = check_block(12);
        if (block_placed[2] == 0 && is_block1==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block2==1)
            block = 4;
        else if (block_placed[6] == 0 && is_block3==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block4==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block5==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block6==1)
            block = 12;
    }
    else if (house_no == 3)
    {
        fdir = 'w';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(5);
        is_block2 = check_block(6);
        is_block3 = check_block(9);
        is_block4 = check_block(10);
        is_block5 = check_block(1);
        is_block6 = check_block(2);
        if (block_placed[4] == 0 && is_block1==1)
            block = 5;
        else if (block_placed[5] == 0 && is_block2==1)
            block = 6;
        else if (block_placed[8] == 0 && is_block3==1)
            block = 9;
        else if (block_placed[9] == 0 && is_block4==1)
            block = 10;
        else if (block_placed[0] == 0 && is_block5==1)
            block = 1;
        else if (block_placed[1] == 0 && is_block6==1)
            block = 2;
    }
    else if (house_no == 4)
    {
        fdir = 'e';
        block_traverse();
        back();
        _delay_ms(100);
        stop();
        if (house_config[house_no-1]==0)
        {
            back();
            _delay_ms(300);
            stop();
            place_lr();
            forward();
            _delay_ms(150);
            stop();
        }
        else
        {
            back();
            _delay_ms(300);
            stop();
            place_hr();
            forward();
            _delay_ms(150);
            stop();
        }
        is_block1 = check_block(7);
        is_block2 = check_block(8);
        is_block3 = check_block(11);
        is_block4 = check_block(12);
        is_block5 = check_block(3);
        is_block6 = check_block(4);
        if (block_placed[6] == 0 && is_block1==1)
            block = 7;
        else if (block_placed[7] == 0 && is_block2==1)
            block = 8;
        else if (block_placed[10] == 0 && is_block3==1)
            block = 11;
        else if (block_placed[11] == 0 && is_block4==1)
            block = 12;
        else if (block_placed[2] == 0 && is_block5==1)
            block = 3;
        else if (block_placed[3] == 0 && is_block6==1)
            block = 4;
    }
    _delay_ms(3500);
    buzzer_on();
}



