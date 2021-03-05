//header files
//#include <stdio.h>
#define F_CPU 14745600
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include <math.h> //included to support power function
#include "lcd.h"
//LEN(x)
#define LEN(x) (sizeof(x) / sizeof((x)[0]))

//global variables
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char floor_array[5],house_total_requirement[5],which_material[9];
unsigned char cost[17][17];
int phi=270; //initial robot direction
unsigned whiteline_left_sensor=0,whiteline_middle_sensor=0,whiteline_right_sensor=0,left_prox_sensor=0,right_prox_sensor=0;
unsigned whiteline_threshold_left = 70 , whiteline_threshold_middle = 40 , whiteline_threshold_right = 70;
unsigned wall_threshold=155;
unsigned proxy_left,proxy_right;

//ATmega2560
//ADC pin configuration
void adc_pin_config (void)
{
DDRF = 0x00; //set PORTF direction as input
PORTF = 0x00; //set PORTF pins floating
DDRK = 0x00; //set PORTK direction as input
PORTK = 0x00; //set PORTK pins floating
}

//Function to Initialize ADC
void adc_init()
{
ADCSRA = 0x00;
ADCSRB = 0x00; //MUX5 = 0
ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
ACSR = 0x80;
ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
unsigned char a;
if(Ch>7)
{
ADCSRB = 0x08;
}
Ch = Ch & 0x07;
ADMUX= 0x20| Ch;
ADCSRA = ADCSRA | 0x40; //Set start conversion bit
while((ADCSRA&0x10)==0); //Wait for ADC conversion to complete
a=ADCH;
ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
ADCSRB = 0x00;
return a;
}



// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location.
void print_sensor(char row, char coloumn,unsigned char channel)   //lcd_print(char row, char coloumn, unsigned int value, int digits)
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
distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
distanceInt = (int)distance;
if(distanceInt>800)
{
distanceInt=800;
}
return distanceInt;
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
DDRA = DDRA | 0x0F;
PORTA = PORTA & 0xF0;
DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void buzzer_pin_config()
{
DDRJ=DDRJ|0x01;
PORTJ=PORTJ | 0x01;
}
//Function to initialize ports
void port_init()
{  
lcd_port_config();  //Configure LCD
adc_pin_config(); //Configure ADC
    servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
motion_pin_config(); //robot motion pins config
left_encoder_pin_config(); //left encoder pin config
right_encoder_pin_config(); //right encoder pin config
buzzer_pin_config();
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
cli(); //Clears the global interrupt
EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
cli(); //Clears the global interrupt
EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
ShaftCountLeft++;  //increment left shaft position count
}


//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void) //servo
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
 ICR1H  = 0x03;
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
  For Overriding normal port functionality to OCRnA outputs.
 {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()  //PWM
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
TCCR5A = 0xA9; /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
   For Overriding normal port functionality to OCRnA outputs.
   {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

TCCR5B = 0x0B; //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
OCR5AL = (unsigned char)left_motor;
OCR5BL = (unsigned char)right_motor;
}
//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
unsigned char PortARestore = 0;

Direction &= 0x0F; // removing upper nibbel for the protection
PortARestore = PORTA; // reading the PORTA original status
PortARestore &= 0xF0; // making lower direction nibbel to 0
PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
PORTA = PortARestore; // executing the command
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
float PositionPanServo = 0;
PositionPanServo = ((float)degrees / 1.86) + 35.0;
OCR1AH = 0x00;
OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
float PositionTiltServo = 0;
PositionTiltServo = ((float)degrees / 1.86) + 35.0;
OCR1BH = 0x00;
OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
float PositionServo = 0;
PositionServo = ((float)degrees / 1.86) + 35.0;
OCR1CH = 0x00;
OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
OCR1AH = 0x03;
OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
OCR1BH = 0x03;
OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
OCR1CH = 0x03;
OCR1CL = 0xFF; //Servo 3 off
}


void forward (void) //both wheels forward
{
motion_set(0x06);
}

void back (void) //both wheels backward
{
motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
motion_set(0x08);
}

void stop (void)
{
motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
float ReqdShaftCount = 0;
unsigned long int ReqdShaftCountInt = 0;

ReqdShaftCount = (float) Degrees*3.9031; // division by resolution to get shaft count
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

ReqdShaftCount = DistanceInMM * 2.15; // division by resolution to get shaft count
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

void left_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
left(); //Turn left
angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
right(); //Turn right
angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_left(); //Turn soft left
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_right();  //Turn soft right
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_left_2(); //Turn reverse soft left
Degrees=Degrees*2;
angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
// 176 pulses for 360 degrees rotation 2.045 degrees per count
soft_right_2();  //Turn reverse soft right
Degrees=Degrees*2;
angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
cli(); //Clears the global interrupt
port_init();  //Initializes all the ports
adc_init();   //ADC initialization
timer1_init(); //servo
timer5_init(); //pwm_wheels
left_position_encoder_interrupt_init();
right_position_encoder_interrupt_init();
sei();   // Enables the global interrupt
}

void sensor_update(void)
{  
whiteline_left_sensor = ADC_Conversion(1);
whiteline_middle_sensor = ADC_Conversion(2);
whiteline_right_sensor = ADC_Conversion(3);
left_prox_sensor = ADC_Conversion(10);
right_prox_sensor = ADC_Conversion(9);

}
void print_all_sensors()
{
print_sensor(2,2,3); //Prints value of White Line Sensor1
print_sensor(2,6,2); //Prints Value of White Line Sensor2
print_sensor(2,10,1); //Prints Value of White Line Sensor3
print_sensor(1,13,9);                           // right prox
print_sensor(1,8,10);                           //left prox

}
void buzzer()
{  
PORTJ=PORTJ & 0x00;
_delay_ms(5000);
PORTJ = PORTJ | (0x01);
}
/***************************************************************/
/***************************************************************/
//ALGORITHM
//functions
void compass(int theta)    /** updates the direction of bot **/
{
phi = phi + theta;
if (phi > 0)
{
phi = phi % 360;
}
else if (phi < 0)
{
phi = 360 - abs(phi) % 360;
}

}

void config()
{
unsigned char floor_array[5]             = {1,0,0,1,0};
unsigned char house_total_requirement[5] = {2,2,2,1,2};
unsigned char which_material[9]          = {3,11,9,5,4,7,1,2,8};
}


void set_cost()
{
/** COST MATRIX **/

cost[0][1] = cost[1][0] = 1;
cost[1][2] = cost[2][1] = 1;   cost[10][11] = cost[11][10] = 1;
cost[2][3] = cost[3][2] = 1;   cost[11][12] = cost[12][11] = 1;
cost[3][4] = cost[4][3] = 1;   cost[12][13] = cost[13][12] = 1;
cost[4][5] = cost[5][4] = 1;   cost[13][14] = cost[14][13] = 1;
cost[5][6] = cost[6][5] = 1;   cost[14][15] = cost[15][14] = 1;
cost[6][7] = cost[7][6] = 1;   cost[15][16] = cost[16][15] = 1;
cost[7][8] = cost[8][7] = 1;   cost[16][1] = cost[1][16] = 1;


cost[4][14] = cost[14][4] = 4;  /* wall follower */
cost[8][9] = cost[9][8] = cost[10][9] = cost[9][10] = 3;   /** white line **/
cost[6][12] = cost[12][6] = 1; /** zig-zag **/
}

//node structure
struct node_alias
{
int came_from;
int cost_so_far;
int x;
int y;
int neigbour_node[3];
int priority_index;
};

struct node_alias node_[17];

void set_nodes()
{

//node 0
node_[0].neigbour_node[0] =  1;
node_[0].neigbour_node[1] = -1;
node_[0].neigbour_node[2] = -1;
node_[0].x=1;
node_[0].y=1;

//node 1
node_[1].neigbour_node[0] = 16;
node_[1].neigbour_node[1] = 2;
node_[1].neigbour_node[2] = -1;
node_[1].x = 1;
node_[1].y = 0;

//node 2
node_[2].neigbour_node[0] =  1;
node_[2].neigbour_node[1] =  3;
node_[2].neigbour_node[2] = -1;
node_[2].x=0;
node_[2].y=0;

//node 3
node_[3].neigbour_node[0] =  2;
node_[3].neigbour_node[1] =  4;
node_[3].neigbour_node[2] = -1;
node_[3].x=0;
node_[3].y=1;

//node 4
node_[4].neigbour_node[0] =  3;
node_[4].neigbour_node[1] =  5;
node_[4].neigbour_node[2] = 14;
node_[4].x=0;
node_[4].y=2;

//node 5
node_[5].neigbour_node[0] =  4;
node_[5].neigbour_node[1] =  6;
node_[5].neigbour_node[2] = -1;
node_[5].x=0;
node_[5].y=3;

//node 6
node_[6].neigbour_node[0] =  5;
node_[6].neigbour_node[1] =  7;
node_[6].neigbour_node[2] = 12;
node_[6].x=0;
node_[6].y=4;

//node 7
node_[7].neigbour_node[0] =  8;
node_[7].neigbour_node[1] =  6;
node_[7].neigbour_node[2] = -1;
node_[7].x=0;
node_[7].y=5;

//node 8
node_[8].neigbour_node[0] =  7;
node_[8].neigbour_node[1] =  9;
node_[8].neigbour_node[2] = -1;
node_[8].x=0;
node_[8].y=6;

//node 9
node_[9].neigbour_node[0] =  8;
node_[9].neigbour_node[1] =  10;
node_[9].neigbour_node[2] = -1;
node_[9].x=1;
node_[9].y=6;

//node 10
node_[10].neigbour_node[0] =  9;
node_[10].neigbour_node[1] = 11;
node_[10].neigbour_node[2] = -1;
node_[10].x=2;
node_[10].y=6;

//node 11
node_[11].neigbour_node[0] =  10;
node_[11].neigbour_node[1] =  12;
node_[11].neigbour_node[2] = -1;
node_[11].x=2;
node_[11].y=5;

//node 12
node_[12].neigbour_node[0] =  6;
node_[12].neigbour_node[1] = 11;
node_[12].neigbour_node[2] = 13;
node_[12].x=2;
node_[12].y=4;

//node 13
node_[13].neigbour_node[0] =  12;
node_[13].neigbour_node[1] =  14;
node_[13].neigbour_node[2] = -1;
node_[13].x=2;
node_[13].y=3;

//node 14
node_[14].neigbour_node[0] =  4;
node_[14].neigbour_node[1] =  13;
node_[14].neigbour_node[2] = 15;
node_[14].x=2;
node_[14].y=2;

//node 15
node_[15].neigbour_node[0] =  14;
node_[15].neigbour_node[1] =  16;
node_[15].neigbour_node[2] = -1;
node_[15].x=2;
node_[15].y=1;

//node 16
node_[16].neigbour_node[0] =  1;
node_[16].neigbour_node[1] = 15;
node_[16].neigbour_node[2] = -1;
node_[16].x=2;
node_[16].y=0;
}

//priority_queue
void put_prior(int temp[],int element, int prior )
{

node_[element].priority_index = prior;

if (temp[0]<0) //if first element = -1
{
temp[1]=temp[0];
temp[0]=element;
}
else
{
for (int n=0; n<20; n++)
{
if (prior<=node_[temp[n]].priority_index)
{
for (int i = 20; i > n; i--)
{
temp[i] = temp[i - 1];
}
temp[n] = element;
break;
}
}
}

}

int get_prior(int temp[])
{
int z=temp[0];
for (int n=0; n<20; n++)
{

temp[n] = temp[n+1];
//temp[n] = element;
//break;
}

return z;
}
int exist(int temp[],int element)
{
for (int i=0;i<20;i++)
{
if (element==temp[i])
return 1;
else if (i==20-1)
return 0;
}
return 0;
}
//heuristic function gives approx distance
int heuristic(struct node_alias a, struct node_alias b)
{
return (abs(a.x - b.x) + abs(a.y - b.y));
}

//a_star
// f = g + h
int path[30] = {-1}; //path

void a_star(int init , int finl)
{

set_cost();
set_nodes();

int current_node=init, goal_node = finl, loop_it=0;

int  frontier[40]    = {current_node,-1}; //test_node
int new_cost         =  0;
int priority;
int cost_so_far_updated[20] = {-1};

node_[current_node].cost_so_far = 0;

while(frontier[0]>0)
{
//while loop run
current_node = get_prior(frontier);

if (current_node==goal_node) //break if goal is found
{
break;
}

int i=0;
while (node_[current_node].neigbour_node[i]>0&&i<3)  //node_[current_node].neighbor_node[i]=next
{
//cost evaluation loop run
new_cost = node_[current_node].cost_so_far + cost[current_node][node_[current_node].neigbour_node[i]];
//if update
if((new_cost < node_[node_[current_node].neigbour_node[i]].cost_so_far)||!exist(cost_so_far_updated,node_[current_node].neigbour_node[i]))
{
//if statement run
node_[node_[current_node].neigbour_node[i]].cost_so_far = new_cost;
priority = new_cost + heuristic(node_[goal_node],node_[node_[current_node].neigbour_node[i]]);
put_prior(frontier,node_[current_node].neigbour_node[i],priority);

node_[node_[current_node].neigbour_node[i]].came_from=current_node;
cost_so_far_updated[loop_it]=current_node;
loop_it++;
}

i++;
}

}

int t=goal_node;
loop_it=0;
path[0]=finl;
//printf("\nPATH: %d ",finl);
while(node_[t].came_from!=init)
{
//printf("%d ",node_[t].came_from);
path[++loop_it]=node_[t].came_from;
t=node_[t].came_from;

}
path[++loop_it]=init;
}

//right_wls
void right_wls()
{
	right();
	_delay_ms(400);
	sensor_update();
	while(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	 {   
		 //velocity(150,150);
		 right();
		 _delay_ms(10);
		 sensor_update();
		 stop();
		 _delay_ms(10);
	 }
	 stop();
	 _delay_ms(100);
}
//left_wls
void left_wls()
{   
	left();
	_delay_ms(400);
	sensor_update();
	while(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	{   
		//velocity(150,150);
		left();
		_delay_ms(10);
		sensor_update();
		stop();
		_delay_ms(10);
		
	}
	stop();
	_delay_ms(100);
}
//align the center of node
void node_align()
{
_delay_ms(300);
forward_mm(80);
_delay_ms(300);
stop();
}
//wobble
void wobble()
{
//int p=0;
/*
while(p<5 && (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
{
sensor_update();
left_degrees(1);
p++;
}
while(p>0 && (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
{
sensor_update();
right_degrees(1);
p--;
}*/
sensor_update();
right_degrees(1);
//p++;

sensor_update();
left_degrees(2);
}

//blackline_follower

void blackline_follower()
{
while (1)
   {
     
lcd_home();  
lcd_string("WHILE");
    //update sensor after each task
sensor_update();
//print_all_sensors();

//B B B node
if (( whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)) //node
{
//unsigned packet=0;
//unsigned packet_threshold=1; //size of packet to be considered a node
/*
while(((whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)) && packet<=packet_threshold)
{  
packet++;
//lcd_print(1,14,packet,3);
sensor_update();
print_all_sensors();
}
*/
if ((whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
{
//node
stop();
print_all_sensors();
_delay_ms(100);
lcd_home();
lcd_string("NODE");
node_align();
break;
}
//packet=0;
}

//W B W
if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
{  
//print_all_sensors();
velocity(150,160);
forward();      //control
//_delay_ms(100); //control
//stop();
//_delay_ms(10);
//sensor_update();
}
/* //W B B
if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)
{
velocity(0,80);
forward();      //control
_delay_ms(250); //control
stop();
//sensor_update();
}

//B B W
if (whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
{  
velocity(80,0);
forward();      //control
_delay_ms(250); //control
stop();
//sensor_update();
}
*/
//B W W
if (whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
{
velocity(140,90);
forward();      //control
_delay_ms(250); //control
stop();
//sensor_update();
}

//W W B
if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)
{
velocity(90,140);
forward();      //control
_delay_ms(250); //control
stop();
//sensor_update();
}

if(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
{
wobble();
}
   }
}

void wall_follower()
{   
	    while(1)
		 {
	     lcd_home();
	     lcd_string("WHILE");
	     //update sensor after each task
	     sensor_update();
	     //print_all_sensors();

	     //B B B node
	    
		     if ((whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
		     {
			     //node
			     stop();
			     print_all_sensors();
			     _delay_ms(100);
			     lcd_home();
			     lcd_string("NODE");
			     node_align();
			     break;
		     }


	     //W B W
	     if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	     {
		     
		     velocity(150,160);
		     forward();      //control
		   
	     }
	    
	     //B W W
	     if (whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	     {
		     velocity(140,90);
		     forward();      //control
		     _delay_ms(250); //control
		     stop();
		     //sensor_update();
	     }

	     //W W B
	     if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)
	      {
		     velocity(90,140);
		     forward();      //control
		     _delay_ms(250); //control
		     stop();
		     //sensor_update();
	     }

			 while(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
			 {
				 sensor_update();
				 print_all_sensors();
                
				 if(left_prox_sensor > 100 && left_prox_sensor < (wall_threshold+3) && right_prox_sensor > 100 && right_prox_sensor < (wall_threshold+3))
				 {
					 velocity(170,170);
					 forward();
					 _delay_ms(300);
					 stop();
					 _delay_ms(10);
				 }
				 else if (left_prox_sensor>wall_threshold)
				 {
					 velocity(150,150);
					 soft_right();
					 _delay_ms(100);
					 stop();
					 _delay_ms(100);
				 }
				 else if (right_prox_sensor>wall_threshold)
				 {
					 velocity(150,150);
					 soft_left();
					 _delay_ms(100);
					 stop();
					 _delay_ms(100);
				 }
			
	     else if(left_prox_sensor<124 || right_prox_sensor<124)
		 {
			     wobble();
                }
			 }
		 }
	/* 
	while(1)
	{ 
		sensor_update();
		print_all_sensors();

		if(left_prox_sensor<wall_threshold && right_prox_sensor<wall_threshold)
		{
			velocity(150,150);
			forward();
			_delay_ms(200);
			stop();
			_delay_ms(100);
		}
		else if (left_prox_sensor>wall_threshold)
		{
			velocity(150,150);
			soft_right();
			_delay_ms(200);
				stop();
				_delay_ms(200);
		}
		else if (right_prox_sensor>wall_threshold)
		{
			velocity(150,150);
			soft_left();
			_delay_ms(200);
				stop();
				_delay_ms(200);
	    }
    }
	*/
}

void white_line_follower()
{

}

void zigzag()
{
 while (1)
 {
	 
	 lcd_home();
	 lcd_string("WHILE");
	 //update sensor after each task
	 sensor_update();
	 //print_all_sensors();

	 //B B B node
	 if (( whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)) //node
	 {
		
		 if ((whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)||(whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
		 {
			 //node
			 stop();
			 print_all_sensors();
			 _delay_ms(100);
			 lcd_home();
			 lcd_string("NODE");
			 node_align();
			 break;
		 }

	 }

	 //W B W
	 if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor > whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	 {
		 //print_all_sensors();
		 velocity(150,160);
		 forward();      //control

	 }
	 
	 //B W W
	 if (whiteline_left_sensor > whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	 {
		 velocity(140,90);
		 forward();      //control
		 _delay_ms(250); //control
		 stop();
		 //sensor_update();
	 }

	 //W W B
	 if (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor > whiteline_threshold_right)
	 {
		 velocity(90,140);
		 forward();      //control
		 _delay_ms(250); //control
		 stop();
		 //sensor_update();
	 }

	 if(whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right)
	 {
		  //wiper
		  unsigned tm=0;
		  
		  //left wiper
		  while(tm<12 && (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
		  {
			  //velocity(150,150);
			  left();
			  _delay_ms(20);
			  sensor_update();
			  stop();
			  _delay_ms(10);
			  tm++;
		  }
		  stop();
		  _delay_ms(100);
		  
		  //right wiper
		  while(tm<30 && tm>11 && (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
		  {
			  //velocity(150,150);
			  right();
			  _delay_ms(20);
			  sensor_update();
			  stop();
			  _delay_ms(10);
			  tm++;
		  }
		  stop();
		  _delay_ms(100);
		  
		  //left wiper
		  while(tm>29 && tm<40 && (whiteline_left_sensor < whiteline_threshold_left && whiteline_middle_sensor < whiteline_threshold_middle && whiteline_right_sensor < whiteline_threshold_right))
		  {
			  //velocity(150,150);
			  left();
			  _delay_ms(20);
			  sensor_update();
			  stop();
			  _delay_ms(10);
			  tm++;
		  }
		  stop();
		  _delay_ms(100);
	 }
	 
	
 }
}

void pick()
{
//pick code
right_wls(); //turning right

unsigned char i = 0;
servo_3(70);//motor 2
_delay_ms(1000);
servo_2(110); //base angle motor 1
_delay_ms(150);//30
servo_1(60); //grip motor 3
_delay_ms(300);
// servo_1_free();
//_delay_ms(1000);//?????????????1000

servo_3(50);//motor 2
_delay_ms(1500);

for (i = 60; i > 17; i=i-5) //ankle down
{
servo_1(i); //motor_3
_delay_ms(30);

}
_delay_ms(500);//need1000

for (i = 110; i > 0; i=i-5) //ankle down
{
servo_2(i); // motor 1
_delay_ms(80);

}
for (i =50; i >20; i=i-5) //ankle down
{
servo_3(i);//motor_2
_delay_ms(80);

}
_delay_ms(1000);//2000

left_wls();
}
void place_high()
{
//place code
unsigned char i = 0;

for(i=20;i<50;i=i+5)
{
servo_3(i);//motor 2
_delay_ms(100);
}
_delay_ms(300);
for(i=0;i<60;i=i+5)
{
servo_2(i); //base angle motor 1
_delay_ms(100);
}
for(i=17;i<40;i=i+2)
{
servo_1(i); //grip motor 3
_delay_ms(80);
}
_delay_ms(500);
servo_2(0);//base angle motor 1
_delay_ms(300);
servo_3(20);//motor 2
_delay_ms(300);
}
void place_low()
{
	//place code
	unsigned char i = 0;
/*
	for(i=20;i<40;i=i+5)
	{
		servo_3(i);//motor 2
		_delay_ms(100);
	}
	_delay_ms(300);
	*/
	for(i=0;i<80;i=i+5)
	{
		servo_2(i); //base angle motor 1
		_delay_ms(100);
	}
	for(i=17;i<40;i=i+2)
	{
		servo_1(i); //grip motor 3
		_delay_ms(80);
	}
	_delay_ms(500);
	servo_2(0);//base angle motor 1
	_delay_ms(300);
	servo_3(20);//motor 2
	_delay_ms(300);
}
void go_to_node(struct node_alias current,struct node_alias destination)
{  
int delta_angle = 0,line_vector = 0;
int temp_x=destination.x-current.x;
int temp_y=destination.y-current.y;
lcd_print(1,1,destination.x,1);
blackline_follower();
if (temp_x == 1 || temp_x == 2)
{
line_vector = 0;
}

else if (temp_x == -1 || temp_x == -2)
{
line_vector = 180;
}

else if (temp_y == 1 || temp_y == 2)
{
line_vector = 90;
}

else if (temp_y == -1 || temp_y == -2)
{
line_vector = 270;
}

delta_angle = line_vector - phi; /** angle to move **/

if (delta_angle == 0) //Aligned
{
//none;
compass(0); //this adds angle to phi
}

if (delta_angle == -180 || delta_angle == 180) //opposite
{

// left_t(2400); //left(180deg)
left_degrees(180);
//printf("\nLEFT function call");
compass(180); //this adds angle to phi

}

else if (delta_angle == 90 || delta_angle == -270) //off by 90deg
{
// left_t(2400); //left(90deg)
left_degrees(90);
//printf("\nLEFT function call");
compass(90); //this adds angle to phi
}
else if (delta_angle == -90 || delta_angle == 270) //off by 90deg
{  
//right_t(2400); //right(90deg)
right_degrees(80);
//printf("\nRIGHT function call");
compass(-90); //this adds angle to phi
}

// forward loop till node

if (current.y == 4 && (destination.x == 0 || destination.x == 2))
{
zigzag();
}
else if (current.y == 6 && (destination.x == 0 || destination.x == 2))
{
white_line_follower();
}

blackline_follower();
//forward_till_next_node
}


//Main Function

int main(void)
{    
init_devices();
lcd_set_4bit(); //4bit mode
lcd_init();     //initialize lcd
//servo initial position
servo_2(20);//motor 2
_delay_ms(1000);
servo_3(0);
//testing
//pick();
//
//place_high();
//_delay_ms(3000);
//place_low();
//wall_follower();
//path plan

   blackline_follower();
   right_wls();
   blackline_follower();
   right_wls();
   blackline_follower();
   pick();
   blackline_follower();
   right_wls();
   wall_follower();
   left_wls();
   blackline_follower();
   blackline_follower();
   right_wls();
   place_high();
   right_wls();
   right_wls();
   zigzag();
   left_wls();
   blackline_follower();
   pick();
   blackline_follower();
   left_wls();
   wall_follower();
   place_low();
buzzer();
}
