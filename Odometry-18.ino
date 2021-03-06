/* Odomotery Exercise - Group M1B - Nov 2018
 * 
 * Authors:
 * Samuel Beardmore Alonso
 * Chris Bennett 
 */


// ************ INITIALISATION OF ENVIRONMENT ************

//Defines for MD25 registers
#define MD25    0x58    //MD25 Address
#define SPEED   0x00    //Desired Speed Register
#define TURN    0x01    //Turn value Register
#define E_1     0x02    //Encoder 1 Register
#define E_2     0x06    //Encoder 2 Register
#define ACC     0x0E    //Acceleration Register
#define CMD     0x10    //Command Register
#define MODE    0x0F    //Control Mode


//Include wire and servo libraries and create servo1 object
#include <Wire.h>
#include <Servo.h>
Servo servo_1;
int pos = 0;


//Assign variables for MD25 use
int spd = 0;                          //Speed
int turn = 0;                         //Turning value as difference in speeds
int mode_val = 3;                     //MD25 mode value
float Dist_1 = 0.0;                   //Encoder 1 distance
float Dist_2 = 0.0;                   //Encoder 2 distance
int a_rate = 5;                       //Acceleration co-efficient (values of 1 to 10, https://www.robot-electronics.co.uk/htm/md25i2c.htm)
int radius = 55;                      //Wheel radius
float r_coeff = radius * 0.017453;    //Coefficient of rotation = wheel radius * (2 pi / 360)


//Assign variable to pins for easy pin changes.
int ledPin = 8;
int buzzPin = 9;
int servoPin = 10;


//Initialise the servo and MD25 for use in the main loop.
void setup() {
  //Serial used for de-bugging
  Serial.begin(9600);

  //Assign the servo pin to the servo object and set the servo to position 0
  servo_1.attach(servoPin);
  servo_1.write(0);

  //Initialise the MD25 in mode 3, reset encoder count and set acceleration rate.
  Wire.begin(); 
  Wire.beginTransmission(MD25);
  Wire.write(MODE);
  Wire.write(mode_val);
  Wire.beginTransmission(MD25);
  Wire.write(CMD);
  Wire.write(0x20);
  Wire.beginTransmission(MD25);
  Wire.write(ACC);
  Wire.write(a_rate);
  Wire.endTransmission();
}



// ************ MAIN LOOP FUNCTION ************

//Main loop
void loop() {
  //Debugging with serial
  if(Serial.available() > 0){
    serial_debug();
  } else {
    delay(250);
  }
}



// ************ MISC. FUNCTIONS ************

void indicate(){
  /* This function indicates when a waypoint has been reached by flashing an LED and sounding a buzzer.
   * It does not require any parameters.
   * Author: Samuel Beardmore Alonso
   */
   for(int i = 0; i < 5; i++){    //For loop allows fast switching of LED & Buzzer
     digitalWrite(ledPin, HIGH);  //Turn LED on
     delay(250);                  //Wait 0.25 seconds
     digitalWrite(ledPin, LOW);   //Turn LED off
     tone(buzzPin, 262, 250);     //Play 262Hz tone for 0.25 second
   }
}


void dispense(){
  /* This function dispenses a marker for special waypoints by rotating the dispenser servo by 1/12 of a full turn.
   * The dispensing servo has 6 positions over an arc of 180 degrees giving 36 degree increments.
   * Author: Samuel Beardmore Alonso
   */
   pos += 36;         //Increment by 36 degrees
   servo_1.write(pos); //Send new position to servo
}



// ************ MOVEMENT FUNCTIONS ************

void move_distance(int dist){
  
}


void arc_distance(int radius, int angle){
  
}

void turn_static(int angle){
  
}



// ************ MD25 INTERFACE FUNCTIONS ************

void set_speed(int v){
  /* This function sets the robot speed to the value passed in the argument v (int).
   * Author: Samuel Beardmore Alonso
   */
   Wire.beginTransmission(MD25);
   Wire.write(SPEED);
   Wire.write(v);
   Wire.endTransmission();
}


void set_turn(int t){
  /* This function sets the robot turn rate to the value passed in the argument t (int).
   * Author: Samuel Beardmore Alonso
   */
   Wire.beginTransmission(MD25);
   Wire.write(TURN);
   Wire.write(t);
   Wire.endTransmission();
}


long read_encoder(unsigned int reg){
  /* This function reads the 4 byte encoder value from the MD25.
   * The address of the encoder to be read (Hexadecimal) is taken as the argument reg.
   * The function returns the distance as a long number in mm.
   * Author: Samuel Beardmore Alonso
   * Bit Shifting method taken from https://github.com/riccardofelluga/MD25-I2C-Arduino-Library/blob/master/MD25.h
   */
  Wire.beginTransmission(MD25);
  Wire.write(reg);
  Wire.endTransmission();


  Wire.requestFrom(MD25, 4);  
  while(Wire.available() < 4);  
  long theta = Wire.read();
  theta <<= 8;
  theta += Wire.read();
  theta <<= 8;
  theta += Wire.read();
  theta <<= 8;
  theta  +=Wire.read();

  long dist = theta * r_coeff;
  return(dist);
}



// ************ DEBUG FUNCTIONS ************

void serial_debug(){
/* This function is designed to accept strings over serial and interpret them as basic instructions for debugging.
 *  m = move straight (distance)
 *  t = turn an angle whilst stopped (angle)
 *  a = arc (radius, angle)
 *  r = print encoder values to serial
 *  example: "a 150 90" Will call the arc_distance function with a radius of 150mm to turn through a 90 degree arc.
 * Author: Samuel Beardmore Alonso
 */
  
  String code = Serial.readString();
  
  if(code[0] == 'm'){     //If a move command
    int d = code[2] - 0;  //Convert char to int
    move_distance(d);     //Call move function
  }

  if(code[0] == 't'){     //If a move command
    int a = code[2] - 0;  //Convert char to int
    turn_static(a);       //Call move function
  }


  if(code[0] == 'a'){     //If an arc command
    int r = code[2] - 0;  //Convert char to int
    int a = code[4] - 0;  //Convert char to int
    arc_distance(r, a);   //Call arc function
  }

  if(code[0] == 'r'){                   //If a read command
    Serial.println(read_encoder(E_1));  //send back encoder 1 value
    Serial.println(read_encoder(E_2));  //send back encoder 2 value
  }
}
