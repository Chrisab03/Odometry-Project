/* Odomotery Exercise - Group M1B - Nov 2018
 * 
 * Authors:
 * Chris Bennett
 * Samuel Beardmore Alonso
 * 
 */


//Assign variable to pins for easy pin changes.
int ledPin = 8;
int buzzPin = 9;
int servoPin = 10;


//Include servo library and create servo1 object
#include <Servo.h>
Servo servo1;
int pos = 0;


//Assign the servo pin to the servo object and set the servo to position 0
void setup() {
  servo1.attach(servoPin);
  servo1.write(0);
}

//Main loop
void loop() {
}


void indicate(){
  /* This function indicates when a waypoint has been reached by flashing an LED and sounding a buzzer.
   * It does not require any parameters.
   * Author: Samuel Beardmore Alonso */
   for(int i = 0; i < 5; i++){    //For loop allows fast switching of LED & Buzzer
     digitalWrite(ledPin, HIGH);  //Turn LED on
     delay(250);                  //Wait 0.25 seconds
     digitalWrite(ledPin, LOW);   //Turn LED off
     tone(buzzPin, 262, 250);     //Play 262Hz tone for 0.25 second
   }
}


void dispense(){
  /* This function dispenses a marker by rotating the dispenser servo by 1/12 of a full turn.
   *  The dispensing servo has 6 positions over an arc of 180 degrees giving 36 degree increments.
   *  Author: Samuel Beardmore Alonso
   */
   pos += 36;         //Increment by 36 degrees
   servo1.write(pos); //Send new position to servo
}
