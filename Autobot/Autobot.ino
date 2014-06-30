// Code for Auto-bot Robocon 2014

/******************************************** SELF- NOTES ************************************************/
// Check that play times for motors are less than delays
// Make variable mode = 0 after execution of each module
// delay to stabilize pwm
// adjustment of angles for pole-walk needed
// direction of zero different for 3-4 and 1-2
// motors with EMs engaged should rotate opposite to desired direction
// Explain delays
// Multiply angles for motor 1,3 or 2,4 by -1
// pneumatics
// include fast libraries
// add setup delay for herkulex
// Auto bot sagging in Pole-walk mode

#include <Herkulex.h>
#include <SoftwareSerial.h>

//Instantiating classes for motors
HerkulexClass Herkulex12;                           // Class for motors 1 & 2
HerkulexClass Herkulex34;                           // Class for motors 3 & 4

/************************************** VARIABLE DEFINITIONS *******************************************/
// Motor IDs
int n1 = 222;
int n2 = 220;
int n3 = 221;
int n4 = 219;

// Mode selector variables
int pwm = 0;
int mod = 0;

// To start and terminate see-saw and swing mode
int count1 = 0;

// Read HALL pin of hall sensor
int hall = 0;

//Define pins for pneumatic valves
#define PNEU1 8                                     // piston moves up by 10 cm
#define PNEU2 9                                    //piston moves down by 15 cm

//Define pin for hall sensor
#define HALL 5

// Define pin for flag hoisting
#define SERVO A0

// Define pins for electromagnets
#define EM1 10
#define EM2 11
#define EM3 12
#define EM4 13

// Define pin for communication
#define SIG 5

void setup ()
{
  /********************************************* PIN DEFINITIONS *****************************************/  
  // Electromagnet Pins
  pinMode (EM1,OUTPUT);
  pinMode (EM2,OUTPUT);
  pinMode (EM3,OUTPUT);
  pinMode (EM4,OUTPUT); 
  
  // Pneumatics
  pinMode (PNEU1,OUTPUT);
  pinMode (PNEU2,OUTPUT);
  pinMode (HALL,INPUT);
  
  // Communications pin
  pinMode (SIG,INPUT);
  
  /********************************************** MOTOR SETUP *******************************************/
  //Serial monitor
  Serial.begin (115200);                  
  // Initialize Serial Communication
  Herkulex12.beginSerial1 (115200);
  Herkulex34.beginSerial3 (115200);
  
  // Clear errors
  Herkulex12.reboot(n1);
  Herkulex12.reboot(n2);
  Herkulex34.reboot(n3);
  Herkulex34.reboot(n4);
  delay(100);
  
  // Re-initialize motors
  Herkulex12.initialize();
  Herkulex34.initialize();
  delay(100);
  
  // Switch on torque
  Herkulex12.torqueON(n1);
  Herkulex12.torqueON(n2);
  Herkulex34.torqueON(n3);
  Herkulex34.torqueON(n4);  
}

void loop ()
{
  pwm = pulseIn (SIG,HIGH);
  switch (mod)
  {
    case 1: // Code for see-saw
            // Loop ensures that motors clamp only once
            if (count1 == 0)
            {
              // Move motors by a small angle
              Herkulex12.moveOneAngle (n1,-10,500,LED_PINK);
              Herkulex12.moveOneAngle (n2,10,500,LED_PINK);
              Herkulex34.moveOneAngle (n3,10,500,LED_PINK);
              Herkulex34.moveOneAngle (n4,-10,500,LED_PINK);
              count1 = 1;
            }
            // Unclamping when a pwm of 250 is given to the Auto bot
            else if (pwm > 1800)
            {
               delay (1000);           // for manual bot to take hold of auto bot
               // Move motors to initial position
               Herkulex12.moveOneAngle (n1,0,1000,LED_PINK);
               Herkulex12.moveOneAngle (n2,0,1000,LED_PINK);
               Herkulex34.moveOneAngle (n3,0,1000,LED_PINK);
               Herkulex34.moveOneAngle (n4,0,1000,LED_PINK);
               count1 = 0;
               mod = 0;
            }
            break;
            
   case 2:  // Code for swing
            // Loop ensures that motors clamp only once
            if (count1 == 0)
            {
              //delay (1000);                          // for auto bot to disenagage from manual bot
              // Move motors by a small angle
              Herkulex12.moveOneAngle (n1,-10,500,LED_CYAN);
              Herkulex12.moveOneAngle (n2,10,500,LED_CYAN);
              Herkulex34.moveOneAngle (n3,10,500,LED_CYAN);
              Herkulex34.moveOneAngle (n4,-10,500,LED_CYAN);
              count1 = 1;
            }
            // Unclamping when a pwm of 250 is given to the Auto bot
            else if (pwm > 1800)
            {
               delay (1000);                           // for manual bot to take hold of auto bot
               // Move motors to initial position
               Herkulex12.moveOneAngle (n1,0,1000,LED_CYAN);
               Herkulex12.moveOneAngle (n2,0,1000,LED_CYAN);
               Herkulex34.moveOneAngle (n3,0,1000,LED_CYAN);
               Herkulex34.moveOneAngle (n4,0,1000,LED_CYAN);
               count1 = 0;
               mod = 0;
            }
            break;
            
   case 3:  // Code for pole-walk
            // Change in colour of LED indicates change of pole (for soft testing purposes)
            // Move all motors to initial position
            if (count1 == 0)
            {
              Herkulex12.moveOneAngle(n1,0,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n2,0,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n3,0,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n4,0,1000,LED_GREEN); 
              
              delay (1000);                               // Should be less than release time, flipper must rest on pole 
              
              digitalWrite (EM1,HIGH);                    // Engage flipper-1 electromagnet
              Serial.println ('Robot on pole 1');
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              
              // Move motors 3 and 4 with respect to auto bot (calibrate angle through testing)
              //Herkulex34.moveOneAngle(n3,-45,200,LED_BLUE);
              //Herkulex34.moveOneAngle(n4,-45,200,LED_BLUE);
                                      
              // Move motors 1 and 2 to move auto bot
              Herkulex12.moveOneAngle(n1,-45,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n2,45,1000,LED_GREEN);            
             
              delay (2500);                                // Time to ensure the motors and bot have moved to the desired position (must be more than motor playtime)
              
              digitalWrite (EM4,HIGH);                    // Engage flipper-4 electromagnet
              delay (1000);                               // Duration for which bot holds both poles 1 and 2
              
              digitalWrite (EM1,LOW);                     // Disengage flipper-1 electromagnet
              Serial.println ('Robot on pole 2');  
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              // Move motors 1 and 2 with respect to auto bot body
              Herkulex12.moveOneAngle(n1,135,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n2,-135,1000,LED_GREEN);
              
              // Move motors 3 and 4 to move auto bot
              Herkulex34.moveOneAngle(n3,-135,1000,LED_GREEN);
              Herkulex34.moveOneAngle(n4,135,1000,LED_GREEN);
              
              delay (2500);                                // Time to ensure the motors and bot have moved to the desired position (must be more than motor playtime)
              
              digitalWrite (EM2,HIGH);                    // Engage flipper-2 electromagnet   
              Serial.println('Robot entirely on pole 2');
              
              delay (1000);                               // Duration for which both EMs are glued to pole 2 (can be done away with)
              
              digitalWrite (EM4,LOW);                     // Disengage flipper-4 electromagnet
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              // Move motors 3 and 4 with respect to auto bot body
              Herkulex34.moveOneAngle(n3,45,1000,LED_BLUE);
              Herkulex34.moveOneAngle(n4,-45,1000,LED_BLUE);
              
              // Move motors 1 and 2 to move auto bot
              Herkulex12.moveOneAngle(n1,-45,1000,LED_BLUE);
              Herkulex12.moveOneAngle(n2,45,1000,LED_BLUE);
              
              delay (2500);                                // Time to ensure the motors and bot have moved to the desired position (must be more than motor playtime)
              
              digitalWrite (EM4, HIGH);                   // Engage flipper-4 electromagnet
              Serial.println('Robot is on poles 2 and 3');
              delay (1000);                               // Duration for which bot holds both poles 2 and 3
              
              digitalWrite (EM2,LOW);                     // Disengage flipper-2 electromagnet
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              
              // Move motors 1 and 2 with respect to auto bot body
              Herkulex12.moveOneAngle(n1,135,1000,LED_GREEN);
              Herkulex12.moveOneAngle(n2,-135,1000,LED_GREEN);
              
              // Move motors 3 and 4 to move auto bot
              Herkulex34.moveOneAngle(n3,-135,1000,LED_GREEN);
              Herkulex34.moveOneAngle(n4,135,1000,LED_GREEN);
              
              delay (2500);                                // Time to ensure the motors and bot have moved to the desired position (must be more than motor playtime)
              
              digitalWrite (EM2,HIGH);                    // Engage flipper-2 electromagnet
              Serial.println('Robot is entirely on pole 3');
              delay (1000);
              
              digitalWrite (EM4,LOW);                     // Disengage fippler-4 electromagnet
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              // Move motors 3 and 4 with respect to auto bot body
              Herkulex34.moveOneAngle(n3,45,1000,LED_BLUE);
              Herkulex34.moveOneAngle(n4,-45,1000,LED_BLUE);
              
              // Move motors 1 and 2 to move auto bot
              Herkulex12.moveOneAngle(n1,0,1000,LED_BLUE);
              Herkulex12.moveOneAngle(n2,0,1000,LED_BLUE);
              Serial.println('Robot is on pole 3');
              
              delay (2500);                                // Time to ensure the motors and bot have moved to the desired position (must be more than motor playtime)
              
              digitalWrite (EM3,HIGH);                    // Engage flipper-3 electromagnet
              delay (1000);                               // Duration for which bot holds both poles 2 and 3
              
              digitalWrite (EM2,LOW);                     // Disengage flipper-2 electromagnet
              
              // Move motors 1 and 2 with respect to auto bot body
              // Herkulex12.moveOneAngle(n1,0,200,LED_GREEN);
              // Herkulex12.moveOneAngle(n2,0,200,LED_GREEN);
              
              delay (1000);                                // Time for bot to stabilize after locking onto to the pole (needs to be calibrated)
              // Move motors 3 and 4 to move auto bot
              Herkulex34.moveOneAngle(n3,0,1000,LED_GREEN);
              Herkulex34.moveOneAngle(n4,0,1000,LED_GREEN);
              Serial.println('Robot is on pole 4');
              count1 = 1;
            }
            else if (pwm > 1800)
            {
              digitalWrite (EM3,LOW);                     // Disengage flipper-3 electromagnet
              delay (1000);                               
              Serial.println ('Pole-walk complete');
              mod = 0; 
              count1 = 0;
            }
            
      
   case 4: // Code for ladder climbing
           delay (1000);                 // for manual to place Autobot
           ladder_climb();               // jump from rung-1 to rung-2
           ladder_climb();               // jump from rung-2 to rung-3
           ladder_climb();               // jump from rung-3 to rung-4
           ladder_climb();               // final jump from rung-4 to flat top
           
           delay (1000);
           analogWrite(SERVO,50);         //flag is hoisted           
           mod = 0;
           break;
    
   default: //Mode Selector
             delay (500);                // for pwm to stabilize
             // Manual not connected to auto
             if (pwm < 200)
             mod = 0;
             // pwm for see-saw
             else if (pwm > 200 && pwm < 600)
             mod = 1;
             // pwm for swing
             else if (pwm > 600 && pwm < 1000)
             mod = 2;
             // pwm for pole-walk
             else if (pwm > 1000 && pwm < 1400)
             mod = 3;
             // pwm for ladder
             else if (pwm > 1400 && pwm < 1800)
             mod = 4;
             // pwm used when manual is normally in contact with auto. Send to terminate see-saw and swing modes
             else 
             break;            
  }
}

void ladder_climb()
{
  // Extension of lower piston by 15 cm  
  digitalWrite(PNEU2,HIGH);
  // Delay to be calibrated to ensure equal extension + retraction time
  delay (1000);
  // Extension of upper piston by 10 cm
  digitalWrite(PNEU1,HIGH);
          
  hall = digitalRead(HALL);
  if(hall == 0)
     {
        digitalWrite(PNEU1,LOW);             
        digitalWrite(PNEU2,LOW);
     }
  delay(1000);           
}


