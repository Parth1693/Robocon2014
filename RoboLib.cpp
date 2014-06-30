#include "Arduino.h"
#include "RoboLib.h"
#include <digitalWriteFast.h>
#include "PS2X_lib.h"
#include "SabertoothSimplified.h"

//******************** Main code ********************//
unsigned int mode = 1;    //Seesaw, Pole walk, Swing, Jungle gym, Run, Drive adjust
bool registers[8];
bool sigBot=false;
bool swpull=false;
bool holdauto=true;
int sigNo=0;
bool pwmAdjust=false;
//**********************************************************************//

//****************** PID ********************//
double ITerm=0;
double kp=1;
double ki=0;
double kd=0;
int errorPrevious;
bool track=false;
//////////////////////////////////Line Follower//////////////////////////
//int line_val0 = 0;
int line_val1 = 0;
int line_val2 = 0;
int line_val3 = 0;
int line_val4 = 0;
int line_val5 = 0;
int line_val6 = 0;
bool override=true;

//******************** PS2 controller ********************//
int error = 0; 
byte type = 0;
byte vibrate = 0;
PS2X ps2x; // create PS2 Controller Class
//**********************************************************************//

//******************** Pneumatics ********************//
int ver_cylinder_pos=0;
int hor_cylinder_pos=0;
int arm_cylinder_pos=1;
int ver_cylinder_target=0;
int hor_cylinder_target=0;
int arm_cylinder_target=1;
int ver_cylinder_status=0;
int hor_cylinder_status=0;
int arm_cylinder_status=0;
//**********************************************************************//


int holding_motor_pwm=150;
int holding_motor_status=0;
//////////******************************Legacy Code******************************//////////
//////////**********************************************************************//////////

/*
Configure pins of arduino as either INPUT/OUTPUT and initialize them as either HIGH/LOW. Comment out pins not in use.
*/
void arduConfig()
{
    pinMode(ver_hall_1,INPUT);
    pinMode(ver_hall_2,INPUT);
    pinMode(ver_hall_3,INPUT);
    pinMode(ver_hall_4,INPUT);
    pinMode(hor_hall_1,INPUT);
    pinMode(hor_hall_2,INPUT);
    pinMode(hor_hall_3,INPUT);
    pinMode(hor_hall_4,INPUT);
    pinMode(ver_cylinder_ext,OUTPUT);
    pinMode(ver_cylinder_ret,OUTPUT);
    pinMode(hor_cylinder_ext,OUTPUT);
    pinMode(hor_cylinder_ret,OUTPUT);
    pinMode(arm_cylinder_ext,OUTPUT);
    pinMode(arm_cylinder_ret,OUTPUT);
    
    pinMode(carriage_clamp_1,OUTPUT);
    pinMode(carriage_clamp_2,OUTPUT);
    pinMode(autobot_holder,OUTPUT);
    pinMode(swing_pulling,OUTPUT);
    
  pinMode(line_1, INPUT);
  pinMode(line_2, INPUT);
  pinMode(line_3, INPUT);
  pinMode(line_4, INPUT);
  pinMode(line_5, INPUT);
  pinMode(line_6, INPUT);
  digitalWrite(line_1,HIGH);
  digitalWrite(line_2,HIGH);
  digitalWrite(line_3,HIGH);
  digitalWrite(line_4,HIGH);
  digitalWrite(line_5,HIGH);
  digitalWrite(line_6,HIGH);

    pinMode(modeled1,OUTPUT);
    pinMode(modeled2,OUTPUT);
    pinMode(modeled3,OUTPUT);
    pinMode(modeled4,OUTPUT);
    pinMode(modeled5,OUTPUT);
    pinMode(modeled6,OUTPUT);
    
    pinMode(pwm,OUTPUT);
      analogWrite(pwm,0);
    pinMode(holdir,OUTPUT);

    digitalWrite(modeled1,LOW);
    digitalWrite(modeled2,LOW);
    digitalWrite(modeled3,LOW);
    digitalWrite(modeled4,LOW);
    digitalWrite(modeled5,LOW);
    digitalWrite(modeled6,LOW);
    
}

void PS2_setup()
{
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATTN, PS2_DAT, pressures, rumble);
  if(error == 0) Serial.println("Found Controller, configured successful");
  else if(error == 1) Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2) Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3) Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  type = ps2x.readType(); 
     switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
       case 2:
         Serial.println("GuitarHero Controller Found");
       break;
     }
}
     
///////******************************************************************************************/////
//void clearRegisters(){
//  for(int i = 7; i>=0; i--){
//     registers[i] = HIGH;
//  }
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void setRegisterPin(int index, int value){
//  registers[index-1] = value;
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void writeRegisters(){
//  digitalWrite(RCLK_Pin, LOW);
//  for(int i = 7; i>=0; i--){
//    digitalWrite(SRCLK_Pin, LOW);
//    
//    int val = registers[i];
//    digitalWrite(SER_Pin, val);
//    
//    digitalWrite(SRCLK_Pin, HIGH);
//  }
//  digitalWrite(RCLK_Pin, HIGH);
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
void setModeLED(int modeLED)
{
for(int i=0;i<=6;i++)
{
  if(i==modeLED) digitalWrite(i,LOW);
  else digitalWrite(i,HIGH);
}

}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void ssLoop()
//{
//    setModeLED(1);
//    ps2x.read_gamepad(false, vibrate);
//if(ps2x.Button(PSB_L1))
//if(ps2x.Button(PSB_R1))
//
//if(ps2x.Button(PSB_PAD_UP))
//if(ps2x.Button(PSB_PAD_RIGHT))
//if(ps2x.Button(PSB_PAD_DOWN))
//if(ps2x.Button(PSB_PAD_LEFT))
//
//if(ps2x.Button(PSB_GREEN))
//if(ps2x.ButtonPressed(PSB_RED))
//if(ps2x.ButtonReleased(PSB_PINK))
//if(ps2x.NewButtonState(PSB_BLUE));
//
//    ver_cylinder_goto(0);
//    hor_cylinder_goto(0);
//    
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void swingLoop()
//{
//  setModeLED(2);
//  ps2x.read_gamepad(false, vibrate);
//if(ps2x.Button(PSB_L1))
//if(ps2x.Button(PSB_R1))
//
//if(ps2x.Button(PSB_PAD_UP))
//if(ps2x.Button(PSB_PAD_RIGHT))
//if(ps2x.Button(PSB_PAD_DOWN))
//if(ps2x.Button(PSB_PAD_LEFT))
//
//if(ps2x.Button(PSB_GREEN))
//if(ps2x.ButtonPressed(PSB_RED))
//if(ps2x.ButtonReleased(PSB_PINK))
//if(ps2x.NewButtonState(PSB_BLUE));
//
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void poleLoop()
//{
//  setModeLED(3);
//  ps2x.read_gamepad(false, vibrate);
//if(ps2x.Button(PSB_L1))
//if(ps2x.Button(PSB_R1))
//
//if(ps2x.Button(PSB_PAD_UP))
//if(ps2x.Button(PSB_PAD_RIGHT))
//if(ps2x.Button(PSB_PAD_DOWN))
//if(ps2x.Button(PSB_PAD_LEFT))
//
//if(ps2x.Button(PSB_GREEN))
//if(ps2x.ButtonPressed(PSB_RED))
//if(ps2x.ButtonReleased(PSB_PINK))
//if(ps2x.NewButtonState(PSB_BLUE));
//
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void gymLoop()
//{
//  setModeLED(4);
//  ps2x.read_gamepad(false, vibrate);
//if(ps2x.Button(PSB_L1))
//if(ps2x.Button(PSB_R1))
//
//if(ps2x.Button(PSB_PAD_UP))
//if(ps2x.Button(PSB_PAD_RIGHT))
//if(ps2x.Button(PSB_PAD_DOWN))
//if(ps2x.Button(PSB_PAD_LEFT))
//
//if(ps2x.Button(PSB_GREEN))
//if(ps2x.ButtonPressed(PSB_RED))
//if(ps2x.ButtonReleased(PSB_PINK))
//if(ps2x.NewButtonState(PSB_BLUE));
//
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void runLoop()
//{
//  setModeLED(0);
//  ps2x.read_gamepad(false, vibrate);
//if(ps2x.Button(PSB_L1))
//if(ps2x.Button(PSB_R1))
//
//if(ps2x.Button(PSB_PAD_UP))
//if(ps2x.Button(PSB_PAD_RIGHT))
//if(ps2x.Button(PSB_PAD_DOWN))
//if(ps2x.Button(PSB_PAD_LEFT))
//
//if(ps2x.Button(PSB_GREEN))
//if(ps2x.ButtonPressed(PSB_RED))
//if(ps2x.ButtonReleased(PSB_PINK))
//if(ps2x.NewButtonState(PSB_BLUE));
//
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////******************************************************************************************/////
//void adjLoop()
//{
//  setModeLED(0);
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////

void signalChild(int sigNum)
{
  if(sigNum==1) analogWrite(commpin,commsee);
  else if(sigNum==2) analogWrite(commpin,commswing);
  else if(sigNum==3) analogWrite(commpin,commpole);
  else if(sigNum==4) analogWrite(commpin,commgym);
  else if(sigNum==5) analogWrite(commpin,commsigd);
  else analogWrite(commpin,0);
}
//////////////////////////////////////////    VERTICAL CYLINDER FUNCTION    ////////////////////////

void ver_cylinder_goto(int ver_cylinder_target)
{
  switch(ver_cylinder_target)
  {

  case 1:            //if target is 1 i.e. base
    if(ver_cylinder_pos>1)
    {
      if(digitalReadFast(ver_hall_1)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(ver_cylinder_ret,LOW);     //Piston Stops Retraction
        ver_cylinder_pos=1;                      //Position Reached
      }
    }
    else if(ver_cylinder_pos<1)
    {
      if(digitalReadFast(ver_hall_1)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(ver_cylinder_ext,LOW);     //Piston Stops extension
        ver_cylinder_pos=1;                      //Position Reached
      }
    }
    break;                                //break after case 1


  case 2:            //if target is 2
    if(ver_cylinder_pos>2)
    {
      if(digitalReadFast(ver_hall_2)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(ver_cylinder_ret,LOW);     //Piston Stops Retraction
        ver_cylinder_pos=2;                      //Position Reached
      }
    }
    else if(ver_cylinder_pos<2)
    {
      if(digitalReadFast(ver_hall_2)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(ver_cylinder_ext,LOW);     //Piston Stops extension
        ver_cylinder_pos=2;                      //Position Reached
      }
    }
    break;                                //break after case 2


  case 3:            //if target is 3
    if(ver_cylinder_pos>3)
    {
      if(digitalReadFast(ver_hall_3)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(ver_cylinder_ret,LOW);     //Piston Stops Retraction
        ver_cylinder_pos=3;                      //Position Reached
      }
    }
    else if(ver_cylinder_pos<3)
    {
      if(digitalReadFast(ver_hall_3)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(ver_cylinder_ext,LOW);     //Piston Stops extension
        ver_cylinder_pos=3;                      //Position Reached
      }
    }
    break;                                //break after case 3

  case 4:            //if target is 4 i.e. top
    if(ver_cylinder_pos>4)
    {
      if(digitalReadFast(ver_hall_4)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(ver_cylinder_ret,LOW);     //Piston Stops Retraction
        ver_cylinder_pos=4;                      //Position Reached
      }
    }
    else if(ver_cylinder_pos<4)
    {
      if(digitalReadFast(ver_hall_4)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(ver_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(ver_cylinder_ext,LOW);     //Piston Stops extension
        ver_cylinder_pos=4;                      //Position Reached
      }
    }
    break;                                //break after case 4

  }
}

////////////////////////      HORIZONTAL CYLINDER FUNCTION    //////////////////////////////////
void hor_cylinder_goto(int hor_cylinder_target)            
{
  switch(hor_cylinder_target)
  {

  case 1:            //if target is 1 i.e. base
    if(hor_cylinder_pos>1)
    {
      if(digitalReadFast(hor_hall_1)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(hor_cylinder_ret,LOW);     //Piston Stops Retraction
        hor_cylinder_pos=1;                      //Position Reached
      }
    }
    else if(hor_cylinder_pos<1)
    {
      if(digitalReadFast(hor_hall_1)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(hor_cylinder_ext,LOW);     //Piston Stops extension
        hor_cylinder_pos=1;                      //Position Reached
      }
    }
    break;                                //break after case 1


  case 2:            //if target is 2
    if(hor_cylinder_pos>2)
    {
      if(digitalReadFast(hor_hall_2)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(hor_cylinder_ret,LOW);     //Piston Stops Retraction
        hor_cylinder_pos=2;                      //Position Reached
      }
    }
    else if(hor_cylinder_pos<2)
    {
      if(digitalReadFast(hor_hall_2)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(hor_cylinder_ext,LOW);     //Piston Stops extension
        hor_cylinder_pos=2;                      //Position Reached
      }
    }
    break;                                //break after case 2


  case 3:            //if target is 3
    if(hor_cylinder_pos>3)
    {
      if(digitalReadFast(hor_hall_3)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(hor_cylinder_ret,LOW);     //Piston Stops Retraction
        hor_cylinder_pos=3;                      //Position Reached
      }
    }
    else if(hor_cylinder_pos<3)
    {
      if(digitalReadFast(hor_hall_3)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(hor_cylinder_ext,LOW);     //Piston Stops extension
        hor_cylinder_pos=3;                      //Position Reached
      }
    }
    break;                                //break after case 3

  case 4:            //if target is 4 i.e. top
    if(hor_cylinder_pos>4)
    {
      if(digitalReadFast(hor_hall_4)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ret,HIGH);    //Retract
      }
      else
      {
        digitalWriteFast(hor_cylinder_ret,LOW);     //Piston Stops Retraction
        hor_cylinder_pos=4;                      //Position Reached
      }
    }
    else if(hor_cylinder_pos<4)
    {
      if(digitalReadFast(hor_hall_4)==HIGH)  //read appropriate sensor, if its not detecting
      {
        digitalWriteFast(hor_cylinder_ext,HIGH);    //extend
      }
      else
      {
        digitalWriteFast(hor_cylinder_ext,LOW);     //Piston Stops extension
        hor_cylinder_pos=4;                      //Position Reached
      }
    }
    break;                                //break after case 4

  }
}


