#include "RoboLib.h"
#include <digitalWriteFast.h>
#include "PS2X_lib.h"
#include "SabertoothSimplified.h"

//////////******************************Legacy Code******************************//////////
#define hall1 00
#define hall2 00
#define hall3 00 
#define hall4 00
#define pneu1 00
#define pneu2 00
#define pneu3 00
#define pneu4 00
int hallvalue1,hallvalue2,hallvalue3,hallvalue4;
int xval,yval;
//////////**********************************************************************//////////

//Saber Definitions
SabertoothSimplified ST1(Serial1);     // Use Serial3 as the serial port.
SabertoothSimplified ST2(Serial2);     // Use Serial2 as the serial port.

int x;  //Joystick X
int y;  //Joystick Y
int t;  //Turning L2,R2
boolean line;  //line present or absent

float x_1;
float y_1;
int deadz=40;
float V_x;  //Sideways Velocity
float V_y;  //Forward Velocity
float V_t;  //Turning velocity
float rot=0.2;  //Rotation Factor for turning while line following
//drive motors
float m1=0;
float m2=0;
float m3=0;
float m4=0;
float divisor;  //divisor  for normalizing

void setup()
{
  // initialize the serial communications:
  Serial.begin(9600);
    Serial1.begin(38400);
    Serial2.begin(38400);
  arduConfig();
  autobrake();
  PS2_setup();
     
//  pinMode(SER_Pin, OUTPUT);
//  pinMode(RCLK_Pin, OUTPUT);
//  pinMode(SRCLK_Pin, OUTPUT);
//  // Provide ground and power by using the analog inputs as normal
//  // digital pins.  This makes it possible to directly connect the
//  // breakout board to the Arduino.  If you use the normal 5V and
//  // GND pins on the Arduino, you can remove these lines.
//
//  clearRegisters();
//  writeRegisters();
}

void loop()
{
  ps2x.read_gamepad(false, vibrate);
//  if(ps2x.Button(PSB_PAD_LEFT))
//  {
//
//  }
  if(ps2x.Button(PSB_PAD_RIGHT))
  {  
   
//manbrake();
//while(1)
//trajectory();

    if(ps2x.Button(PSB_L1))//Mode Toggle Button
  {
    if(ps2x.ButtonPressed(PSB_PAD_UP))
    rot+=0.05;
    if(ps2x.ButtonPressed(PSB_PAD_DOWN))
    rot-=0.05;
    
    Serial.println(rot);
//    if(mode==4)
//      mode=1;
//    else
//      mode+=1; 
  }
  
//  if(mode==0)
//  {
//    
//    if(ps2x.Button(PSB_R1))
//    {
//      if(ps2x.ButtonPressed(PSB_PAD_UP))
//      {
//          if(swpull) digitalWrite(swing_pulling,HIGH);
//          else digitalWrite(swing_pulling,LOW);
//          swpull=!swpull;
//          Serial.println(swpull);
//      }
//      if(ps2x.ButtonPressed(PSB_PAD_DOWN))
//      {
//          if(swpull) digitalWrite(swing_pulling,HIGH);
//          else digitalWrite(swing_pulling,LOW);
//          swpull=!swpull;
//          Serial.println(swpull);
//      }
//    }
//  }
  
  if(mode==1)                                      //See-Saw
  {
    setModeLED(1);
    //See-Saw Mode Module
    if(ps2x.Button(PSB_R1))  //Mode Action ON
    {
    Serial.println("Mode 1");
      if(ps2x.Button(PSB_TRIANGLE))  //go to see saw placing position
      {
        hor_cylinder_target=1;
        Serial.println("HorT1");
      }
      if(ps2x.Button(PSB_CROSS))  //
      {
        hor_cylinder_target=2;
        Serial.println("HorT2");
      }
      if(ps2x.Button(PSB_CIRCLE))  //
      {
        hor_cylinder_target=3;
                Serial.println("HorT3");
      }
      if(ps2x.Button(PSB_SQUARE))  //
      {
        hor_cylinder_target=4;
                Serial.println("HorT4");
      }
      if(ps2x.ButtonPressed(PSB_PAD_UP))  //Swing Pulling Mechanism ON/OFF
      {
        if(swpull) digitalWrite(swing_pulling,HIGH);
        else digitalWrite(swing_pulling,LOW);
        swpull=!swpull;
        Serial.println(swpull);
      }
      if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //AutoBot Holding Magnet ON/OFF
      {
        if(holdauto) digitalWrite(autobot_holder,HIGH);
        else digitalWrite(autobot_holder,LOW);
        holdauto=!holdauto;
        Serial.println(holdauto);
      }
      if(ps2x.Button(PSB_PAD_LEFT))  //
      {
        sigBot=true;
        sigNo=1;        
      }
      if(ps2x.Button(PSB_PAD_RIGHT))  
      {
        sigBot=true;
        sigNo=5;
      }
    }
  }

  else if(mode==2)                                  //Swing
  {
    setModeLED(2);
    //Swing Mode Module
    if(ps2x.Button(PSB_R1))  //Mode Action ON
    {
    Serial.println("Mode 2");
      if(ps2x.Button(PSB_TRIANGLE))  //
      {
       ver_cylinder_target=1;
      }
      if(ps2x.Button(PSB_CROSS))  //
      {
       ver_cylinder_target=2;
      }
      if(ps2x.Button(PSB_CIRCLE))  //
      {
        ver_cylinder_target=3;
      }
      if(ps2x.Button(PSB_SQUARE))  //
      {
        ver_cylinder_target=4;
      }
      if(ps2x.ButtonPressed(PSB_PAD_UP))  //Swing Pulling Mechanism ON/OFF
      {
        if(swpull) digitalWrite(swing_pulling,HIGH);
        else digitalWrite(swing_pulling,LOW);
        swpull=!swpull;
        Serial.println(swpull);
      }
      if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //AutoBot Holding Magnet ON/OFF
      {
        if(holdauto) digitalWrite(autobot_holder,HIGH);
        else digitalWrite(autobot_holder,LOW);
        holdauto=!holdauto;
        Serial.println(holdauto);
      }
      if(ps2x.Button(PSB_PAD_LEFT))  //
      {
        sigBot=true;
        sigNo=2;
      }
      if(ps2x.Button(PSB_PAD_RIGHT))  
      {
        sigBot=true;
        sigNo=5;
      }
    }
  }

  else if(mode==3)                              //Pole Walk
  {
    setModeLED(3);
    //Pole Walk Mode Module
    if(ps2x.Button(PSB_R1))  //Mode Action ON
    {
    Serial.println("Mode 3");
      if(ps2x.Button(PSB_TRIANGLE))  //
      {
        hor_cylinder_target=1;
      }
      if(ps2x.Button(PSB_CROSS))  //
      {
        hor_cylinder_target=2;
      }
      if(ps2x.Button(PSB_CIRCLE))  //
      {
        hor_cylinder_target=3;
      }
      if(ps2x.Button(PSB_SQUARE))  //
      {
        hor_cylinder_target=4;
      }
      if(ps2x.ButtonPressed(PSB_PAD_UP))  //Swing Pulling Mechanism ON/OFF
      {
        if(swpull) digitalWrite(swing_pulling,HIGH);
        else digitalWrite(swing_pulling,LOW);
        swpull=!swpull;
        Serial.println(swpull);
      }
      if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //AutoBot Holding Magnet ON/OFF
      {
        if(holdauto) digitalWrite(autobot_holder,HIGH);
        else digitalWrite(autobot_holder,LOW);
        holdauto=!holdauto;
        Serial.println(holdauto);
      }
      if(ps2x.Button(PSB_PAD_LEFT))  //
      {
        sigBot=true;
        sigNo=3;
      }
      if(ps2x.Button(PSB_PAD_RIGHT))  //
      {
        sigBot=true;
        sigNo=5;
      }
    }
  }

  else if(mode==4)                                //Ladder
  {
    setModeLED(4);
    //Ladder Mode Module
    if(ps2x.Button(PSB_R1))  //Mode Action ON
    {
    Serial.println("Mode 4");
      if(ps2x.Button(PSB_TRIANGLE))  //
      {
        ver_cylinder_target=1;
      }
      if(ps2x.Button(PSB_CROSS))  //
      {
        ver_cylinder_target=2;
      }
      if(ps2x.Button(PSB_CIRCLE))  //
      {
        ver_cylinder_target=3;
      }
      if(ps2x.Button(PSB_SQUARE))  //
      {
        ver_cylinder_target=4;
      }
      if(ps2x.ButtonPressed(PSB_PAD_UP))  //Swing Pulling Mechanism ON/OFF
      {
        if(swpull) digitalWrite(swing_pulling,HIGH);
        else digitalWrite(swing_pulling,LOW);
        swpull=!swpull;
        Serial.println(swpull);
      }
      if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //AutoBot Holding Magnet ON/OFF
      {
        if(holdauto) digitalWrite(autobot_holder,HIGH);
        else digitalWrite(autobot_holder,LOW);
        holdauto=!holdauto;
        Serial.println(holdauto);
      }
      if(ps2x.Button(PSB_PAD_LEFT))  
      {
        sigBot=true;
        sigNo=4;
      }
      if(ps2x.Button(PSB_PAD_RIGHT))  //
      {
        sigBot=true;
        sigNo=5;
      }
    }
  }
  
if(sigBot) {
  Serial.println("Communicating...");
    signalChild(sigNo);
}

  if(ps2x.Button(PSB_R1)==0)  //Mode Action OFF
  {
    modeoff();  ////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  x=ps2x.Analog(PSS_RX);
  y=ps2x.Analog(PSS_LY);

//  if(ps2x.Button(PSB_R2))
//  if(ps2x.Button(PSB_PAD_LEFT)) {
//    t=-1;
//    Serial.println("CW");
//  }
//  else if(ps2x.Button(PSB_PAD_RIGHT)) {
//    t=1;
//    Serial.println("CCW");
//  }
//  else t=0;
//  
//  if(ps2x.Button(PSB_R2))
//  {
//    if(ps2x.Button(PSB_PAD_UP)) {
//      Serial.println("AutoCW");
//      autocw();
//    }
//    else if(ps2x.Button(PSB_PAD_DOWN)) {
//      Serial.println("AutoCCW");
//      autoccw();
//    }
//    else if(ps2x.Button(PSB_CROSS)) {
//      Serial.println("AutoBrake");
//    autobrake();
//    }
//
//  }
//  
//  if(ps2x.Button(PSB_R2))
//  {
//    if(ps2x.Button(PSB_SQUARE)) {
//      Serial.println("ManBrake");
//      manbrake();
//    }
//  }
  if(ps2x.Button(PSB_R2))
  t=1;
  else if(ps2x.Button(PSB_L2))
  t=-1;
  else t=0;
  
  if(ps2x.ButtonPressed(PSB_SELECT))
  {
    pwmAdjust=!pwmAdjust;
    Serial.print("Mode Changed: ");
    Serial.println(pwmAdjust);
  }

//  if(ps2x.ButtonPressed(PSB_START))
//  {
//    override=!override;
//    if(override)
//    Serial.print("OverrideOverrideOverrideOverride");
//  }


  ///////////////////function calls////////////////////

  if(ver_cylinder_target!=ver_cylinder_pos)
  {
    ver_cylinder_goto(ver_cylinder_target);
  }
  if(hor_cylinder_target!=hor_cylinder_pos)
  {
    hor_cylinder_goto(hor_cylinder_target);
  }

  holo_drive( x , y , t , line);
//else {  //override false
//    if(ps2x.Button(PSB_PAD_LEFT))  //Line following left
//    {
//      PIDControl(1);
//    }
//    if(ps2x.Button(PSB_PAD_RIGHT))  //Line following right
//    {
//      PIDControl(-1);
//    }
//
//}
/////////////////////////////////Enable Serial output to Debug the program//////////////////////

//  Serial.print(" Carriage Status= ");
//  Serial.print(ver_cylinder_status);
//  Serial.print(" Carriage Position= ");
//  Serial.print(ver_cylinder_pos);
//  Serial.print(" Carriage Target= ");
//  Serial.print(ver_cylinder_target);
//  Serial.print(" Slider Status= ");
//  Serial.print(hor_cylinder_status);
//  Serial.print(" Slider Position= ");
//  Serial.print(hor_cylinder_pos);
//  Serial.print(" Slider Target= ");
//  Serial.print(hor_cylinder_target);
//  Serial.print(" Holding Motor Status= ");
//  Serial.print(holding_motor_status);

}

//////////////////holonomic drive function if line is detected, drive changes according to the mode///////////////////

void holo_drive(int x, int y, int t,boolean line)
{

  if(x>=0 && x<(128-deadz/2))
  {
    x_1=(x-(127.5-deadz/2))/(127.5-deadz/2);
  }
  else if(x>128+deadz/2 && x<=256)
  {
    x_1=(x-(127.5+deadz/2))/(127.5-deadz/2);
  }
  else
  {
    x_1 = 0;
  }

  if(y >= 0 && y <(128-deadz/2))
  {
    y_1=-((y-(127.5-deadz/2)) / (127.5-deadz/2));
  }
  else if(y > 128+deadz/2 && y<=256)
  {
    y_1=-((y -(127.5+deadz/2)) / (127.5-deadz/2));
  }
  else
  {
    y_1 = 0;
  }

  if(x_1>=0)  
    V_x = (pow(11,x_1)-1);
  else
    V_x = -pow(11,-x_1)+1;

  if(y_1>=0)
    V_y = pow(11,y_1)-1;
  else
    V_y = -pow(11,-y_1)+1;

  V_t = 5*t;

  if(ps2x.Button(PSB_PAD_LEFT))  //Change the drive if curve selected
  {
   //if(mode != 2 || mode !=0)
   V_x=(abs(x_1)/x_1)*10;
      V_t= -V_x*rot;  //rot is the multiplication factor which decides difference in absolute values of left and right wheels  
  }
//m1(Serial1,2);m2(Serial1,1);m3(Serial2,2),m4(Serial2,1)
  m1 = (V_x + V_y - V_t)*12.7;  //Subtract |V_t| from absolute value of linear velocity
  m2 = (V_x - V_y - V_t)*12.7;
  m3 = (-V_x - V_y - V_t)*12.7; //Add |V_t| to absolute value of linear velocity
  m4 = (-V_x + V_y - V_t)*12.7; 

  if(abs(m1)>127 || abs(m2)>127 || abs(m3)>127 || abs(m4)>127)  //Normalizing the values to counter the overflow
  {
    divisor= max(abs(m1),abs(m2));
    divisor= max(abs(divisor),abs(m3));
    divisor= max(abs(divisor),abs(m4));
    //    divisor= divisor;
    m1= m1*127/divisor;
    m2= m2*127/divisor;
    m3= m3*127/divisor;
    m4= m4*127/divisor;
  }
//Normalizing becuz speed too damn HIGH!
if(pwmAdjust){
m1=m1/4;
m2=m2/4;
m3=m3/4;
m4=m4/4;

}
else{
m1=m1/2;
m2=m2/2;
m3=m3/2;
m4=m4/2;
}
  ST2.motor(2,(int)-m3);
  ST2.motor(1,(int)-m4);
  ST1.motor(1,(int)-m2);
  ST1.motor(2,(int)m1);

//Serial.print("Divisor ");
//   Serial.print(divisor);
//   Serial.print(" x_1= ");
//   Serial.print(x_1);
//   Serial.print(" y_1= ");
//   Serial.print(y_1);
//   Serial.print(" vx= ");
//   Serial.print(V_x);
//   Serial.print(" vy = ");      
//   Serial.print(V_y);
//   Serial.print(" pwm1= ");
//   Serial.print((int)m1);
//   Serial.print(" pwm2= ");
//   Serial.print((int)m2);
//   Serial.print(" pwm3= ");
//   Serial.print((int)m3);
//   Serial.print(" pwm4= ");
//   Serial.println((int)m4);

}

//void manual_ctrl()
//{
//  Serial.println("Manual Ctrl Mode");
//  while(1)
//  {
//    ps2x.read_gamepad(false, vibrate);
//  Serial.println("Loop");
//  if(ps2x.Button(PSB_R2))
//  {
//  if(ps2x.Button(PSB_L1)) break;
//  }
//  
//  }
//}

void autocw()
{
  digitalWrite(holdir,HIGH);
  analogWrite(pwm,60);
}

void autoccw()
{
  digitalWrite(holdir,LOW);
  analogWrite(pwm,60);
}

void autobrake()
{
  analogWrite(pwm,0);
}

void manbrake()
{
  ST1.motor(1,0);
  ST1.motor(2,0);
  ST2.motor(1,0);
  ST2.motor(2,0);
}

void modeoff()
{
      if(ps2x.ButtonPressed(PSB_TRIANGLE))  //Carriage UP
    {
      digitalWriteFast(carriage_clamp_1,LOW);          //Release clamping magnets
      digitalWriteFast(carriage_clamp_2,LOW);          //Release clamping magnets
      digitalWriteFast(ver_cylinder_ext,HIGH);        //Enable pneumatic extension
      ver_cylinder_status=1;
    }
    if(ps2x.ButtonReleased(PSB_TRIANGLE))  //Carriage Stops
    {
      digitalWriteFast(carriage_clamp_1,HIGH);          //Engage clamping magnets
      digitalWriteFast(carriage_clamp_2,HIGH);          //Engage clamping magnets
      digitalWriteFast(ver_cylinder_ext,LOW);          //Disable pneumatic extension
      ver_cylinder_status=0;
    }
    if(ps2x.ButtonPressed(PSB_CROSS))  //Carriage Down
    {
      digitalWriteFast(carriage_clamp_1,LOW);          //Release clamping magnets
      digitalWriteFast(carriage_clamp_2,LOW);          //Release clamping magnets
      digitalWriteFast(ver_cylinder_ret,HIGH);        //Enable pneumatic return
      ver_cylinder_status=-1;
    }
    if(ps2x.ButtonReleased(PSB_CROSS))  //Carriage Stops
    {
      digitalWriteFast(carriage_clamp_1,HIGH);          //Engage clamping magnets
      digitalWriteFast(carriage_clamp_2,HIGH);          //Engage clamping magnets
      digitalWriteFast(ver_cylinder_ret,LOW);          //Enable pneumatic return
      ver_cylinder_status=0;
    }
    if(ps2x.ButtonPressed(PSB_CIRCLE))  //Slider Forward
    {
      digitalWriteFast(hor_cylinder_ext,HIGH);
      hor_cylinder_status=1;
    }
    if(ps2x.ButtonReleased(PSB_CIRCLE))  //Slider Stops
    {
      digitalWriteFast(hor_cylinder_ext,LOW);
      hor_cylinder_status=0;
    }
    if(ps2x.ButtonPressed(PSB_SQUARE))  //Slider Back
    {
      digitalWriteFast(hor_cylinder_ret,HIGH);
      hor_cylinder_status=-1;
    }
    if(ps2x.ButtonReleased(PSB_SQUARE))  //Slider Stops
    {
      digitalWriteFast(hor_cylinder_ret,LOW);
      hor_cylinder_status=0;
    }
    if(ps2x.ButtonPressed(PSB_PAD_UP))  //Arm cylinder Extends
    {
      digitalWriteFast(arm_cylinder_ext,HIGH);
      arm_cylinder_status=1;
    }
    if(ps2x.ButtonReleased(PSB_PAD_UP))  //Arm cylinder stops
    {
      digitalWriteFast(arm_cylinder_ext,LOW);
      arm_cylinder_status=0;
    }
    if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //Arm cylinder Retracts
    {
      digitalWriteFast(arm_cylinder_ret,HIGH);
      arm_cylinder_status=-1;
    }
    if(ps2x.ButtonReleased(PSB_PAD_DOWN))  //Arm cylinder stops
    {
      digitalWriteFast(arm_cylinder_ret,LOW);
      arm_cylinder_status=0;
    }

}

void line_poll()
{
  line_val1 = analogRead(line_1);
  line_val2 = analogRead(line_2);
  line_val3 = analogRead(line_3);
  line_val4 = analogRead(line_4);
  line_val5 = analogRead(line_5);
  line_val6 = analogRead(line_6);
  
  
//  line_val7 = analogRead(line_7);
//  line_val8 = analogRead(line_8);
//  line_val9 = analogRead(line_9);
//  line_val10 = analogRead(line10);
//  line_val11 = analogRead(line_11);
//  line_val112 = analogRead(line_12);
  
  
  
  
  //Serial.print("Line_0 = ");
  //Serial.print(line_val0);
  
  //Serial.print(" Line_1 = ");
  //Serial.print(line_val1);
    
//  Serial.print(" Line_1 = ");
//  Serial.print(line_val1);
//
//  Serial.print(" Line_2 = ");
//  Serial.print(line_val2);
//  
//  Serial.print(" Line_3 = ");
//  Serial.print(line_val3);
//  
//  Serial.print(" Line_4 = ");
//  Serial.print(line_val4);
//  
//  Serial.print(" Line_5 = ");
//  Serial.print(line_val5);
//  
//  Serial.print(" Line_6 = ");
//  Serial.println(line_val6);

}

void line_action()
{
  if(line_val3>=th1 && line_val3<=th2)
  {
    if(ps2x.Button(PSB_START)) override=true;
    else {
      override=false;
      ST1.motor(1,0);  //1
      ST1.motor(2,0);  //2
      ST2.motor(1,0);  //3
      ST2.motor(2,0);  //4
      }
  }
}

void PIDControl(int ctrldir)
{
  int error=0; 
  double pwm0=m1,pwm1=m2;
//  unsigned long now = millis();
//  double timeChange = (double)(now - lastTime);
  if(line_val1>=th1 && line_val1<=th2) error=2;
  else if(line_val2>=th1 && line_val2<=th2) error=1;
  if(line_val6>=th1 && line_val6<=th2) error=-2;
  else if(line_val2>=th1 && line_val2<=th2) error=-1;

      ITerm+= (ki * error);
      if(ITerm > pwmMax) ITerm= pwmMax;
      else if(ITerm < pwmMin) ITerm= pwmMin;
      
      pwm0+= kp*error + ITerm + kd*(error - errorPrevious);
      pwm1-= kp*error + ITerm + kd*(error - errorPrevious);
      if(pwm0> pwmMax) pwm0 = pwmMax;
      else if(pwm0 < pwmMin) pwm0 = pwmMin;
      if(pwm1> pwmMax) pwm1 = pwmMax;
      else if(pwm1 < pwmMin) pwm1 = pwmMin;
      
      errorPrevious = error;
      
//    ST1.motor(1,(int)pwm0);  //1
//    ST1.motor(2,(int)pwm0);  //2
//    ST2.motor(1,(int)pwm1);  //3
//    ST2.motor(2,(int)pwm1);  //4
}
















//void xleft()
//{
//    ST1.motor(1,31);  //1
//    ST1.motor(2,31);  //2
//    ST2.motor(1,-31);  //3
//    ST2.motor(2,-31);  //4
//}
//
//void xright()
//{
//    ST1.motor(1,-31);  //1
//    ST1.motor(2,-31);  //2
//    ST2.motor(1,31);  //3
//    ST2.motor(2,31);  //4
//}
//
//void trajectory()
//{
//  int s1=10;
//  static int s2=10;
////  ps2x.read_gamepad(false, vibrate);
////  if(ps2x.Button(PSB_TRIANGLE))
////  {
//  ST1.motor(1,-s2);
//  ST1.motor(2,s2);
//  ST2.motor(1,s1);
//  ST2.motor(2,s1);
//  delay(250);
//  Serial.println("trajectory");
//  s2++;
////  }
//}
}
