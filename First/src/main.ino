#include <Arduino.h>
#include <defines.h>
#include <Common.h>
#include <Motor.h>
#include <MotorController.h>
#include <tsop.h>
#include <MotorController.h>
#include <I2C.h>
#include <Compass.h>
#include <lightSensor.h>
#include <lightSensorArray.h>

// 13 12 11 Front Left   Positive -true-> Negative    COSINE
// 10 9  8  Back Left    Negative -false-> Negative   SINE
// 7  6  5  Back Right   Positive -false-> Positive   COSINE
// 4  3  2  Front Right  Negative -true-> Positive    SINE

/*
           0
          _  _
        /  V  \
270   |       |   90
      \______/
         180
*/

MotorController Motor;
TSOP tsop;
Compass compass;
LightSensorArray lights;

const int GoalAcc = 7;
const int MoveSpd = 255;

unsigned long previousMillis = 0;
const long interval = 200;
bool voiding = false;
int oldLight = 0;

unsigned long compMillis = 0;
int previousHeading = 0;
const double kp = 4.5;
const double kd = 9;//-8;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  compass.compassSetup();
  compass.calibrate();
  Motor.Setup(1);
  tsop.Setup();
  lights.Setup();
  lights.GetVal();
  int robot = 2;
  if(robot==1){
    lights.SetThresh(35,100,70,140);
  } else if(robot==2){
    lights.SetThresh(20,70,60,90);
  } else{
    lights.SetThresh(999,999,999,999);
  }
  //int range = 20;
  //lights.SetThresh(lights.lightValues[0]+range,lights.lightValues[1]+range,lights.lightValues[2]+range,lights.lightValues[3]+range);
}

void loop(){
  compass.updateGyro();
  tsop.Read();
  tsop.FilterValues();
  tsop.GetAngle(3);
  tsop.GetStrength(3);
  lights.GetVal();
  int angle = tsop.angle;
  int strength = tsop.strength;
  int light = lights.LightAngle();
  unsigned long currentMillis = millis();


  int relativeHeading = compass.heading > 180 ? (360 - compass.heading) : -compass.heading;
  // int correctionRotation = relativeHeading * 3;

  double diffTime = ((double)(currentMillis - compMillis))/100.0;
  double difference = ((double)(relativeHeading - previousHeading)) / diffTime;
  compMillis = currentMillis;
  //Serial.print((previousHeading-relativeHeading));
  //Serial.print("\t");
  // Serial.println(difference);
  previousHeading = relativeHeading;

  int correction = round(kp*((double)relativeHeading) + kd*difference);


  //Motor.Move(0, correction , 0);
  // Serial.print(relativeHeading);
  // Serial.print("\t");
  // Serial.println(correction);

  // Serial.println(angle);
  //Serial.println(relativeHeading);

  if(!voiding){
    if (light == -30){
      //Not touching line
      if (angle == -30){
        //No ball around, compass correct
        Motor.Move(0,correction,0);
      } else{
        //Orbit Code
        if (strength<=90){
          //Too far away,  move towards Ball
          Motor.Move(angle,correction,MoveSpd);
        } else{
          //Close Orbit
          if (angle>=180){
            //Ball on left side
            if (angle<210){
              //Move Right
              Motor.Move(90,correction,MoveSpd);
            } else if (angle<280){
              //Back clear
              Motor.Move(185,correction,MoveSpd);
            } else if (angle<350){
              // Ball is at front left, now move left
              Motor.Move(270,correction,MoveSpd);
            } else{
              // Ball at front,
              Motor.Move(0,correction,MoveSpd);
            }
          } else if (angle<180){
            // Ball on right side
            if (angle>150){
              // Move left to make back clear
              Motor.Move(270,correction,MoveSpd);
            } else if (angle>60){
              // Back clear, move back
              Motor.Move(165,correction,MoveSpd);
            } else if (angle>20){
              // Ball is at front right, move right
              Motor.Move(90,correction,MoveSpd);
            } else{
              // Ball infront
              Motor.Move(0,correction,MoveSpd);
            }
          }
        }
      }
    } else {
      previousMillis = currentMillis;
      voiding = true;
      oldLight = light;
      Motor.Move(oldLight, correction, 255);
    }
    }else{
      Motor.Move(oldLight, correction, 255);
      if(currentMillis - previousMillis >= interval){
        voiding = false;
      }
    }
	}
