#include <Arduino.h>

#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include <QTRSensors.h>

L3G gyro;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA; 
Zumo32U4Buzzer buzzer;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

uint64_t encoder_glob_a = 0;
uint64_t encoder_glob_b = 0;

int16_t turnSpeed = 125;
int16_t speed_forward = 100;
int16_t speed_forward_slave = 100;
int16_t calSpeed =   100;

void setup() 
{
  Serial1.begin(230400);
  Serial.begin(230400);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  lineSensors.initFiveSensors();
  calibrateLineSensors();
}

void calibrateLineSensors()
{ 
    buttonA.waitForButton();
    motors.setSpeeds(-85,-85);

    for (uint16_t i = 0; i < 100; i++)
    {
        if (i == 50)
        {
            motors.setSpeeds(85,85);

        }
        lineSensors.calibrate(QTR_EMITTERS_ON);
        delay(20);
    }

    motors.setSpeeds(0,0);
    buzzer.playFrequency(440, 200, 15); 
    buttonA.waitForButton();
}

// Turn left
void turnLeft(int turn_agle) 
{
  move_back();
  turnSensorReset();
  motors.setSpeeds(turnSpeed, -turnSpeed);
  int angle = 0;
  do {
    delay(1);
    turnSensorUpdate();
    angle = get_angle();
  } while (angle != turn_agle);
  motors.setSpeeds(0, 0);
  delay(100);
}

// Turn right
void turnRight(int turn_agle) 
{
    move_back();
    turnSensorReset();
    motors.setSpeeds(-turnSpeed, turnSpeed);
    int angle = 0;
    do {
        delay(1);
        turnSensorUpdate();
        angle = get_angle();
    } while (angle != -turn_agle);
    motors.setSpeeds(0, 0);
    delay(100);
}

int get_angle ()
{
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

void alight_left_or_right(char* type)
{
    //move_back();
    int countsLeft = encoders.getCountsAndResetLeft();
    int countsRight = encoders.getCountsAndResetRight();
    if (type == "left")
    {
        do
        {
            countsLeft = encoders.getCountsLeft();
            countsRight = encoders.getCountsRight();
            motors.setSpeeds(calSpeed, -calSpeed);
        }
        while (countsLeft > -15 && countsRight < 15);
    }
    else if (type == "right")
    {
        do
        {
          countsLeft = encoders.getCountsLeft();
          countsRight = encoders.getCountsRight();
          motors.setSpeeds(-calSpeed, calSpeed);
        }
        while (countsLeft < 15 && countsRight > -15);
    }
    motors.setSpeeds(0, 0);
    delay(100);
}

void move_back()
{
    int countsLeft = encoders.getCountsAndResetLeft();
    int countsRight = encoders.getCountsAndResetRight();
    
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
    
    motors.setSpeeds(0, 0);
    delay(150);
    do 
    {
        countsLeft = encoders.getCountsLeft();
        countsRight = encoders.getCountsRight();
        motors.setSpeeds(100, 100);
    }
    while (countsLeft > -200 && countsRight > -200);

    motors.setSpeeds(0, 0);
    delay(150);
}

void move_forward()
{
  encoder_glob_a += encoders.getCountsAndResetLeft();
  encoder_glob_b += encoders.getCountsAndResetRight();
  if(encoder_glob_a > encoder_glob_b)
  {
    speed_forward_slave ++;
  } else if (encoder_glob_a < encoder_glob_b)
  {
    speed_forward_slave --;
  }
  motors.setSpeeds(-speed_forward, -speed_forward_slave);
  delay(2);
}

void reset_encoders()
{
   encoder_glob_a = encoders.getCountsAndResetLeft();
   encoder_glob_b = encoders.getCountsAndResetRight();
   encoder_glob_a = 0;
   encoder_glob_b = 0;
   speed_forward_slave = 100 ;
}

bool sensors_on_black() 
{
    return (lineSensorValues[0] >= 500 
        && lineSensorValues[2] >= 500 
        && lineSensorValues[4] >= 500);
}

bool sensors_on_white()
{
    return (lineSensorValues[0] < 500 && lineSensorValues[4] < 500);
}

bool while_on_line ()
{
    return (lineSensorValues[0]  > 100 && lineSensorValues[2] > 100 && lineSensorValues[4] > 100);
}

void forward()
{
   reset_encoders();
   do 
   {
        lineSensors.readCalibrated(lineSensorValues, QTR_EMITTERS_ON);
        //when 3 sensors reach the line 
        if (sensors_on_white())
        {
            move_forward();
        }
        else if (sensors_on_black())
        {
            //go trought the black line
            while ( while_on_line() )
            {
                  move_forward();
                  lineSensors.readCalibrated(lineSensorValues, QTR_EMITTERS_ON);
            }
            motors.setSpeeds(0, 0);
            break;
        }
        //if there is a white serface
        else if (lineSensorValues[0] >= 500 && lineSensorValues[4] < 500)
        {
            alight_left_or_right("left");
            reset_encoders();
        }
        else if (lineSensorValues[0] < 500 && lineSensorValues[4] >= 500)
        {    
            alight_left_or_right("right");
            reset_encoders();
        }           
   }while (1);
}

void loop() {
    lineSensors.readCalibrated(lineSensorValues, QTR_EMITTERS_ON);
     if (Serial1.available())
     {
        char command = Serial1.read();
        switch (command)
        {
            case 'f':
                forward();
                break;
            case 'r':
                turnRight(90);
                break;
            case 'l':
                turnLeft(90);
                break;
            case 'd':
                turnRight(180);
                break;
        }
     }  
}
