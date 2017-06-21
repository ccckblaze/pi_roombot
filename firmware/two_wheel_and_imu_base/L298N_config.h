#ifndef _L298N_CONFIG_H_
#define _L298N_CONFIG_H_

#include "motor_driver_config.h"

// Motor Driver Pins
//Left Motor
#define left_motor_enable 4
#define left_motor_in1 5
#define left_motor_in2 6
//Right Motor
#define right_motor_enable 7
#define right_motor_in1 8
#define right_motor_in2 9
// Left and Right motor driver objects

void setupMotors()
{

}
void commandLeftMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(left_motor_in1, 1);
    digitalWrite(left_motor_in2, 0);
  }
  else
  {
    digitalWrite(left_motor_in1, 0);
    digitalWrite(left_motor_in2, 1);
  }
  analogWrite(left_motor_enable, abs(cmd));
}
void commandRightMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(right_motor_in1, 1);
    digitalWrite(right_motor_in2, 0);
  }
  else
  {
    digitalWrite(right_motor_in1, 0);
    digitalWrite(right_motor_in2, 1);
  }
  analogWrite(right_motor_enable, abs(cmd));
}

#endif  // _L298N_CONFIG_H_

