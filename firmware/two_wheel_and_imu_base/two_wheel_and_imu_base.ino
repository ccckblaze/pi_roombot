/*

 Copyright (c) 2013-2015, Tony Baltovski 
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
 
 */

#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_base/UpdateGains.h>
#include <ros_arduino_msgs/Encoders.h>
#include <ros_arduino_msgs/CmdDiffVel.h>
#include <ros_arduino_msgs/RawImu.h>
#include <pi_roombot/UpdatePin.h>
#include <geometry_msgs/Vector3.h>

/********************************************************************************************
/                                                     USER CONFIG                           *
/********************************************************************************************/

// Select your baud rate here
#define BAUD 115200

// Select your motor driver here
//#define PololuMC33926
#define DFRobotL298PShield

// Define your encoder pins here.
// Try to use pins that have interrupts
// Left side encoders pins
#define LEFT_ENCODER_A 14  // Interrupt on Teensy 3.0
#define LEFT_ENCODER_B 15  // Interrupt on Teensy 3.0
// Right side encoders pins
#define RIGHT_ENCODER_A 6  // Interrupt on Teensy 3.0
#define RIGHT_ENCODER_B 7  // Interrupt on Teensy 3.0

/********************************************************************************************
/                                                 END OF USER CONFIG                        *
/********************************************************************************************/

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#if defined(PololuMC33926)
  #include <PololuMC33926.h>
#endif

#include "motor_driver_config.h"

// IMU
#if defined(WIRE_T3)
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

#include "imu_configuration.h"

typedef struct {
  float desired_velocity;     // [m/s]
  uint32_t current_time;      // [milliseconds]
  uint32_t previous_time;     // [milliseconds]
  int32_t current_encoder;    // [counts]
  int32_t previous_encoder;   // [counts]
  float previous_error;       // 
  float total_error;          // 
  int16_t command;            // [PWM]
}
ControlData;

// Encoder objects from PJRC encoder library.
Encoder left_encoder(LEFT_ENCODER_A,LEFT_ENCODER_B);
Encoder right_encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B);

// Vehicle characteristics
float counts_per_rev[1];
float gear_ratio[1];
int encoder_on_motor_shaft[1];
float wheel_radius[1];         // [m]
float meters_per_counts;       // [m/counts]
int pwm_range[1];

// Gains;
float pid_gains[3];
float Kp, Ki, Kd;

// Structures containing PID data
ControlData left_motor_controller;
ControlData right_motor_controller;

// Control methods prototypes
void updateControl(ControlData * ctrl, int32_t encoder_reading);
void doControl(ControlData * ctrl);
void Control();

int control_rate[1];   // [Hz]
int encoder_rate[1];   // [Hz]
int imu_rate[1];       // [Hz]
int no_cmd_timeout[1]; // [seconds]

bool is_first = true;

uint32_t up_time;             // [milliseconds]
uint32_t last_encoders_time;  // [milliseconds]
uint32_t last_cmd_time;       // [milliseconds]
uint32_t last_control_time;   // [milliseconds]
uint32_t last_imu_time;       // [milliseconds]
uint32_t last_status_time;    // [milliseconds]

char frame_id[] = "base_link";

// ROS node
ros::NodeHandle nh;

// ROS subribers/service callbacks prototye
void cmdDiffVelCallback(const ros_arduino_msgs::CmdDiffVel& diff_vel_msg); 

// ROS subsribers
ros::Subscriber<ros_arduino_msgs::CmdDiffVel> sub_diff_vel("cmd_diff_vel", cmdDiffVelCallback);

// ROS services prototype
void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res);
void updatePinCb(const pi_roombot::UpdatePin::Request &req, pi_roombot::UpdatePin::Response &res);

// ROS services
ros::ServiceServer<ros_arduino_base::UpdateGains::Request, ros_arduino_base::UpdateGains::Response> update_gains_server("update_gains", &updateGainsCb);
ros::ServiceServer<pi_roombot::UpdatePin::Request, pi_roombot::UpdatePin::Response> update_pin_server("update_pin", &updatePinCb);

// ROS publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
ros_arduino_msgs::RawImu raw_imu_msg;

// ROS publishers
ros::Publisher pub_encoders("encoders", &encoders_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

void setup() 
{ 
  // Set the node handle
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  encoders_msg.header.frame_id = frame_id;
  // Pub/Sub
  nh.advertise(pub_encoders);
  nh.advertise(raw_imu_pub);
  nh.subscribe(sub_diff_vel);
  nh.advertiseService(update_gains_server);
  nh.advertiseService(update_pin_server);
  
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller.");

  if (!nh.getParam("control_rate", control_rate,1))
  {
    control_rate[0] = 50;
  }
  if (!nh.getParam("encoder_rate", encoder_rate,1))
  {
    encoder_rate[0] = 50;
  }
  if (!nh.getParam("imu_rate", imu_rate,1))
  {
    imu_rate[0] = 50;
  }
  if (!nh.getParam("no_cmd_timeout", no_cmd_timeout,1))
  {
    no_cmd_timeout[0] = 1;
  }
  if (!nh.getParam("pid_gains", pid_gains,3))
  { 
    pid_gains[0] = 150;  // Kp
    pid_gains[1] =   0;  // Ki
    pid_gains[2] =  20;  // Kd
  }

  if (!nh.getParam("counts_per_rev", counts_per_rev,1))
  {
    counts_per_rev[0] = 48.0;
  }
  if (!nh.getParam("gear_ratio", gear_ratio,1))
  {
    gear_ratio[0] = 75.0/1.0;
  }
  if (!nh.getParam("encoder_on_motor_shaft", encoder_on_motor_shaft,1))
  {
    encoder_on_motor_shaft[0] = 1;
  }
  if (!nh.getParam("wheel_radius", wheel_radius,1))
  {
    wheel_radius[0] = 0.120/2.0;
  }
  if (!nh.getParam("pwm_range", pwm_range,1))
  {
    pwm_range[0] = 255;
  }

  // Compute the meters per count
  if (encoder_on_motor_shaft[0] == 1)
  {
    meters_per_counts = ((PI * 2 * wheel_radius[0]) / (counts_per_rev[0] * gear_ratio[0]));
  }
  else
  {
    meters_per_counts = ((PI * 2 * wheel_radius[0]) / counts_per_rev[0]);
  }
  // Create PID gains for this specific control rate
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
  
  // Initialize the motors
  setupMotors();

  // Start Wire
  #if defined(WIRE_T3)
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  #else
    Wire.begin();
  #endif
} 

void loop() 
{
  // For motors
  if ((millis() - last_encoders_time) >= (1000 / encoder_rate[0]))
  { 
    encoders_msg.left = left_encoder.read();
    encoders_msg.right = right_encoder.read();
    encoders_msg.header.stamp = nh.now();
    pub_encoders.publish(&encoders_msg);
    last_encoders_time = millis();
  }
  if ((millis()) - last_control_time >= (1000 / control_rate[0]))
  {
    Control();
    last_control_time = millis();
  }

  // Stop motors after a period of no commands
  if((millis() - last_cmd_time) >= (no_cmd_timeout[0] * 1000))
  {
    left_motor_controller.desired_velocity = 0.0;
    right_motor_controller.desired_velocity = 0.0;
  }
  // For IMU
  if (is_first)
  { 
    raw_imu_msg.accelerometer = check_accelerometer();
    raw_imu_msg.gyroscope = check_gyroscope();
    raw_imu_msg.magnetometer = check_magnetometer();
      
    if (!raw_imu_msg.accelerometer)
    {
      nh.logerror("Accelerometer NOT FOUND!");
    }
      
    if (!raw_imu_msg.gyroscope)
    {
      nh.logerror("Gyroscope NOT FOUND!");
    }
      
    if (!raw_imu_msg.magnetometer)
    {
      nh.logerror("Magnetometer NOT FOUND!");
    }
      
    is_first = false;
  }
  else if (millis() - last_imu_time >= 1000 / imu_rate[0])
  {
    raw_imu_msg.header.stamp = nh.now();
    raw_imu_msg.header.frame_id = "imu_link";
    if (raw_imu_msg.accelerometer)
    {
      measure_acceleration();
      raw_imu_msg.raw_linear_acceleration = raw_acceleration;
    }

    if (raw_imu_msg.gyroscope)
    {
      measure_gyroscope();
      raw_imu_msg.raw_angular_velocity = raw_rotation;
    }
      
    if (raw_imu_msg.magnetometer)
    {
      measure_magnetometer();
      raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
    }

    raw_imu_pub.publish(&raw_imu_msg);

    last_imu_time = millis();
  }

  nh.spinOnce();
}

void cmdDiffVelCallback( const ros_arduino_msgs::CmdDiffVel& diff_vel_msg) 
{
  left_motor_controller.desired_velocity = diff_vel_msg.left;
  right_motor_controller.desired_velocity = diff_vel_msg.right;
  last_cmd_time = millis();
}

void updateControl(ControlData * ctrl, int32_t encoder_reading)
{
  ctrl->current_encoder = encoder_reading;
  ctrl->current_time = millis();;
}

void doControl(ControlData * ctrl)
{
  float estimated_velocity = meters_per_counts * (ctrl->current_encoder - ctrl->previous_encoder) * 1000.0 / (ctrl->current_time - ctrl->previous_time);
  float error = ctrl->desired_velocity - estimated_velocity;
  float cmd = Kp * error + Ki * (error + ctrl->total_error) + Kd * (error - ctrl->previous_error);

  cmd += ctrl->command;

  if(cmd >= pwm_range[0])
  {
    cmd = pwm_range[0];
  }
  else if (cmd <= -pwm_range[0])
  {
    cmd = -pwm_range[0];
  }
  else
  {
    ctrl->total_error += error;
  }

  ctrl->command = cmd;
  ctrl->previous_time = ctrl->current_time;
  ctrl->previous_encoder = ctrl->current_encoder;
  ctrl->previous_error = error;
}

void Control()
{
  updateControl(&left_motor_controller, left_encoder.read());
  updateControl(&right_motor_controller, right_encoder.read());

  doControl(&left_motor_controller);
  doControl(&right_motor_controller);

  commandLeftMotor(left_motor_controller.command);
  commandRightMotor(right_motor_controller.command);
}

void updateGainsCb(const ros_arduino_base::UpdateGains::Request & req, ros_arduino_base::UpdateGains::Response & res)
{
  for ( int x = 0; x < 3; x++)
  {
    pid_gains[x] = req.gains[x];
  }
  
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
  
  nh.loginfo("Gains Updated!");
}

void updatePinCb(const pi_roombot::UpdatePin::Request & req, pi_roombot::UpdatePin::Response & res)
{
  if(req.mode == OUTPUT) {
    pinMode(req.pin, req.mode);
    digitalWrite(req.pin, req.state);
    res.result = 0;
  }
  else if(req.mode == INPUT) {
    pinMode(req.pin, req.mode);
    res.result = digitalRead(req.pin);
  }
  else {
    res.result = -1;
  }
  
  nh.loginfo("Pin Updated!");
}
