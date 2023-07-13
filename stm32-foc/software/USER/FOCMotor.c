/************************************************
本文件主要来自CSDN博主loop222对SimpleFOC项目的移植
原教程：https://blog.csdn.net/loop222/article/details/120471390
************************************************/

#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "MagneticSensor.h" 

/******************************************************************************/
float shaft_angle;//!< current motor angle
float electrical_angle;
float shaft_velocity;
float current_sp;
float shaft_velocity_sp;
float shaft_angle_sp;
DQVoltage_s voltage;
DQCurrent_s current;

TorqueControlType torque_controller;
MotionControlType controller;

float sensor_offset=0;
float zero_electric_angle;
/******************************************************************************/
// shaft angle calculation
float shaftAngle(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return sensor_direction*getAngle() - sensor_offset;
}
/******************************************************************************/
float electricalAngle(void)
{
  return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}
/******************************************************************************/


