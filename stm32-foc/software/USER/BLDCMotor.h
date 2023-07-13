#ifndef BLDCMotor_H
#define BLDCMotor_H

/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

/******************************************************************************/
extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void setTargetVotage(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/******************************************************************************/

#endif
