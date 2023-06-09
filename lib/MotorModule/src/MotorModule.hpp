#ifndef MOTOR_MODULE_HPP
#define MOTOR_MODULE_HPP

#include "../../MathOps/src/MathOps.hpp"
#include <mcp2515.h>

/// Value Limits ///
#define P_MIN -95.5f        // Radians
 #define P_MAX 95.5f        
 #define V_MIN -45.0f       // Rad/s
 #define V_MAX 45.0f
 #define KP_MIN 0.0f        // N-m/rad
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f        // N-m/rad/s
 #define KD_MAX 5.0f
 #define I_MIN -18.0f
 #define I_MAX 18.0f
 
 
/// Structs for organizing commands and data ///
typedef struct
{
    int id;
    float position, velocity, current;
} StateStruct;

    
typedef struct
{
    int id;
    float i_ff, p_des, kp, v_des, kd;
}ControlStruct;

typedef struct
{
    StateStruct state;
    ControlStruct control;
    byte rxMsg[6];
    byte txMsg[8];
}MotorStruct;

void pack_cmd(MotorStruct * motor);
void unpack_reply(MotorStruct * motor);
void enable_motor(MotorStruct * motor, MCP2515 can);
void disable_motor(MotorStruct * motor, MCP2515 can);


#endif // MOTOR_MODULE_HPP