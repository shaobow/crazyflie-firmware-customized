/*
 * 24-774 LQR 2nd try
 * 12/03/2022
 */

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_lqr.h"
#include "physicalConstants.h"

const static float g = 9.81;

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float err_x;
static float err_y;
static float err_z;
static float err_roll;
static float err_pitch;
static float err_yaw;

static int cmd_tick;

static float K_dlqr[4][12] = {
      { 1.2582, -1.7788,  4.7904, -0.1549,  0.0516,  0.0006,  0.3540, -0.5002,  1.3885, -0.0001,       0,       0},
      { 0.0033, -0.0154, -0.0176,  0.0730,  0.0051, -0.0053,  0.0040, -0.0112, -0.0140,  0.0089,       0,       0},
      { 0.0071, -0.0011, -0.0061,  0.0052,  0.0460,  0.0071,  0.0052, -0.0014, -0.0051,       0,  0.0091,       0},
      { 0.0452,  0.0197, -0.0121, -0.0249,  0.0322,  0.1083,  0.0311,  0.0103, -0.0117,       0,       0,  0.0138}
    };

const static float deg2millirad = (float) M_PI / 180.0f;
const static float cf_weight = CF_MASS * g;

void controllerLqrInit(void)
{
}

bool controllerLqrTest(void)
{
  return true;
}

void controllerLqr(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  control->controlMode = controlModeForceTorque;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    cmd_tick = 0;
    return;
  }

  // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
  float state_rateRoll = sensors->gyro.x * deg2millirad; // gyro unit: rad/sec
  float state_ratePitch = -sensors->gyro.y * deg2millirad;
  float state_rateYaw = sensors->gyro.z * deg2millirad;

  float u[4];

  float e1 = setpoint->position.x - state->position.x;
  float e2 = setpoint->position.y - state->position.y;
  float e3 = setpoint->position.z - state->position.z;

  float e4 = (setpoint->attitude.roll - state->attitude.roll) * deg2millirad; // unit: deg
  float e5 = (setpoint->attitude.pitch - state->attitude.pitch) * deg2millirad;
  float e6 = (setpoint->attitude.yaw - state->attitude.yaw) * deg2millirad;

  float e7 = setpoint->velocity.x - state->velocity.x;
  float e8 = setpoint->velocity.y - state->velocity.y;
  float e9 = setpoint->velocity.z - state->velocity.z;

  float e10 = setpoint->attitudeRate.roll * deg2millirad - state_rateRoll; // setpoint->attitudeRate unit: deg/sec
  float e11 = setpoint->attitudeRate.pitch * deg2millirad - state_ratePitch;
  float e12 = setpoint->attitudeRate.yaw * deg2millirad - state_rateYaw;
  float error[12] = {e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12};
  int i = 0, j = 0; 
  float res = 0;

  while (i < 4){
    while (j < 12){
      res += K_dlqr[i][j] * error[j];
      j++;
    }

    u[i] = res;
    i++;

    j = 0;
    res = 0;
  }

  /* feedback */
  control->thrustSi = u[0] + cf_weight;
  control->torqueX = u[1];
  control->torqueY = u[2];
  control->torqueZ = u[3];

  cmd_thrust = control->thrustSi;
  cmd_roll = control->torqueX;
  cmd_pitch = control->torqueY;
  cmd_yaw = control->torqueZ;

  err_x = e1;
  err_y = e2;
  err_z = e3;
  err_roll = e4;
  err_pitch = e5;
  err_yaw = e6;

  cmd_tick = 1;
}

/**
 * Logging variables for the command and reference signals for the
 * Mellinger controller
 */

LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)

LOG_ADD(LOG_FLOAT, err_x, &err_x)
LOG_ADD(LOG_FLOAT, err_y, &err_y)
LOG_ADD(LOG_FLOAT, err_z, &err_z)
LOG_ADD(LOG_FLOAT, err_roll, &err_roll)
LOG_ADD(LOG_FLOAT, err_pitch, &err_pitch)
LOG_ADD(LOG_FLOAT, err_yaw, &err_yaw)

LOG_ADD(LOG_INT8, cmd_tick, &cmd_tick)
LOG_GROUP_STOP(ctrlMel)

