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

# define NUM_STATE 12
# define NUM_CTRL 4

static float K_dlqr_0_0[NUM_CTRL][NUM_STATE] = {
    {-0.0000,    0.0000,    6.3246,   -0.0000,   -0.0000,    0.0000,   -0.0000,    0.0000,    2.3112,   -0.0000,   -0.0000,    0.0000},
    { 0.0000,   -3.8730,    0.0000,   14.6873,    0.0000,    0.0000,    0.0000,   -3.5492,    0.0000,    2.2362,    0.0000,    0.0000},
    { 3.8730,    0.0000,    0.0000,    0.0000,   14.6873,    0.0000,    3.5492,   -0.0000,    0.0000,    0.0000,    2.2362,    0.0000},
    { 0.0000,    0.0000,   -0.0000,   -0.0000,    0.0000,    7.0711,    0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    1.0002},
};

static float K_dlqr_10_0[NUM_CTRL][NUM_STATE] = {
    {-0.0000,   -0.3913,    6.2922,    0.0056,   -0.0000,    0.0000,   -0.0000,   -0.1204,    2.2982,    0.0000,   -0.0000,    0.0000},
    { 0.0000,   -3.8532,   -0.6389,   14.7752,    0.0000,    0.0000,    0.0000,   -3.4995,   -0.6170,    2.2362,    0.0000,    0.0000},
    { 3.7445,   -0.0000,   -0.0000,    0.0000,   14.3896,    0.6976,    3.4074,   -0.0000,   -0.0000,    0.0000,    2.2362,    0.0000},
    { 0.9893,   -0.0000,   -0.0000,    0.0000,    1.5597,    7.2379,    0.8303,   -0.0000,   -0.0000,    0.0000,    0.0000,    1.0002},
};

static float K_dlqr_0_10[NUM_CTRL][NUM_STATE] = {
    { 0.3853,    0.0000,    6.2932,   -0.0000,    0.0483,   -0.0000,    0.1184,    0.0000,    2.2975,   -0.0000,    0.0000,   -0.0000},
    {-0.0000,   -3.7482,    0.0000,   14.3981,   -0.0000,   -0.6879,   -0.0000,   -3.4115,    0.0000,    2.2362,   -0.0000,   -0.0000},
    { 3.8538,   -0.0000,   -0.6291,    0.0000,   14.6179,   -0.0000,    3.5570,   -0.0000,   -0.6171,    0.0000,    2.2362,    0.0000},
    { 0.0000,    0.9752,   -0.0000,   -1.5379,    0.0000,    7.2330,    0.0000,    0.8186,   -0.0000,   -0.0000,    0.0000,    1.0002},
};

static float K_dlqr_10_10[NUM_CTRL][NUM_STATE] = {
    {0.3834,   -0.3894,    6.2613,    0.0050,    0.0480,   -0.0000,    0.1178,   -0.1197,    2.2848,    0.0000,    0.0000,   -0.0000},
    {0.1025,   -3.7399,   -0.6370,   14.5095,    0.2607,   -0.6530,    0.1229,   -3.3761,   -0.6166,    2.2362,    0.0000,   -0.0000},
    {3.7271,   -0.0987,   -0.6249,    0.2607,   14.3269,    0.6756,    3.4164,   -0.1176,   -0.6134,    0.0000,    2.2362,    0.0000},
    {0.9755,    0.9230,   -0.0064,   -1.4600,    1.5105,    7.3849,    0.8172,    0.7607,   -0.0078,   -0.0000,    0.0000,    1.0002},
};

auto *K_dlqr;
K_dlqr = &K_dlqr_0_0;

float const deg2millirad = (float) M_PI / 180.0f;
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

  // gain scheduling
  if (5 < state->attitude.roll < 15) {
    if (5 < state->attitude.pitch < 15) {
      K_dlqr = &K_dlqr_10_10;
    } else if (-5 < state->attitude.pitch < 5) {
      K_dlqr = &K_dlqr_10_0;
    }
  } else if (-5 < state->attitude.roll < 5) {
    if (5 < state->attitude.pitch < 15) {
      K_dlqr = &K_dlqr_0_10;
    }
  }

  while (i < NUM_CTRL){
    while (j < NUM_STATE){
      res += *K_dlqr[i][j] * error[j];
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
