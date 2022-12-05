/*
 * 24-774 LQR 2nd try
 * 12/03/2022
 */

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_lqr_reduced.h"
#include "physicalConstants.h"

// Logging variables
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

// static float err_x;
// static float err_y;
// static float err_z;
static float err_roll;
static float err_pitch;
static float err_yaw;

// tune control thrust
static float ctrl_thrust = 0.0;

# define NUM_STATE 6
# define NUM_CTRL 3

static float K_dlqr[NUM_CTRL][NUM_STATE] = {
    {0.0088,    0.0000,    0.0000,    0.0088,    0.0000,    0.0000},
    {0.0000,    0.0091,   -0.0000,   -0.0000,    0.0091,   -0.0000},
    {0.0000,   -0.0000,    0.0138,    0.0000,   -0.0000,    0.0138},
};


void controllerLqrReducedInit(void)
{
}

bool controllerLqrReducedTest(void)
{
  return true;
}

void controllerLqrReduced(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  control->controlMode = controlModeForceTorque;
  // control->controlMode = controlModeLegacy;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  // gyro unit: rad/sec
  float const deg2millirad = (float)M_PI / 180.0f;
  // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
  float state_rateRoll = sensors->gyro.x * deg2millirad;
  float state_ratePitch = -sensors->gyro.y * deg2millirad;
  float state_rateYaw = sensors->gyro.z * deg2millirad;

  float u[NUM_CTRL];

  float e_r = (setpoint->attitude.roll - state->attitude.roll) * deg2millirad; // unit: deg
  float e_p = (setpoint->attitude.pitch - state->attitude.pitch) * deg2millirad;
  float e_y = (setpoint->attitude.yaw - state->attitude.yaw) * deg2millirad;

  float e_rd = setpoint->attitudeRate.roll * deg2millirad - state_rateRoll; // setpoint->attitudeRate unit: deg/sec
  float e_pd = setpoint->attitudeRate.pitch * deg2millirad - state_ratePitch;
  float e_yd = setpoint->attitudeRate.yaw * deg2millirad - state_rateYaw;
  float error[NUM_STATE] = {e_r, e_p, e_y, e_rd, e_pd, e_yd};
  int i = 0, j = 0; 
  float res = 0;

  while (i < NUM_CTRL){
    while (j < NUM_STATE){
      res += K_dlqr[i][j] * error[j];
      j++;
    }

    u[i] = res;
    i++;

    j = 0;
    res = 0;
  }

  /* feedback */
  control->thrustSi = CF_MASS * 10.0f;
  control->torqueX = u[0];
  control->torqueY = u[1];
  control->torqueZ = u[2];

  cmd_thrust = control->thrustSi;
  cmd_roll = control->torqueX;
  cmd_pitch = control->torqueY;
  cmd_yaw = control->torqueZ;

  err_roll = e_r;
  err_pitch = e_p;
  err_yaw = e_y;
}

/**
 * Tunning variables for the full state Controller
 */
PARAM_GROUP_START(ctrlMel)
/**
 * @brief Thrust value
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ctrl_thrust, &ctrl_thrust)

PARAM_GROUP_STOP(ctrlMel)

/**
 * Logging variables for the command and reference signals for the
 * Mellinger controller
 */
LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)

LOG_ADD(LOG_FLOAT, err_roll, &err_roll)
LOG_ADD(LOG_FLOAT, err_pitch, &err_pitch)
LOG_ADD(LOG_FLOAT, err_yaw, &err_yaw)
LOG_GROUP_STOP(ctrlMel)
