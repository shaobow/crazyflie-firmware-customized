/*
 * 24-774 LQR 2nd try
 * 12/03/2022
 */

#include "controller_lqr.h"

#include "log.h"
#include "math.h"
#include "math3d.h"
#include "param.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"

// Logging variables
static float cmd_thrust = 0.0;
static float cmd_roll = 0.0;
static float cmd_pitch = 0.0;
static float cmd_yaw = 0.0;

static float err_z = 0.0;
static float err_roll = 0.0;
static float err_pitch = 0.0;

static float coeff = 0.0;

#define NUM_STATE 12
#define NUM_CTRL 4

static float K_dlqr[NUM_CTRL][NUM_STATE] = {
    {1.2582, -1.7788, 4.7904, -0.1549, 0.0516, 0.0006, 0.3540, -0.5002, 1.3885,
     -0.0001, 0, 0},
    {0.0033, -0.0154, -0.0176, 0.0730, 0.0051, -0.0053, 0.0040, -0.0112,
     -0.0140, 0.0089, 0, 0},
    {0.0071, -0.0011, -0.0061, 0.0052, 0.0460, 0.0071, 0.0052, -0.0014, -0.0051,
     0, 0.0091, 0},
    {0.0452, 0.0197, -0.0121, -0.0249, 0.0322, 0.1083, 0.0311, 0.0103, -0.0117,
     0, 0, 0.0138}};

void controllerLqrInit(void) {}

bool controllerLqrTest(void) { return true; }

void controllerLqr(control_t *control, setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const uint32_t tick) {
  float const deg2rad = M_PI_F / 180.0f;
  coeff = deg2rad;

  control->controlMode = controlModeForceTorque;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeVelocity;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
    // gyro unit: rad/sec
    float state_rateRoll = sensors->gyro.x * deg2rad;
    float state_ratePitch = -sensors->gyro.y * deg2rad;
    float state_rateYaw = sensors->gyro.z * deg2rad;

    float u[NUM_CTRL] = {0.0f};
    float error[NUM_STATE] = {0.0f};

    // error xyz [m]
    error[0] = setpoint->position.x - state->position.x;
    error[1] = setpoint->position.y - state->position.y;
    error[2] = setpoint->position.z - state->position.z;

    // error rpy [rad]
    error[3] = (setpoint->attitude.roll - state->attitude.roll) * deg2rad;
    error[4] = (setpoint->attitude.pitch + state->attitude.pitch) * deg2rad;
    error[5] = (setpoint->attitude.yaw - state->attitude.yaw) * deg2rad;

    // error vx vy vz [m/s]
    error[6] = setpoint->velocity.x - state->velocity.x;
    error[7] = setpoint->velocity.y - state->velocity.y;
    error[8] = setpoint->velocity.z - state->velocity.z;

    // error vr vp vy [rad/s]
    error[9] = setpoint->attitudeRate.roll * deg2rad - state_rateRoll;
    error[10] = setpoint->attitudeRate.pitch * deg2rad - state_ratePitch;
    error[11] = setpoint->attitudeRate.yaw * deg2rad - state_rateYaw;

    // matrix multiplication
    float res = 0.0f;
    for (int i = 0; i < NUM_CTRL; i++) {
      res = 0.0f;
      for (int j = 0; j < NUM_STATE; j++) {
        res += K_dlqr[i][j] * error[j];
      }
      u[i] = res;
    }

    // feedback
    control->thrustSi = u[0] + CF_MASS * 9.81f;
    control->torqueX = u[1];
    control->torqueY = u[2];
    control->torqueZ = u[3];

    // log values
    cmd_thrust = control->thrustSi;
    cmd_roll = control->torqueX;
    cmd_pitch = control->torqueY;
    cmd_yaw = control->torqueZ;

    err_z = error[2];
    err_roll = error[3];
    err_pitch = error[4];
  }
}

/**
 * Logging variables for the command and reference signals for the
 * LQR controller
 */

LOG_GROUP_START(ctrlLqr)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Z error
 */
LOG_ADD(LOG_FLOAT, err_z, &err_z)
/**
 * @brief Roll error
 */
LOG_ADD(LOG_FLOAT, err_roll, &err_roll)
/**
 * @brief Pitch error
 */
LOG_ADD(LOG_FLOAT, err_pitch, &err_pitch)

// LOG_ADD(LOG_FLOAT, des_z, &des_z)
// LOG_ADD(LOG_FLOAT, des_roll, &des_roll)
// LOG_ADD(LOG_FLOAT, des_pitch, &des_pitch)

LOG_ADD(LOG_FLOAT, coeff, &coeff)
LOG_GROUP_STOP(ctrlLqr)
