/*
 * 24-774 LQR 2nd try
 * 12/03/2022
 */

#include "controller_lqr.h"
#include "position_controller.h"

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
static int gain_type = 0;

// init flag
static bool isInit = false;

// status check
static bool leave_ground = false;

// use full state flag
//#define FULL_STATE 1

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

#define LQR_UPDATE_RATE RATE_250_HZ

#ifdef FULL_STATE

#define NUM_STATE 12
#define NUM_CTRL 4

void controllerLqrInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
  leave_ground = false;
}

bool controllerLqrTest(void) { return isInit; }

// %DT-LQR
// Ts = 1/250; % 250 Hz
// R = diag([1e3 1e7 1e7 1e7]);
// Q = diag([1e1 1e1 1e4 ...
//           1e2 1e2 1e3 ...
//           1e1 1e1 1e2 ...
//           1e1 1e1 1e1]);
static float K_dlqr_0[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	-0.000000,	 3.043261,	-0.000000,	 0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.506903,	 0.000000,	 0.000000,	-0.000000},
		{-0.000000,	-0.000861,	 0.000000,	 0.005624,	-0.000000,	 0.000000,	 0.000000,	-0.001315,	 0.000000,	 0.000948,	-0.000000,	-0.000000},
		{ 0.000864,	 0.000000,	 0.000000,	-0.000000,	 0.005650,	-0.000000,	 0.001320,	 0.000000,	 0.000000,	-0.000000,	 0.000953,	-0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.008980,	-0.000000,	-0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.001094},
};

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const uint32_t tick) {

  control->controlMode = controlModeForceTorque;

  if(RATE_DO_EXECUTE(RATE_100_HZ, tick)){
    if(setpoint->position.z >= 0.01f){
      leave_ground = true;
    }else{
      control->thrustSi = 0.0f;
      control->torqueX = 0.0f;
      control->torqueY = 0.0f;
      control->torqueZ = 0.0f;
      leave_ground = false;
    }
  }

  if (RATE_DO_EXECUTE(LQR_UPDATE_RATE, tick) && leave_ground) {
    // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
    // gyro unit: rad/sec
    float state_rateRoll = radians(sensors->gyro.x);
    float state_ratePitch = radians(sensors->gyro.y);
    float state_rateYaw = radians(sensors->gyro.z);

    float u[NUM_CTRL] = {0.0f};
    float error[NUM_STATE] = {0.0f};

    // error xyz [m]
    error[0] = setpoint->position.x - state->position.x;
    error[1] = setpoint->position.y - state->position.y;
    error[2] = setpoint->position.z - state->position.z;

    // error rpy [rad]
    error[3] = radians(setpoint->attitude.roll - state->attitude.roll);
    error[4] = radians(-setpoint->attitude.pitch + state->attitude.pitch);
    error[5] = radians(capAngle(capAngle(setpoint->attitude.yaw) - capAngle(state->attitude.yaw)));

    // error vx vy vz [m/s]
    error[6] = setpoint->velocity.x - state->velocity.x;
    error[7] = setpoint->velocity.y - state->velocity.y;
    error[8] = setpoint->velocity.z - state->velocity.z;

    // error vr vp vy [rad/s]
    error[9] = radians(setpoint->attitudeRate.roll) - state_rateRoll;
    error[10] = radians(setpoint->attitudeRate.pitch) - state_ratePitch;
    error[11] = radians(setpoint->attitudeRate.yaw) - state_rateYaw;

    // gain scheduling calculate K matrix
    float* K = &K_dlqr_0[0][0];

    // matrix multiplication
    float res = 0.0f;
    for (int i = 0; i < NUM_CTRL; i++) {
      res = 0.0f;
      for (int j = 0; j < NUM_STATE; j++) {
        res += *(K+i*NUM_STATE+j) * error[j];
      }
      u[i] = res;
    }

    // feedback
    control->thrustSi = u[0] + CF_MASS * 9.81f;
    if(control->thrustSi > 0){
      control->torqueX = u[1];
      control->torqueY = u[2];
      control->torqueZ = u[3];
    }else{
      control->torqueX = 0.0f;
      control->torqueY = 0.0f;
      control->torqueZ = 0.0f;
    }

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

#else

#define NUM_STATE 6
#define NUM_CTRL 3

static attitude_t attitudeDesired;
static float actuatorThrust;  // do not use this to calculate thrust

// log PID + LQR
static float des_roll;
static float des_pitch;
static float des_yaw;

// tunable scale factor
static float scale = 0.00000736f; // 0.0027*9.81/36000
static float rp_gain = 0.85f;

void controllerLqrInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
  leave_ground = false;
  positionControllerInit();
}

bool controllerLqrTest(void) { return isInit; }

// %DT-LQR
// Ts = 1/250; % 250 Hz
// R = diag([1e7 1e7 1e7]);
// Q = diag([1e2 1e2 1e3 ...
//           1e1 1e1 1e1]);
static float K_dlqr_0[NUM_CTRL][NUM_STATE] = {
		{ 0.002740,	 0.000000,	-0.000000,	 0.000910,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002751,	 0.000000,	-0.000000,	 0.000914,	 0.000000},
		{-0.000000,	-0.000000,	 0.008980,	-0.000000,	-0.000000,	 0.001094},
};

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const uint32_t tick) {

  control->controlMode = controlModeForceTorque;

  if(RATE_DO_EXECUTE(RATE_100_HZ, tick)){
    if(setpoint->position.z >= 0.01f){
      leave_ground = true;
    }else{
      control->thrustSi = 0.0f;
      control->torqueX = 0.0f;
      control->torqueY = 0.0f;
      control->torqueZ = 0.0f;
      leave_ground = false;
    }
  }

  if (RATE_DO_EXECUTE(LQR_UPDATE_RATE, tick)  && leave_ground) {
    if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    des_yaw = attitudeDesired.yaw;
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick) && leave_ground) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    des_roll = attitudeDesired.roll;
    des_pitch = attitudeDesired.pitch;
  }

  if (RATE_DO_EXECUTE(LQR_UPDATE_RATE, tick) && leave_ground) {
    // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
    // gyro unit: rad/sec
    float state_rateRoll = radians(sensors->gyro.x);
    float state_ratePitch = radians(sensors->gyro.y);
    float state_rateYaw = radians(sensors->gyro.z);

    float u[NUM_CTRL] = {0.0f};
    float error[NUM_STATE] = {0.0f};

    // error rpy [rad]
    error[0] = radians(attitudeDesired.roll*rp_gain - state->attitude.roll);
    error[1] = radians(-attitudeDesired.pitch*rp_gain + state->attitude.pitch);
    error[2] = radians(capAngle(capAngle(attitudeDesired.yaw) - capAngle(state->attitude.yaw)));

    // error vr vp vy [rad/s]
    error[3] = radians(setpoint->attitudeRate.roll) - state_rateRoll;
    error[4] = radians(setpoint->attitudeRate.pitch) - state_ratePitch;
    error[5] = radians(setpoint->attitudeRate.yaw) - state_rateYaw;

    float* K = &K_dlqr_0[0][0];

    // matrix multiplication
    float res = 0.0f;
    for (int i = 0; i < NUM_CTRL; i++) {
      res = 0.0f;
      for (int j = 0; j < NUM_STATE; j++) {
        res += *(K+i*NUM_STATE+j) * error[j];
      }
      u[i] = res;
    }

    control->thrustSi = actuatorThrust*scale;
    if(control->thrustSi > 0){
      control->torqueX = u[0];
      control->torqueY = u[1];
      control->torqueZ = u[2];
    }else{
      control->torqueX = 0.0f;
      control->torqueY = 0.0f;
      control->torqueZ = 0.0f;
      positionControllerResetAllPID();
    }

    // log values
    cmd_thrust = control->thrustSi;
    cmd_roll = control->torqueX;
    cmd_pitch = control->torqueY;
    cmd_yaw = control->torqueZ;

    err_z = error[0];
    err_roll = error[1];
    err_pitch = error[2];
  }
}

#endif

/**
 * Tuning settings for LQR controller
 */

PARAM_GROUP_START(ctrlLqr)
#ifndef FULL_STATE
/**
 * @brief reduced LQR thrust scale factor
 */
PARAM_ADD(PARAM_FLOAT, scale, &scale)
/**
 * @brief reduced LQR roll pitch scale factor
 */
PARAM_ADD(PARAM_FLOAT, rp_gain, &rp_gain)
#endif
PARAM_GROUP_STOP(ctrlLqr)

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
/**
 * @brief gain schedule type
 */
LOG_ADD(LOG_INT8, gain_type, &gain_type)
#ifndef FULL_STATE
/**
 * @brief desired roll
 */
LOG_ADD(LOG_FLOAT, des_roll, &des_roll)
/**
 * @brief desired pitch
 */
LOG_ADD(LOG_FLOAT, des_pitch, &des_pitch)
/**
 * @brief desired yaw
 */
LOG_ADD(LOG_FLOAT, des_yaw, &des_yaw)
/**
 * @brief pid thrust
 */
LOG_ADD(LOG_FLOAT, pid_thrust, &actuatorThrust)
#endif
LOG_GROUP_STOP(ctrlLqr)
