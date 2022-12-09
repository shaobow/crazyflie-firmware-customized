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

// init flag
static bool isInit = false;

// status check
static bool leave_ground = false;
// static bool start_fall = false;
// static int cnt = 0;
// static float height = 0.0f;

// use full state flag
#define FULL_STATE 1

// tune variable
static float acc_tol = 0.1; // [Gs]
static int max_cnt = 50; // 0.1 [s]

void controllerLqrReset(void) {
  isInit = false;
  // start_fall = false;
  // height = 0.0f;
  // cnt = 0;
}

void controllerLqrInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
  leave_ground = false;
}

#define LQR_UPDATE_RATE RATE_500_HZ

#ifdef FULL_STATE

#define NUM_STATE 12
#define NUM_CTRL 4

bool controllerLqrTest(void) { return isInit; }

// static float K_dlqr_0_0[NUM_CTRL][NUM_STATE] = {
//     {-0.0000,    0.0000,    6.3246,   -0.0000,   -0.0000,    0.0000,   -0.0000,    0.0000,    2.3112,   -0.0000,   -0.0000,    0.0000},
//     { 0.0000,   -3.8730,    0.0000,   14.6873,    0.0000,    0.0000,    0.0000,   -3.5492,    0.0000,    2.2362,    0.0000,    0.0000},
//     { 3.8730,    0.0000,    0.0000,    0.0000,   14.6873,    0.0000,    3.5492,   -0.0000,    0.0000,    0.0000,    2.2362,    0.0000},
//     { 0.0000,    0.0000,   -0.0000,   -0.0000,    0.0000,    7.0711,    0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    1.0002},
// };

// static float K_dlqr_10_0[NUM_CTRL][NUM_STATE] = {
//     {-0.0000,   -0.3913,    6.2922,    0.0056,   -0.0000,    0.0000,   -0.0000,   -0.1204,    2.2982,    0.0000,   -0.0000,    0.0000},
//     { 0.0000,   -3.8532,   -0.6389,   14.7752,    0.0000,    0.0000,    0.0000,   -3.4995,   -0.6170,    2.2362,    0.0000,    0.0000},
//     { 3.7445,   -0.0000,   -0.0000,    0.0000,   14.3896,    0.6976,    3.4074,   -0.0000,   -0.0000,    0.0000,    2.2362,    0.0000},
//     { 0.9893,   -0.0000,   -0.0000,    0.0000,    1.5597,    7.2379,    0.8303,   -0.0000,   -0.0000,    0.0000,    0.0000,    1.0002},
// };

// static float K_dlqr_0_10[NUM_CTRL][NUM_STATE] = {
//     { 0.3853,    0.0000,    6.2932,   -0.0000,    0.0483,   -0.0000,    0.1184,    0.0000,    2.2975,   -0.0000,    0.0000,   -0.0000},
//     {-0.0000,   -3.7482,    0.0000,   14.3981,   -0.0000,   -0.6879,   -0.0000,   -3.4115,    0.0000,    2.2362,   -0.0000,   -0.0000},
//     { 3.8538,   -0.0000,   -0.6291,    0.0000,   14.6179,   -0.0000,    3.5570,   -0.0000,   -0.6171,    0.0000,    2.2362,    0.0000},
//     { 0.0000,    0.9752,   -0.0000,   -1.5379,    0.0000,    7.2330,    0.0000,    0.8186,   -0.0000,   -0.0000,    0.0000,    1.0002},
// };

// static float K_dlqr_10_10[NUM_CTRL][NUM_STATE] = {
//     {0.3834,   -0.3894,    6.2613,    0.0050,    0.0480,   -0.0000,    0.1178,   -0.1197,    2.2848,    0.0000,    0.0000,   -0.0000},
//     {0.1025,   -3.7399,   -0.6370,   14.5095,    0.2607,   -0.6530,    0.1229,   -3.3761,   -0.6166,    2.2362,    0.0000,   -0.0000},
//     {3.7271,   -0.0987,   -0.6249,    0.2607,   14.3269,    0.6756,    3.4164,   -0.1176,   -0.6134,    0.0000,    2.2362,    0.0000},
//     {0.9755,    0.9230,   -0.0064,   -1.4600,    1.5105,    7.3849,    0.8172,    0.7607,   -0.0078,   -0.0000,    0.0000,    1.0002},
// };

static float K_dlqr[NUM_CTRL][NUM_STATE] = {
  {0.000000,	0.000000,	3.102058,	-0.000000,	0.000000,	0.000000,	-0.000000,	0.000000,	0.513555,	-0.000000,	0.000000,	-0.000000},
  {-0.000000,	-0.000927,	-0.000000,	0.006036,	-0.000000,	0.000000,	-0.000000,	-0.001414,	-0.000000,	0.001013,	-0.000000,	0.000000},
  {0.000928,	0.000000,	0.000000,	0.000000,	0.006054,	0.000000,	0.001417,	0.000000,	0.000000,	-0.000000,	0.001018,	0.000000},
  {0.000000,	-0.000000,	-0.000000,	0.000000,	-0.000000,	0.009470,	-0.000000,	-0.000000,	-0.000000,	0.000000,	0.000000,	0.001144}};

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const uint32_t tick) {

  control->controlMode = controlModeForceTorque;

  if(RATE_DO_EXECUTE(RATE_100_HZ, tick)){
    if(setpoint->position.z >= 0.02f){
      leave_ground = true;
    }else{
      leave_ground = false;
    }
  }

  // if(RATE_DO_EXECUTE(RATE_500_HZ, tick) && !start_fall){
  //   if((state->acc.z > -1.0f - acc_tol) && (state->acc.z < -1.0f + acc_tol)){
  //     cnt++;
  //   }else{
  //     cnt = 0;
  //     start_fall = false;
  //   }

  //   if(cnt > max_cnt){
  //     height = state->position.z - 0.02f;
  //     start_fall = true;
  //   }
  // }

  // if(RATE_DO_EXECUTE(RATE_50_HZ, tick)){
  //     height -= 0.001f;
  //     if(height <= 0.05f || state->position.z <= 0.05f){
  //       control->thrustSi = 0.0f;
  //       control->torqueX = 0.0f;
  //       control->torqueY = 0.0f;
  //       control->torqueZ = 0.0f;
  //       controllerLqrReset();
  //     }
  //     setpoint->mode.z = modeAbs;
  //     setpoint->position.z = height;
  // }

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
    error[4] = radians(setpoint->attitude.pitch + state->attitude.pitch);
    error[5] = radians(setpoint->attitude.yaw - state->attitude.yaw);

    // error vx vy vz [m/s]
    error[6] = setpoint->velocity.x - state->velocity.x;
    error[7] = setpoint->velocity.y - state->velocity.y;
    error[8] = setpoint->velocity.z - state->velocity.z;

    // error vr vp vy [rad/s]
    error[9] = radians(setpoint->attitudeRate.roll) - state_rateRoll;
    error[10] = radians(setpoint->attitudeRate.pitch) - state_ratePitch;
    error[11] = radians(setpoint->attitudeRate.yaw) - state_rateYaw;

    // gain scheduling calculate K matrix
    float* K = &K_dlqr[0][0];
    // if(5 < state->attitude.roll < 15) {
    //   if (5 < state->attitude.pitch < 15) {
    //     K_dlqr_ptr = &K_dlqr_10_10;
    //   } else if (-5 < state->attitude.pitch < 5) {
    //     K_dlqr_ptr = &K_dlqr_10_0;
    //   }else if(-15 < state->attitude.pitch < -5){
        
    //   }
    // } else if(-5 < state->attitude.roll < 5) {
    //   if (5 < state->attitude.pitch < 15) {
    //     K_dlqr_ptr = &K_dlqr_0_10;
    //   }
    // } else if(-15 < state->attitude.roll < -5){

    // }

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

#define NUM_STATE 8
#define NUM_CTRL 4

bool controllerLqrTest(void) { return isInit; }

static float K_dlqr[NUM_CTRL][NUM_STATE] = {
   { 3.1125,   -0.0000,   -0.0000,   -0.0000,    0.4246,    0.0000,    0.0000,   -0.0000},
   { 0.0000,    0.0092,   -0.0000,   -0.0000,    0.0000,    0.0011,    0.0000,    0.0000},
   {-0.0000,   -0.0000,    0.0029,    0.0000,   -0.0000,   -0.0000,    0.0010,    0.0000},
   {-0.0000,    0.0000,   -0.0000,    0.0030,    0.0000,   -0.0000,    0.0000,    0.0010}};
   
void controllerLqr(control_t *control, setpoint_t *setpoint,
                   const sensorData_t *sensors, const state_t *state,
                   const uint32_t tick) {

  control->controlMode = controlModeForceTorque;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeVelocity;

  if (RATE_DO_EXECUTE(LQR_UPDATE_RATE, tick)) {
    // using sensor info for state estimation of dotRoll, dotPitch, and dotYaw
    // gyro unit: rad/sec
    float state_rateRoll = radians(sensors->gyro.x);
    float state_ratePitch = radians(sensors->gyro.y);
    float state_rateYaw = radians(sensors->gyro.z);

    float u[NUM_CTRL] = {0.0f};
    float error[NUM_STATE] = {0.0f};

    // error z [m]
    error[0] = setpoint->position.z - state->position.z;

    // error rpy [rad]
    error[1] = radians(setpoint->attitude.roll - state->attitude.roll);
    error[2] = radians(setpoint->attitude.pitch + state->attitude.pitch);
    error[3] = radians(setpoint->attitude.yaw - state->attitude.yaw);

    // error vz [m/s]
    error[4] = setpoint->velocity.z - state->velocity.z;

    // error vr vp vy [rad/s]
    error[5] = radians(setpoint->attitudeRate.roll) - state_rateRoll;
    error[6] = radians(setpoint->attitudeRate.pitch) - state_ratePitch;
    error[7] = radians(setpoint->attitudeRate.yaw) - state_rateYaw;

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
/**
 * @brief LQR acceleration tolerance [m/s^2]
 */
PARAM_ADD(PARAM_FLOAT, acc_tol, &acc_tol)
/**
 * @brief LQR max counter number
 */
PARAM_ADD(PARAM_INT8, max_cnt, &max_cnt)
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
// /**
//  * @brief start fall flag
//  */
// LOG_ADD(LOG_INT8, start_fall, &start_fall)
// /**
//  * @brief current set point height
//  */
// LOG_ADD(LOG_FLOAT, height, &height)
LOG_GROUP_STOP(ctrlLqr)
