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

// init and status flag
static bool isInit = false;
static bool start_fall = false;
static int cnt = 0;
static float height = 0.0f;

// use full state flag
//#define FULL_STATE 1

// tune variable
static float acc_tol = 0.1; // [Gs]
static int max_cnt = 50; // 0.1 [s]

void controllerLqrReset(void) {
  isInit = false;
  start_fall = false;
  height = 0.0f;
  cnt = 0;
}

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

#define LQR_UPDATE_RATE RATE_500_HZ

#ifdef FULL_STATE

#define NUM_STATE 12
#define NUM_CTRL 4

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  isInit = true;
  leave_ground = false;
}

bool controllerLqrTest(void) { return isInit; }

// LQR controller with abs(30) angles linearization points
static float K_dlqr_0[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	 0.000000,	 3.102058,	-0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.513555,	-0.000000,	-0.000000,	 0.000000},
		{ 0.000000,	-0.000924,	 0.000000,	 0.008583,	 0.000000,	-0.000000,	 0.000000,	-0.003187,	-0.000000,	 0.001045,	 0.000000,	-0.000000},
		{ 0.000926,	-0.000000,	-0.000000,	 0.000000,	 0.008612,	 0.000000,	 0.003194,	-0.000000,	-0.000000,	 0.000000,	 0.001051,	 0.000000},
		{ 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.009470,	 0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.001144},
};

static float K_dlqr_30[NUM_CTRL][NUM_STATE] = {
		{ 0.000000,	-0.000000,	 3.102058,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	 0.000000,	 0.000000,	 0.000000},
		{ 0.000462,	-0.000800,	 0.000000,	 0.008583,	 0.000000,	 0.000000,	 0.001594,	-0.002760,	 0.000000,	 0.001045,	-0.000000,	 0.000000},
		{ 0.000802,	 0.000463,	 0.000000,	 0.000000,	 0.008612,	-0.000000,	 0.002766,	 0.001597,	 0.000000,	-0.000000,	 0.001051,	-0.000000},
		{ 0.000000,	-0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	-0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n30[NUM_CTRL][NUM_STATE] = {
		{ 0.000000,	 0.000000,	 3.102058,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.513555,	-0.000000,	 0.000000,	 0.000000},
		{-0.000462,	-0.000800,	-0.000000,	 0.008583,	-0.000000,	-0.000000,	-0.001594,	-0.002760,	-0.000000,	 0.001045,	 0.000000,	-0.000000},
		{ 0.000802,	-0.000463,	 0.000000,	-0.000000,	 0.008612,	-0.000000,	 0.002766,	-0.001597,	 0.000000,	 0.000000,	 0.001051,	-0.000000},
		{ 0.000000,	 0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_60[NUM_CTRL][NUM_STATE] = {
		{ 0.000000,	 0.000000,	 3.102058,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.513555,	 0.000000,	 0.000000,	 0.000000},
		{ 0.000800,	-0.000462,	 0.000000,	 0.008583,	-0.000000,	-0.000000,	 0.002760,	-0.001594,	 0.000000,	 0.001045,	-0.000000,	 0.000000},
		{ 0.000463,	 0.000802,	 0.000000,	-0.000000,	 0.008612,	 0.000000,	 0.001597,	 0.002766,	 0.000000,	-0.000000,	 0.001051,	 0.000000},
		{-0.000000,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.009470,	 0.000000,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.001144},
};

static float K_dlqr_n60[NUM_CTRL][NUM_STATE] = {
		{ 0.000000,	-0.000000,	 3.102058,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	 0.000000},
		{-0.000800,	-0.000462,	-0.000000,	 0.008583,	 0.000000,	 0.000000,	-0.002760,	-0.001594,	-0.000000,	 0.001045,	 0.000000,	-0.000000},
		{ 0.000463,	-0.000802,	 0.000000,	 0.000000,	 0.008612,	 0.000000,	 0.001597,	-0.002766,	 0.000000,	 0.000000,	 0.001051,	 0.000000},
		{-0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.009470,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.001144},
};

static float K_dlqr_90[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	 0.000000,	 3.102058,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	 0.513555,	-0.000000,	 0.000000,	 0.000000},
		{ 0.000924,	-0.000000,	-0.000000,	 0.008583,	 0.000000,	-0.000000,	 0.003187,	 0.000000,	-0.000000,	 0.001045,	 0.000000,	-0.000000},
		{ 0.000000,	 0.000926,	 0.000000,	 0.000000,	 0.008612,	-0.000000,	 0.000000,	 0.003194,	 0.000000,	 0.000000,	 0.001051,	-0.000000},
		{-0.000000,	-0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.009470,	-0.000000,	-0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n90[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	-0.000000,	 3.102058,	 0.000000,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	 0.000000,	 0.000000,	 0.000000},
		{-0.000924,	-0.000000,	 0.000000,	 0.008583,	-0.000000,	 0.000000,	-0.003187,	 0.000000,	 0.000000,	 0.001045,	-0.000000,	 0.000000},
		{ 0.000000,	-0.000926,	 0.000000,	-0.000000,	 0.008612,	-0.000000,	 0.000000,	-0.003194,	 0.000000,	-0.000000,	 0.001051,	-0.000000},
		{-0.000000,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	 0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_120[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	 0.000000,	 3.102058,	-0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000800,	 0.000462,	-0.000000,	 0.008583,	 0.000000,	-0.000000,	 0.002760,	 0.001594,	-0.000000,	 0.001045,	 0.000000,	 0.000000},
		{-0.000463,	 0.000802,	 0.000000,	 0.000000,	 0.008612,	-0.000000,	-0.001597,	 0.002766,	 0.000000,	 0.000000,	 0.001051,	-0.000000},
		{-0.000000,	 0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.009470,	 0.000000,	 0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n120[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	-0.000000,	 3.102058,	 0.000000,	 0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.513555,	 0.000000,	 0.000000,	-0.000000},
		{-0.000800,	 0.000462,	 0.000000,	 0.008583,	-0.000000,	 0.000000,	-0.002760,	 0.001594,	 0.000000,	 0.001045,	-0.000000,	-0.000000},
		{-0.000463,	-0.000802,	 0.000000,	-0.000000,	 0.008612,	-0.000000,	-0.001597,	-0.002766,	 0.000000,	-0.000000,	 0.001051,	-0.000000},
		{-0.000000,	-0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.009470,	 0.000000,	-0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_150[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	-0.000000,	 3.102058,	-0.000000,	 0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000462,	 0.000800,	-0.000000,	 0.008583,	-0.000000,	 0.000000,	 0.001594,	 0.002760,	-0.000000,	 0.001045,	 0.000000,	 0.000000},
		{-0.000802,	 0.000463,	 0.000000,	-0.000000,	 0.008612,	 0.000000,	-0.002766,	 0.001597,	 0.000000,	 0.000000,	 0.001051,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.000000,	 0.000000,	 0.001144},
};

static float K_dlqr_n150[NUM_CTRL][NUM_STATE] = {
		{-0.000000,	 0.000000,	 3.102058,	 0.000000,	 0.000000,	-0.000000,	-0.000000,	 0.000000,	 0.513555,	 0.000000,	 0.000000,	-0.000000},
		{-0.000462,	 0.000800,	 0.000000,	 0.008583,	 0.000000,	-0.000000,	-0.001594,	 0.002760,	 0.000000,	 0.001045,	-0.000000,	-0.000000},
		{-0.000802,	-0.000463,	 0.000000,	 0.000000,	 0.008612,	 0.000000,	-0.002766,	-0.001597,	 0.000000,	-0.000000,	 0.001051,	 0.000000},
		{ 0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.000000,	 0.009470,	-0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.000000,	 0.001144},
};

static float K_dlqr_180[NUM_CTRL][NUM_STATE] = {
		{ 0.000000,	-0.000000,	 3.102058,	-0.000000,	-0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	-0.000000,	-0.000000},
		{-0.000000,	 0.000924,	 0.000000,	 0.008583,	 0.000000,	 0.000000,	-0.000000,	 0.003187,	-0.000000,	 0.001045,	 0.000000,	 0.000000},
		{-0.000926,	 0.000000,	-0.000000,	 0.000000,	 0.008612,	-0.000000,	-0.003194,	 0.000000,	-0.000000,	 0.000000,	 0.001051,	-0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	-0.000000,	-0.000000,	 0.009470,	 0.000000,	-0.000000,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

void controllerLqr(control_t *control, setpoint_t *setpoint,
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

    // cap between -180 to 180 deg
    float yaw_cap = capAngle(state->attitude.yaw);

    if (-15 <= yaw_cap && yaw_cap < 15) {
      K = &K_dlqr_0[0][0];
      gain_type = 0;
    } else if(15 <= yaw_cap && yaw_cap < 45 ) {
      K = &K_dlqr_30[0][0];
      gain_type = 30;
    } else if(45 <= yaw_cap && yaw_cap < 75) {
      K = &K_dlqr_60[0][0];
      gain_type = 60;
    } else if(75 <= yaw_cap && yaw_cap < 105) {
      K = &K_dlqr_90[0][0];
      gain_type = 90;
    } else if(105 <= yaw_cap && yaw_cap < 135) {
      K = &K_dlqr_120[0][0];
      gain_type = 120;
    } else if(135 <= yaw_cap && yaw_cap < 165 ) {
      K = &K_dlqr_150[0][0];
      gain_type = 150;
    } else if(165 <= yaw_cap || yaw_cap < -165) {
      K = &K_dlqr_180[0][0];
      gain_type = 180;
    } else if(-165 <= yaw_cap && yaw_cap < -135) {
      K = &K_dlqr_n150[0][0];
      gain_type = 210;
    } else if(-135 <= yaw_cap && yaw_cap < -105) {
      K = &K_dlqr_n120[0][0];
      gain_type = 240;
    } else if(-105 <= yaw_cap && yaw_cap < -75) {
      K = &K_dlqr_n90[0][0];
      gain_type = 270;
    } else if(-75 <= yaw_cap && yaw_cap < -45) {
      K = &K_dlqr_n60[0][0];
      gain_type = 300;
    } else if(-45 <= yaw_cap && yaw_cap < -15) {
      K = &K_dlqr_n30[0][0];
      gain_type = 330;
    }

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

#else

#define NUM_STATE 8
#define NUM_CTRL 4

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static float actuatorThrust;  // do not use this to calculate thrust

// log PID + LQR
static float des_roll;
static float des_pitch;
static float des_yaw;

void controllerLqrInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
  leave_ground = false;
  positionControllerInit();
}

bool controllerLqrTest(void) { return isInit; }

static float K_dlqr_0[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_30[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n30[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_60[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n60[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_90[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n90[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_120[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n120[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_150[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_n150[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
};

static float K_dlqr_180[NUM_CTRL][NUM_STATE] = {
		{ 3.102058,	 0.000000,	 0.000000,	-0.000000,	 0.513555,	-0.000000,	 0.000000,	-0.000000},
		{ 0.000000,	 0.002939,	-0.000000,	 0.000000,	 0.000000,	 0.000973,	-0.000000,	 0.000000},
		{-0.000000,	 0.000000,	 0.002945,	-0.000000,	-0.000000,	 0.000000,	 0.000976,	 0.000000},
		{ 0.000000,	 0.000000,	-0.000000,	 0.009470,	-0.000000,	 0.000000,	-0.000000,	 0.001144},
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

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)  && leave_ground) {
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

    // error z [m]
    error[0] = setpoint->position.z - state->position.z;

    // error rpy [rad]
    error[1] = radians(attitudeDesired.roll - state->attitude.roll);
    error[2] = radians(-attitudeDesired.pitch + state->attitude.pitch);
    error[3] = radians(capAngle(capAngle(attitudeDesired.yaw) - capAngle(state->attitude.yaw)));

    // error vz [m/s]
    error[4] = setpoint->velocity.z - state->velocity.z;

    // error vr vp vy [rad/s]
    error[5] = radians(setpoint->attitudeRate.roll) - state_rateRoll;
    error[6] = radians(setpoint->attitudeRate.pitch) - state_ratePitch;
    error[7] = radians(setpoint->attitudeRate.yaw) - state_rateYaw;

    float* K = &K_dlqr_0[0][0];

    // cap between -180 to 180 deg
    float yaw_cap = capAngle(state->attitude.yaw);

    if (-15 <= yaw_cap && yaw_cap < 15) {
      K = &K_dlqr_0[0][0];
      gain_type = 0;
    } else if(15 <= yaw_cap && yaw_cap < 45 ) {
      K = &K_dlqr_30[0][0];
      gain_type = 30;
    } else if(45 <= yaw_cap && yaw_cap < 75) {
      K = &K_dlqr_60[0][0];
      gain_type = 60;
    } else if(75 <= yaw_cap && yaw_cap < 105) {
      K = &K_dlqr_90[0][0];
      gain_type = 90;
    } else if(105 <= yaw_cap && yaw_cap < 135) {
      K = &K_dlqr_120[0][0];
      gain_type = 120;
    } else if(135 <= yaw_cap && yaw_cap < 165 ) {
      K = &K_dlqr_150[0][0];
      gain_type = 150;
    } else if(165 <= yaw_cap || yaw_cap < -165) {
      K = &K_dlqr_180[0][0];
      gain_type = 180;
    } else if(-165 <= yaw_cap && yaw_cap < -135) {
      K = &K_dlqr_n150[0][0];
      gain_type = 210;
    } else if(-135 <= yaw_cap && yaw_cap < -105) {
      K = &K_dlqr_n120[0][0];
      gain_type = 240;
    } else if(-105 <= yaw_cap && yaw_cap < -75) {
      K = &K_dlqr_n90[0][0];
      gain_type = 270;
    } else if(-75 <= yaw_cap && yaw_cap < -45) {
      K = &K_dlqr_n60[0][0];
      gain_type = 300;
    } else if(-45 <= yaw_cap && yaw_cap < -15) {
      K = &K_dlqr_n30[0][0];
      gain_type = 330;
    }

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
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
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
#endif
LOG_GROUP_STOP(ctrlLqr)
