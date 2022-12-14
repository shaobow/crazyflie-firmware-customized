#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "FREE_THROW"

/* Tuning variables */
static const float acc_tolerance = 0.1; // [Gs]
static const float pos_tolerance = 0.05; // [m]

// NOTE: CANNOT BE smaller than 1
static const int max_cnt_IDLE = 5; // total 0.1   [s]
static const int max_cnt_INAIR = 100; // total 2  [s]
static const int max_cnt_LANDING = 50; // total 1 [s]

static int cnt_IDLE;
static int cnt_INAIR;
static int cnt_LANDING;

static float estAz;
static float estZ;

static float setpoint_height; // [m]

typedef enum {
  IDLE, 
  FULLTHRUST,
  INAIR,
  MANUAL,
  LANDING,
} StateCF;

static StateCF sCF = IDLE;

static void setSetpoint(setpoint_t *setpoint, float z){
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeVelocity;
  setpoint->position.z = z;
}

void appMain()
{
  setpoint_t setpoint; 

  paramVarId_t idFlag_start_fall = paramGetVarId("ctrlLqr", "f_start_fall");
  uint8_t flag_start_fall = 0;

  /* Getting logging ID of the state estimates */
  logVarId_t idAz = logGetVarId("stateEstimate", "az");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  cnt_IDLE = 0;
  cnt_INAIR = 0;
  cnt_LANDING = 0;
  sCF = IDLE;

  setpoint_height = 0.5; // [m]

  DEBUG_PRINT("Entering free throw cycle... \n");

  while(1) {
    vTaskDelay(M2T(20)); // 20 msec (50Hz)

    /* Read data from stateEstimator */
    estAz = logGetFloat(idAz);
    estZ = logGetFloat(idZ);
    DEBUG_PRINT("az: %f ; ", (double) estAz);
    DEBUG_PRINT("z: %f \n", (double) estZ);

    /* Get parameter value for debugging */
    // flag_start_fall = paramGetVarId(idFlag_start_fall); 

    switch(sCF) {
      case IDLE:
        /* detect free fall for certain time */
        if((estAz > -1.0f - acc_tolerance) && (estAz < -1.0f + acc_tolerance)){
          cnt_IDLE++;
        } else {
          cnt_IDLE = 0;
        }

        if(cnt_IDLE > max_cnt_IDLE) {
          cnt_IDLE = 0;
          paramSetInt(idFlag_start_fall, 1);
          sCF = INAIR;
        }

        break;
      case INAIR:
        setSetpoint(&setpoint, setpoint_height);
        commanderSetSetpoint(&setpoint, 3);

        /* stabilized for certain time */
        if((estAz > 0.0f - acc_tolerance) && (estAz < 0.0f + acc_tolerance)){
          cnt_INAIR ++;
        } else {
          cnt_INAIR = 0;
        }

        if(cnt_INAIR > max_cnt_INAIR) {
          cnt_INAIR = 0;
          sCF = LANDING;
        }
        
        break;
      case LANDING:
        setSetpoint(&setpoint, setpoint_height);
        commanderSetSetpoint(&setpoint, 3);

        /* if lower to an evalation smaller than pos_tolerance */
        if(estZ < pos_tolerance){
          setpoint_height -= 0.001f;
          cnt_LANDING++;
        } else {
          cnt_LANDING = 0;
        }

        if(cnt_LANDING > max_cnt_LANDING) {
          // TODO: shut off for a hard land 
          cnt_LANDING = 0;
          flag_start_fall = 0;
          paramSetInt(idFlag_start_fall, 0);
          setpoint_height = 0.5;
          sCF = IDLE;
        }

        break;
      default:
        DEBUG_PRINT("** SOMETHING WRONG ** \n");
    }

    // // Set a parameter value 
    // //  Note, this will influence the flight quality if you change estimator
    // uint8_t new_value = 2;
    // paramSetInt(idEstimator, new_value);
    
  }
}

// PARAM_GROUP_START(appFreeThrow)
// PARAM_ADD(PARAM_UINT8, start_fall, &start_fall)
// PARAM_GROUP_STOP(appFreeThrow)

LOG_GROUP_START(appFreeThrow)
LOG_ADD(LOG_FLOAT, estAz, &estAz)
LOG_ADD(LOG_FLOAT, estZ, &estZ)
LOG_ADD(LOG_UINT8, sCF, &sCF)
LOG_ADD(LOG_UINT8, cnt_IDLE, &cnt_IDLE)
LOG_ADD(LOG_UINT8, cnt_INAIR, &cnt_INAIR)
LOG_ADD(LOG_UINT8, cnt_LANDING, &cnt_LANDING)
LOG_ADD(LOG_FLOAT, setpoint_height, &setpoint_height)
LOG_GROUP_STOP(appFreeThrow)

