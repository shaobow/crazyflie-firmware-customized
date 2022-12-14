#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

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

typedef enum {
  IDLE, 
  FULLTHRUST,
  INAIR,
  MANUAL,
  LANDING,
} StateCF;

static StateCF sCF;

void appMain()
{
  // paramVarId_t idEstimator = paramGetVarId("stabilizer", "estimator");
  // uint8_t estimator_type = 0;

  /* Getting logging ID of the state estimates */
  logVarId_t idAz = logGetVarId("stateEstimate", "az");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  cnt_IDLE = 0;
  cnt_INAIR = 0;
  cnt_LANDING = 0;
  sCF = IDLE;

  DEBUG_PRINT("Entering free throw cycle... \n");

  while(1) {
    vTaskDelay(M2T(20)); // 20 msec

    /* Read data from stateEstimator */
    estAz = logGetFloat(idAz);
    estZ = logGetFloat(idZ);
    DEBUG_PRINT("az: %f ; ", (double) estAz);
    DEBUG_PRINT("z: %f \n", (double) estZ);

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
          sCF = INAIR;
        }

        break;
      case INAIR:
        /* stabilized for certain time */
        if((estAz > 0.0f - acc_tolerance) && (estAz < 0.0f + acc_tolerance)){
          cnt_INAIR++;
        } else {
          cnt_INAIR = 0;
        }

        if(cnt_INAIR > max_cnt_INAIR) {
          cnt_INAIR = 0;
          sCF = LANDING;
        }
        
        break;
      case LANDING:
        /* if lower to an evalation smaller than pos_tolerance */
        if(estZ < pos_tolerance){
          cnt_LANDING++;
        } else {
          cnt_LANDING = 0;
        }

        if(cnt_LANDING > max_cnt_LANDING) {
          // TODO: shut off for a hard land 
          cnt_LANDING = 0;
          sCF = IDLE;
        }

        break;
      default:
        DEBUG_PRINT("** SOMETHING WRONG ** \n");
    }

    // DEBUG_PRINT("cnt: %d ; state of CF: %d \n", cnt, sCF);

    // // Set a parameter value 
    // //  Note, this will influence the flight quality if you change estimator
    // uint8_t new_value = 2;
    // paramSetInt(idEstimator, new_value);
    
  }
}

// PARAM_GROUP_START(appFreeThrow)
// PARAM_ADD(PARAM_UINT8, goLeft, &goLeft)
// PARAM_ADD(PARAM_FLOAT, distanceWall, &distanceToWall)
// PARAM_ADD(PARAM_FLOAT, maxSpeed, &maxForwardSpeed)
// PARAM_GROUP_STOP(appFreeThrow)

LOG_GROUP_START(appFreeThrow)
LOG_ADD(LOG_FLOAT, estAz, &estAz)
LOG_ADD(LOG_FLOAT, estZ, &estZ)
LOG_ADD(LOG_UINT8, sCF, &sCF)
LOG_ADD(LOG_UINT8, cnt_IDLE, &cnt_IDLE)
LOG_ADD(LOG_UINT8, cnt_INAIR, &cnt_INAIR)
LOG_ADD(LOG_UINT8, cnt_LANDING, &cnt_LANDING)
LOG_GROUP_STOP(appFreeThrow)