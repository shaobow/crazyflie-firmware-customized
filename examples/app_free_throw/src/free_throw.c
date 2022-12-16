/* 
Controller type Any(0), PID(1), Mellinger(2), INDI(3), Brescianini(4) (Default: 0)
*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

// #include "commander.h"
#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "FREE_THROW"

/* Tuning variables */
static const float acc_tolerance = 0.1; // [Gs]
static const float pos_tolerance = 0.05f; // [m]
static const float hand_tolerance = 0.4f;

static const float height_takeoff = 1.0f; // [m]
static const float duration_takeoff = 1.0f; // [sec]
static const float height_land = 0.0f; /* target final landing height */
static const float duration_land = 1.0f; 

// NOTE: CANNOT BE smaller than 1
static const int time_delay_hz = 100; // [Hz]
static const int time_delay = 1000 / time_delay_hz; // [ms] per iteration; 100Hz
static const int max_cnt_IDLE = 50 / time_delay; // total 0.05   [s]
static const int max_cnt_INAIR = 2000 / time_delay; // total 2  [s]
static const int max_cnt_HAND = 100 / time_delay; // total 0.1  [s]
static const int max_cnt_LANDING = 1000 / time_delay; // total 1 [s]


static int cnt_IDLE;
static int cnt_INAIR;
static int cnt_HAND;
static int cnt_LANDING;

static float estAz;
static float estZ;

typedef enum {
  IDLE, 
  INAIR,
  HAND,
  LANDING,
} StateCF;

static StateCF sCF = IDLE;

void appMain()
{
  paramVarId_t idFlag_start_fall = paramGetVarId("ctrlLqr", "flag_start_fall");
  uint8_t flag_received = paramGetInt(idFlag_start_fall); // get current controller type
  DEBUG_PRINT("flag_start_fall type is: %d \n", flag_received);

  /* Getting logging ID of the state estimates */
  logVarId_t idAz = logGetVarId("stateEstimate", "az");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");

  cnt_IDLE = 0;
  cnt_INAIR = 0;
  cnt_HAND = 0;
  cnt_LANDING = 0;

  DEBUG_PRINT("Entering free throw cycle... \n");

  while(1) {
    vTaskDelay(M2T(time_delay));

    /* Read data from stateEstimator */
    estAz = logGetFloat(idAz);
    estZ = logGetFloat(idZ);

    switch(sCF) {
      case IDLE:
        /* detect free fall for certain time */
        if((estAz > -1.0f - acc_tolerance) && (estAz < -1.0f + acc_tolerance)){
          cnt_IDLE++;
        } else {
          cnt_IDLE = 0;
        }

        if(cnt_IDLE > max_cnt_IDLE) {
          /* reset */
          cnt_IDLE = 0;

          /* send CF command */
          paramSetInt(idFlag_start_fall, 1);
          flag_received = paramGetInt(idFlag_start_fall);
          DEBUG_PRINT("[IDLE] flag_start_fall type NOW is: %d \n", flag_received);

          crtpCommanderHighLevelTakeoff(height_takeoff, duration_takeoff);

          /* switch state */
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
          /* reset */
          cnt_INAIR = 0;

          // crtpCommanderHighLevelStop();

          /* switch state */
          sCF = HAND;

          // crtpCommanderHighLevelLand(height_land, duration_land);
          // sCF = LANDING;
        }
        
        break;
      case HAND:
          if(estZ < hand_tolerance) {
          //   cnt_HAND++;
          // } else {
          //   cnt_HAND = 0;
          // }

          // if(cnt_HAND > max_cnt_HAND){
            /* reset */
            cnt_HAND = 0;

            crtpCommanderHighLevelLand(height_land, duration_land);

            /* switch state */
            sCF = LANDING;
          }
      case LANDING:
        /* if lower to an evalation smaller than pos_tolerance */
        if(estZ < pos_tolerance){
          cnt_LANDING++;
        } else {
          cnt_LANDING = 0;
        }

        if(cnt_LANDING > max_cnt_LANDING) {
          /* reset */
          cnt_LANDING = 0;

          /* send CF command */
          paramSetInt(idFlag_start_fall, 0);
          flag_received = paramGetInt(idFlag_start_fall);
          DEBUG_PRINT("[LANDING] flag_start_fall type NOW is: %d \n", flag_received);

          /* switch state */
          sCF = IDLE;
        }
        break;
      default:
        DEBUG_PRINT("** SOMETHING WRONG ** \n");
    }
    
  }
}

LOG_GROUP_START(appFreeThrow)
LOG_ADD(LOG_FLOAT, estAz, &estAz)
LOG_ADD(LOG_FLOAT, estZ, &estZ)
// LOG_ADD(LOG_FLOAT, height, &height)
LOG_ADD(LOG_UINT8, sCF, &sCF)
// LOG_ADD(LOG_UINT8, flag_new, &flag_new)
// LOG_ADD(LOG_UINT8, flag_received, &flag_received)
LOG_GROUP_STOP(appFreeThrow)