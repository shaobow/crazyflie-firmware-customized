#include "app.h"
#include "commander.h"
#include "param.h"
#include "log.h"
#include "stabilizer_types.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FREE_THROW"
#include "debug.h"

static int cnt = 0;
static float traj_height = 0.0f;

// tune variable
static const float acc_tolerance = 0.1; // [Gs]
static const float pos_tolerance = 0.05; // [m]

static const int time_delay = 2000;
static const int max_cnt_IDLE = 50; // 0.1 [s]
static const int max_cnt_INAIR = 200;
static const int max_cnt_LANDING = 200;

static float estAz;
static float estPoseZ;

typedef enum {
    IDLE,
    FULLTHRUST, 
    INAIR, 
    LANDING,
} StateCF;

static StateCF sCF = IDLE;

// static void setTrajSetpoint(setpoint_t *setpoint, float z){
//     setpoint->mode.z = modeAbs;
//     setpoint->position.z = z; 
// }

void appMain(){
    // setpoint_t setpoint; 

    /* Getting Logging ID of the state estimates */
    logVarId_t idStateEstimateAz = logGetVarId("stateEstimate", "az"); // state->acc.z (TODO: may not be the LOG_FLOAT)
    logVarId_t idStateEstimatePosZ = logGetVarId("stateEstimate", "z"); // state->position.z

    vTaskDelay(M2T(time_delay));
    DEBUG_PRINT("Entering free throw cycle...\n");

    while(1){
        estAz = logGetFloat(idStateEstimateAz);
        estPoseZ = logGetFloat(idStateEstimatePosZ);

        switch(sCF){
            case IDLE: // @ RATE_DO_EXECUTE(RATE_500_HZ, tick)
                if((estAz > -1.0f - acc_tolerance) && (estAz < -1.0f + acc_tolerance))
                    cnt++;
                else 
                    cnt = 0;

                if(cnt > max_cnt_IDLE){
                    traj_height = estPoseZ - 0.02f; // TODO: calcualte how much already fell 
                    cnt = 0;
                    // sCF = FULLTHRUST;
                    sCF = INAIR; // TODO: skip FULLTHRUST for now;
                }

                DEBUG_PRINT("stateCF: IDLE\n");
                break;
            case FULLTHRUST:
                // TODO: give an initial full thrust 
                break;
            case INAIR:
                // TODO: calculate trajectory 
                
                if((estAz > 0.0f - acc_tolerance) && (estAz < 0.0f + acc_tolerance))
                    cnt++;
                else 
                    cnt = 0;

                if(cnt > max_cnt_INAIR){
                    cnt = 0;
                    sCF = LANDING; 
                }

                DEBUG_PRINT("stateCF: INAIR\n");
                break;
            case LANDING:
                // call HighLevelcommander to land 

                if((estPoseZ > -pos_tolerance) && (estPoseZ < pos_tolerance))
                    cnt++;
                else 
                    cnt = 0;

                if(cnt > max_cnt_LANDING){
                    cnt = 0;
                    sCF = LANDING; 
                }

                DEBUG_PRINT("stateCF: LANDING\n");
        }
        vTaskDelay(M2T(time_delay));
    }
}

/**
 * Tuning settings for LQR controller
 */
PARAM_GROUP_START(app_free_throw)
/**
 * @brief max counter number for IDLE state
 */
PARAM_ADD(PARAM_INT8, max_cnt_IDLE, &max_cnt_IDLE)
/**
 * @brief max counter number for INAIR state
 */
PARAM_ADD(PARAM_INT8, max_cnt_INAIR, &max_cnt_INAIR)
/**
 * @brief max counter number for LANDING state
 */
PARAM_ADD(PARAM_INT8, max_cnt_LANDING, &max_cnt_LANDING)
PARAM_GROUP_STOP(app_free_throw)


/**
 * Logging variables for the command and reference signals for the
 * app_free_throw
 */
LOG_GROUP_START(app_free_throw)
/**
 * @brief State of CF (IDLE, FULLTHRUST, INAIR, LANDING, etc.)
 */
LOG_ADD(LOG_UINT8, sCF, &sCF)
LOG_ADD(LOG_FLOAT, estAz, &estAz)
LOG_ADD(LOG_FLOAT, estPoseZ, &estPoseZ)
LOG_GROUP_STOP(app_free_throw)