#include "app.h"
#include "commander.h"
#include "param.h"
#include "log.h"
#include "stabilizer_types.h"

#include "FreeRTO.h"
#include "task.h"

#define DEBUG_MODULE "FREE_THROW"
#include "debug.h"

// static bool flag_start_fall = false;
static int cnt = 0;
static float traj_height = 0.0f;

static const float acc_tolerance = 0.1; // [Gs]
static const int max_cnt = 50; // 0.1 [s]

const TickType_t xDelay = 500/portTICK_PERIOD_MS; /* Block for 500 ms*/

static enum StateCF{
    IDLE = 0,
    FULLTHRUST = 1, 
    INAIR = 2, 
    LANDING = 3,
} sCF;

setTrajSetpoint(setpoint_t *setpoint, float traj_height){
    setpoint->mode.z = modeAbs;
    setpoint->position.z = traj_height; 
}

void appMain(){
    setpoint_t setpoint; 
    float estAz;
    float estPoseZ;

    // Getting Logging ID of the state estimates 
    logVarId_t idStabilizerAz = logGetVarId("stabilizer", "az"); // state->acc.z (TODO: may not be the LOG_FLOAT)
    logVarId_t idStabilizerPosZ = logGetVarId("stabilizer", "z"); // state->position.z
    
    sCF = IDLE; 

    DEBUG_PRINT("Entering free throw cycle...\n");

    while(1){
        estAz = logGetFloat(idStablizerAz);
        estPoseZ = logGetFloat(idStabilizerPosZ);

        switch(sCF){
            case IDLE:
                if((estAz > -1.0f - acc_tolerance) && (estAz < -1.0f + acc_tolerance)){
                    cnt++;
                } else {
                    cnt = 0;
                } 

                if(cnt > max_cnt){
                    height = estPoseZ - 0.02f;
                    sCF = FULLTHRUST;
                }
                break;
            case FULLTHRUST:
            // TO DO: give an initial full thrust 
            break;
            case INAIR:
            
            break;
            case LANDING:
        }
    }

    // step 1: if detects -g, set traj 
    while(!flag_start_fall){
        estAz = logGetFloat(idStabilizerAz);
        estPoseZ = logGetFloat(idStabilizerPosZ);

        if((estAz > -1.0f - acc_tolerance) && (estAz < -1.0f + acc_tolerance)){
            cnt ++;
        } else {
            cnt = 0;
            flag_start_fall = false; 
        }

        if(cnt > max_cnt){
            traj_height = estPoseZ - 0.02f;
            flag_start_fall = true;
        } else {
            vTaskDelay(M2T(10)); // TODO: set to 500_hz
        }
    }

    while(flag_start_fall){
        setTrajSetpoint(&setpoint, traj_height); // TODO: set traj
        commanderSetpoint(&setpoint, 3); // TODO: commanding highLevel_commander 

        estPoseZ = logGetFloat(idStabilizerPoseZ);
        if(estPoseZ <= 0.05f || estPoseZ <= 0.05f){
            setTrajSetpoint(&setpoint, 0); // TODO: landing 
            commanderSetpoint(&setpoint, 3); // TODO: commanding
        }
    }
}