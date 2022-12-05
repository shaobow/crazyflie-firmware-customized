/**
 * 24-774 LQR 2nd try
 * 12/03/2022
 */
#ifndef __CONTROLLER_LQR_REDUCED_H__
#define __CONTROLLER_LQR_REDUCED_H__

#include "stabilizer_types.h"

void controllerLqrReducedInit(void);
bool controllerLqrReducedTest(void);
void controllerLqrReduced(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_LQR_REDUCED_H__
