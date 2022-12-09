/**
 * 24-774 LQR 2nd try
 * 12/03/2022
 */
#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

void controllerLqrInit(void);
bool controllerLqrTest(void);
void controllerLqr(control_t *control, const setpoint_t *setpoint,
                                        const sensorData_t *sensors,
                                        const state_t *state,
                                        const uint32_t tick);

#endif //__CONTROLLER_LQR_H__
