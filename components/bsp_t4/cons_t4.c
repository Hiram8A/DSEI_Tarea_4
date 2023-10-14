#include <stdio.h>
#include "bsp_t4.h"

void BSP_CONS_Estados(uint8_t* out, TimerHandle_t timer, bool flag){
	printf("Luminaria 1: %s,\tLuminaria 2: %s,\tPersiana: %s\n",
		   out[0] == 1 ? "ON" : "OFF",
		   out[1] == 1 ? "ON" : "OFF",
		   xTimerIsTimerActive(timer) ? flag == 1 ? "UP" : "DOWN" : "OFF");
}

void timer_callback(TimerHandle_t xTimer) {
	BSP_TIMER_Param *params = (BSP_TIMER_Param *)pvTimerGetTimerID(xTimer);

	printf("Luminaria 1: %s,\tLuminaria 2: %s,\tPersiana: OFF\n",
    params->out[0] == 1 ? "ON" : "OFF",
    params->out[1] == 1 ? "ON" : "OFF");
}
