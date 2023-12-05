#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"

#include "port_driver.h"
#include "math.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#include "queue.h"
#include "event_groups.h"
#include "stack_macros.h"
#include "mem_check.h"
#include "init.h"

#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"
#include "stdio.h"
#include "Display.h"
#include "qamdec.h"



void vDisplay(void* pvParameters){
	
	resetReason_t reason = getResetReason();
	vDisplayClear();
	vDisplayWriteStringAtPos(0,0, "%f", reconstructedFloat);
	vDisplayWriteStringAtPos(1,0, "%f", reconstructedFloat);
	vDisplayWriteStringAtPos(2,0, "%f", reconstructedFloat);
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskDelay(1000/portTICK_RATE_MS);
}
