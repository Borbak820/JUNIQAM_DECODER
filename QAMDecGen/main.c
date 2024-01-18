/*
 * QAMDecGen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : Martin Burger
 */ 

//#include <avr/io.h>
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

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

#include "qaminit.h"
#include "qamgen.h"
#include "qamdec.h"
#include "main.h"
#include "vAnalyze.h"

extern void vApplicationIdleHook( void );
void vLedBlink(void *pvParameters);
void vProtocol(void* pvParameters);
void vQuamDec(void * pvParameters);
void vQuamGen(void * pvParameters);



#define suspended 0
#define resumed 1

TaskHandle_t ledTask;
TaskHandle_t ProtocolTask;
TaskHandle_t QuamDecTask;
TaskHandle_t QuamGenTask;


void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
	resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	initDAC();				//Wird für den Sender Gebraucht!
	initDACTimer();			//Wird für den Sender Gebraucht!
	initGenDMA();			//Wird für den Sender Gebraucht!
 	initADC();				//Wird für den Empfänger Gebraucht!
 	initADCTimer();			//Wird für den Empfänger Gebraucht!
 	initDecDMA();			//Wird für den Empfänger Gebraucht!
	
	
	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+500, NULL, 2, &QuamGenTask);			//Wird für den Sender Gebraucht!
 	xTaskCreate(vQuamDec, NULL, configMINIMAL_STACK_SIZE+200, NULL, 2, &QuamDecTask);			//Wird für den Empfänger Gebraucht!
 	xTaskCreate(vAnalyze, NULL, configMINIMAL_STACK_SIZE+800, NULL, 1, NULL);			//Wird für den Empfänger Gebraucht!
 	xTaskCreate(vDisplay, NULL, configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);			//Wird für den Empfänger Gebraucht!
	xTaskCreate(vProtocol, NULL, configMINIMAL_STACK_SIZE, NULL, 4, &ProtocolTask);

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	vDisplayWriteStringAtPos(2,0,"DiveBuddy V1.0");
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskStartScheduler();
	return 0;
}

void vProtocol(void* pvParameters){
	int stateGen = 0;
	int stateDec = 0;
	
	vTaskSuspend(QuamDecTask);
	vTaskSuspend(QuamGenTask);
	if (stateGen = suspended){
		vTaskResume(QuamDecTask);
		for (int i = 0; i < 10; ++i) {
			vTaskDelay(1/portTICK_RATE_MS);
		}
	}
	
}
