/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
*  Author: Chaos
*/ 

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
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"
#include "errorHandler.h"

#include "qaminit.h"
#include "qamdec.h"
/*Defines*/
#define quarterjump1 7
#define quarterjump2 8 //Perfekt f�r Sync Weil diese Spr�nge nur in einem Fall auftreten k�nnen 3 -> 0
#define quarterjump3 9

#define halfjump1 15
#define halfjump2 16
#define halfjump3 17

#define threequartersjump1 23
#define threequartersjump2 24
#define threequartersjump3 25

#define fulljump1 31
#define fulljump2 32
#define fulljump3 33

#define onequarterjump1 39
#define onequarterjump2 40
#define onequarterjump3 41

#define onehalfjump1 47
#define onehalfjump2 48
#define onehalfjump3 49

#define onethreequartersjump1 55
#define onethreequartersjump2 56 //Perfekt f�r Sync Weil diese Spr�nge nur in einem Fall auftreten k�nnen 0 -> 3
#define onethreequartersjump3 57


/*Queue Init*/
QueueHandle_t decoderQueue;

/*Array Init*/
uint16_t ringbuffer[256]; //Array nicht Global erstellen 
uint8_t receivebuffer[100];
uint8_t maxpos[2];

/*Var Init*/
uint16_t max = 0;
uint8_t master_offset = 32;
uint8_t diff_offset = 0;
uint8_t Offset = 0;
uint8_t lastnumber = 3;
uint16_t Ringbuffer_Pos = 0;
uint8_t j = 0;
uint8_t k = 0;
uint8_t debug = 0;

/*Pointer Init*/
uint16_t * p_Writing = &ringbuffer[0];
uint16_t * p_Reading = &ringbuffer[0];
uint8_t * p_MAXPOS1r = &maxpos[0];
uint8_t * p_MAXPOS2r = &maxpos[1];

void quarterjump(void){				
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
		case 1:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
		case 2:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
		case 3:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
	}
}

void halfjump(void){
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
		case 1:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
		case 2:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
		case 3:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
	}
	
}

void threequartersjump(void){
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
		case 1:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
		case 2:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
		case 3:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
	}
	
}

void fulljump(void){
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
		case 1:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
		case 2:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
		case 3:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
	}
}

void onequarterjump(void){
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
		case 1:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
		case 2:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
		case 3:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
	}
}

void onehalfjump(void){
		switch(lastnumber){
			case 0:
			receivebuffer[k] = 2;
			lastnumber = 2;
			break;
			case 1:
			receivebuffer[k] = 3;
			lastnumber = 3;
			break;
			case 2:
			receivebuffer[k] = 0;
			lastnumber = 0;
			break;
			case 3:
			receivebuffer[k] = 1;
			lastnumber = 1;
			break;
		}
}

void onethreequartersjump(void){
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 3;
		lastnumber = 3;
		break;
		case 1:
		receivebuffer[k] = 0;
		lastnumber = 0;
		break;
		case 2:
		receivebuffer[k] = 1;
		lastnumber = 1;
		break;
		case 3:
		receivebuffer[k] = 2;
		lastnumber = 2;
		break;
	}
}

void analyzediff(void){
	
	//Schleife erstellen f�r die Grosse von unserem Paket: Im Testfall sind es 58 Elemente
	//Nach schleife den Sync Modus wieder starten.
		switch(Offset){ // Startwert ist 3
			case quarterjump1: //Cases zusammenf�hren f�r weniger zeilen code!! case1:case2:case3: Code break;
				quarterjump(); //Wenn man zu oft hier landet kann man beim Offset noch +1 dazurechnen
				break;
			case quarterjump2:
				quarterjump();
				break;
			case quarterjump3:
				quarterjump();
				break;
			case halfjump1:
				halfjump();
				break;
			case halfjump2:				
				halfjump();
				break;
			case halfjump3:
				halfjump();
				break;
			case threequartersjump1:
				threequartersjump();
				break;
			case threequartersjump2:				
				threequartersjump();
				break;
			case threequartersjump3:
				threequartersjump();
				break;
			case fulljump1:
				fulljump();
				break;
			case fulljump2:				
				fulljump();
				break;
			case fulljump3:
				fulljump();
				break;
			case onequarterjump1:
				onequarterjump();
				break;
			case onequarterjump2:
				onequarterjump();
				break;
			case onequarterjump3:
				onequarterjump();
				break;
			case onehalfjump1:
				onehalfjump();
				break;
			case onehalfjump2:
				onehalfjump();
				break;
			case onehalfjump3:
				onehalfjump();
				break;
			case onethreequartersjump1:
				onethreequartersjump();
				break;
			case onethreequartersjump2:
				onethreequartersjump();
				break;
			case onethreequartersjump3:
				onethreequartersjump();
				break;
			default:
				//Code f�r Resett einbauen
				break;
		}
		k++;
		switch(k){
			case 62: //Wert noch anpassen working 60
				k = 0;
				
				for (int i = 0; i < 62; i++)
				{
					receivebuffer[i] = 0;
				}
// 				receivebuffer[0] = 0;
// 				receivebuffer[1] = 0;
// 				receivebuffer[2] = 0;
// 				receivebuffer[3] = 0;
				
				debug++;
				//Code f�r neuen Start
				break;
// 			case 56:
// 				vTaskDelay(2/portTICK_RATE_MS);
// 				break;
		}	
		
}

void vQuamDec(void* pvParameters)
{
	( void ) pvParameters;
	decoderQueue = xQueueCreate( 4, NR_OF_SAMPLES * sizeof(int16_t) );
	
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	
	uint16_t bufferelement[NR_OF_SAMPLES];
	
	xEventGroupWaitBits(evDMAState, DMADECREADY, false, true, portMAX_DELAY);
	for(;;) {
		while(uxQueueMessagesWaiting(decoderQueue) > 0) { // Nur arbeiten wenn in der Queue auch Werte drin sind
			if(xQueueReceive(decoderQueue, &bufferelement[0], portMAX_DELAY) == pdTRUE) {
				for (int i = 0; i < 32; i++) // Die Werte von der Queue werden in das Ringbuffer geschrieben
				{
					*p_Writing = bufferelement[i];
					p_Writing++;
					Ringbuffer_Pos++;
					//Differenz Wert
					
				}
				if (((p_Writing - p_Reading)%32) == 0) //�berpr�fen �b == sinnvoll ist eher >=. Mehr als 32 Wert vorraus
				{			//Letztes Byte als Z�hl Variabel nutzen 				
					for (int i = 0; i < 32; i++)
					{
						if ((*p_Reading > *(p_Reading-1)) && *p_Reading > 1300) //Werte m�ssen angepasst werden mit den Werten von Merlin
						{
							max = *p_Reading; //Bei der 3.ten Nachricht will das Programm hier nicht mehr reingehen weil die Pointeradressen nicht mehr sauber durch 32 teilbar sind 
							*p_MAXPOS2r = j;
						}
						p_Reading++;
						j++;
					}
					Offset = *p_MAXPOS2r - *p_MAXPOS1r;
					*p_MAXPOS1r = *p_MAXPOS2r;
					p_Reading = p_Writing; //Sinnvoll? Decodieren vlt Separieren von schreiben in den Ringbuffer | vlt Neue Task erstellen?
				}
				if(Ringbuffer_Pos == 256) //% Operator | Bits maskieren!? Ringbuffer 0/256 == 0 
				{
					Ringbuffer_Pos = 0;
					j = 0;
					p_Writing = &ringbuffer[0]; //Der Code wurde Seriallisiert. Die Tasks 
					p_Reading = &ringbuffer[0];
					/*debug++;*/
				}
				switch(maxpos[0]){
					case 0:
					break;
					default:
					analyzediff();
					break;
				}
			} //Klammer While
		} //Klammer For
		vTaskDelay( 2 / portTICK_RATE_MS );
	}
}

void fillDecoderQueue(uint16_t buffer[NR_OF_SAMPLES])
{
	BaseType_t xTaskWokenByReceive = pdFALSE;

	xQueueSendFromISR( decoderQueue, &buffer[0], &xTaskWokenByReceive );
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;

	fillDecoderQueue( &adcBuffer0[0] );
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB |= 0x10;

	fillDecoderQueue( &adcBuffer1[0] );
}