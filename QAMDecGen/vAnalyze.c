/*
 * vAnalyze.c
 *
 * Created: 05.12.2023 16:47:15
 *  Author: mikaj
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
#include "NHD0420Driver.h"

#include "qaminit.h"
#include "qamdec.h"
#include "string.h"
#include "main.h"
#include "Defines.h"


/*Array Init*/
unsigned char byteArray[4];

/*Semaphore Init*/
SemaphoreHandle_t xMutex = NULL;

/*Var Init*/
uint8_t receivebuffer[50];
uint8_t k = 0;
uint8_t checksumGL = 0; // Initialisierung der Checksumme
uint8_t calculatedChecksum = 0; // Variable fuer die berechnete Checksumme
float reconstructedFloat = 0;
uint8_t debug = 0;
unsigned char CSArray[4];


//////////////////////////////////////////////////////////////////////////
/*						Jumps fuer die Synchronisations					*/
//////////////////////////////////////////////////////////////////////////

uint8_t quarterjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	receivebuffer[k] = 0;
	return newnumber;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
}

uint8_t halfjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
	
}

uint8_t threequartersjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
	
}

uint8_t fulljump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
}

uint8_t onequarterjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
}

uint8_t onehalfjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
}

uint8_t onethreequartersjump(uint8_t lastnumber, uint8_t k){
	uint8_t newnumber = 0;
	switch(lastnumber){
		case 0:
		receivebuffer[k] = 3;
		newnumber = 3;
		return newnumber;
		break;
		case 1:
		receivebuffer[k] = 0;
		newnumber = 0;
		return newnumber;
		break;
		case 2:
		receivebuffer[k] = 1;
		newnumber = 1;
		return newnumber;
		break;
		case 3:
		receivebuffer[k] = 2;
		newnumber = 2;
		return newnumber;
		break;
		default:
		return 4;
		break;
	}
}

uint8_t analyzediff(int16_t Pos, int16_t nextpos, uint8_t lastnumber, uint8_t k);



//////////////////////////////////////////////////////////////////////////
/*			Funktion für das Suchen der Peaks im Ringpuffer				*/
//////////////////////////////////////////////////////////////////////////

int16_t getNextHighPos(uint32_t Pos){
	int16_t syncpos = -1;

	for (int i = 0; i < 60; ++i)
	{
		Pos = Pos + 4;
		if ((ringbuffer[Pos & BitMask] > 2000)) {	//2000 für Kabelgebunden / Für Drahtlos 500!
			syncpos = (Pos & BitMask);
			return syncpos;
		}
		if (ringbuffer[Pos & BitMask] < 50)	{
			vTaskDelay(1/portTICK_RATE_MS);
		}
		
	}
	if (syncpos != -1)
	{
		return syncpos;
	}else{
		return -1;
	}
}



//////////////////////////////////////////////////////////////////////////
/*			Funktion für das extrahieren der Temperaturdaten			*/
//////////////////////////////////////////////////////////////////////////

void getDataTemp(void) {
	uint8_t datalenght = 4;
	static int pReceivebuffer = 12;
	static int byteArrayIndex = 0;

	for (int i = 0; i < 16; i++) {
		switch (receivebuffer[pReceivebuffer]) {
			case 0:
			byteArray[byteArrayIndex] |= (0b000 << (i % 4) * 2);
			break;
			case 1:
			byteArray[byteArrayIndex] |= (0b001 << (i % 4) * 2);
			break;
			case 2:
			byteArray[byteArrayIndex] |= (0b010 << (i % 4) * 2);
			break;
			case 3:
			byteArray[byteArrayIndex] |= (0b11 << (i % 4) * 2); 
			break;
		}
		pReceivebuffer++;

		// Wenn viermal ins byteArray[0] geschrieben wurde, gehe zum nächsten Element
		if ((i + 1) % datalenght == 0) {
			byteArrayIndex++;
		}
	}

	// Zurücksetzen von pReceivebuffer und byteArrayIndex, wenn es am Ende des receivebuffer angelangt ist
	if (pReceivebuffer >= 28) {
		pReceivebuffer = 12;
		byteArrayIndex = 0;
	}
}



//////////////////////////////////////////////////////////////////////////
/*				Funktion für das extrahieren der Checksumme				*/
//////////////////////////////////////////////////////////////////////////

void getChecksum(void) {
	static int pReceivebuffer = 28;
	static int CSArrayIndex = 0;
	memcpy(&CSArray, 0, sizeof(uint8_t));	//Array leeren

	for (int i = 0; i < 4; i++) {
		switch (receivebuffer[pReceivebuffer]) {
			case 0:
			CSArray[CSArrayIndex] |= (0b000 << (i % 4) * 2);
			break;
			case 1:
			CSArray[CSArrayIndex] |= (0b001 << (i % 4) * 2);
			break;
			case 2:
			CSArray[CSArrayIndex] |= (0b010 << (i % 4) * 2);
			break;
			case 3:
			CSArray[CSArrayIndex] |= (0b11 << (i % 4) * 2);
			break;
		}
		pReceivebuffer++;
	}
	memcpy(&checksumGL, CSArray, sizeof(uint8_t));
	// Zurücksetzen von pReceivebuffer und byteArrayIndex, wenn es am Ende des receivebuffer angelangt ist
	if (pReceivebuffer >=32) {
		pReceivebuffer = 28;
		CSArrayIndex = 0;
	}
}



//////////////////////////////////////////////////////////////////////////
/*				Funktion für das Protokoll-Handling						*/
//////////////////////////////////////////////////////////////////////////

void vAnalyze(void *pvParameters){
	uint32_t read_pos = 0;
	int16_t pos = 0;
	int16_t nextpos = 0;
	uint8_t currentnumber = 4;
	uint8_t lastnumber = 0;
	uint8_t protocolmode = 0;
	uint8_t RX_Pos = 0;
	uint8_t symbol = 0;
	xMutex = xSemaphoreCreateMutex();
	(void) pvParameters;
	
	for (;;) {
		xSemaphoreTake(xMutex, portMAX_DELAY);	//Mutex für die Variabel write_pos!
		if (((write_pos) - (read_pos)) >= 70 ) {
			xSemaphoreGive(xMutex);
			pos = getNextHighPos(read_pos);
			nextpos = getNextHighPos(pos);
			currentnumber = analyzediff(pos, nextpos, lastnumber, RX_Pos);
			RX_Pos++;
			lastnumber = currentnumber;
			if (nextpos == -1) {			// Falls wir kein neuen Peak finden konnten!
				read_pos = pos-4;
			}
			else {
				read_pos = nextpos-4;		// Damit wir beim nächsten Durchlauf den letzten Peak wieder finden können
			}
			switch(protocolmode){			// Hier wird unser Protocolhandling betrieben
				case Idle0:					// Check ob das Startmuster stimmt
					if (currentnumber == 3) {
						protocolmode = Idle1;
					}
					break;
					
				case Idle1:
					if (currentnumber == 0) {
						protocolmode = Idle2;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case Idle2:
					if (currentnumber == 3) {
						protocolmode = Idle3;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case Idle3:
					if (currentnumber == 0) {
						protocolmode = Idle4;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case Idle4:
					if (currentnumber == 3) {
						protocolmode = Idle5;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case Idle5:
					if (currentnumber == 2) {
						protocolmode = type;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case type:
					if (currentnumber == 0) {
						protocolmode = sync;
					}
					else {
						protocolmode = Idle0;						
					}
					break;
					
				case sync:
					if (currentnumber == 1)	{		// Ab diesem Zeitpunkt sind wir uns Sicher dass wir den Start von einem Datenpaket gefunden haben.
						RX_Pos =8;
						debug = 1;
						receivebuffer[0] = 3;
						receivebuffer[1] = 0;
						receivebuffer[2] = 3;
						receivebuffer[3] = 0;
						receivebuffer[4] = 3;
						receivebuffer[5] = 2;
						receivebuffer[6] = 0;
						receivebuffer[7] = 1;
						protocolmode = Data;
					}
					else {
						protocolmode = Idle0;
					}
					break;
					
				case Data:
					if(RX_Pos == 32){
						protocolmode = checksum;
					}
					else {
						break;					
					}
					break;
					
				case checksum:		//Berechnet die Soll-Checksumme der Empfangenen Daten
					calculatedChecksum = 0;
					for (size_t i = 0; i < (NR_OF_SAMPLES-4); i++) {
						calculatedChecksum += receivebuffer[i];
					}
					getChecksum();	//Extrahiert die gesendete Checksumme
					
					if (checksumGL == calculatedChecksum) {
						protocolmode = FINAL;
						getDataTemp();
						memcpy(&reconstructedFloat, byteArray, sizeof(float));
						for (int i = 0; i < 4 ; i++) {
							byteArray[i] = 0;
						}
						
					}
					else {
						RX_Pos = 0;
						protocolmode = Idle0;
					}
					break;
					
				case FINAL:
						protocolmode = Idle0;
						RX_Pos = 0;
						break;
			}
				
		}
		else{
			xSemaphoreGive(xMutex);
		}
	vTaskDelay(1/portTICK_RATE_MS);
	}
}



//////////////////////////////////////////////////////////////////////////
/*		Funktion für die Analyse der Differenz von zwei Peaks			*/
//////////////////////////////////////////////////////////////////////////

uint8_t analyzediff(int16_t Pos, int16_t nextpos, uint8_t number, uint8_t rxpos){
	uint8_t Offset = 0;
	
	uint8_t newnumber = 4;
	if (nextpos == -1) {
		Offset = 8;
	}
	else {
		Offset = nextpos - Pos;
	}
	switch(Offset){
		case quarterjump1: 
		newnumber = quarterjump(number, rxpos); 
		break;
		case quarterjump2:
		newnumber = quarterjump(number, rxpos);
		break;
		case quarterjump3:
		newnumber = quarterjump(number, rxpos);
		break;
		case quarterjump4:
		newnumber = quarterjump(number, rxpos);
		break;
		case quarterjump5:
		newnumber = quarterjump(number, rxpos);
		break;
		case quarterjump6:
		newnumber = quarterjump(number, rxpos);
		break;
		case halfjump0:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump1:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump2:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump3:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump4:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump5:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump6:
		newnumber = halfjump(number, rxpos);
		break;
		case halfjump7:
		newnumber = halfjump(number, rxpos);
		break;
		case threequartersjump0:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump1:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump2:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump3:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump4:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump5:
		newnumber = threequartersjump(number, rxpos);
		break;
		case threequartersjump6:
		newnumber = threequartersjump(number, rxpos);
		break;
		case fulljump0:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump1:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump2:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump3:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump4:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump5:
		newnumber = fulljump(number, rxpos);
		break;
		case fulljump6:
		newnumber = fulljump(number, rxpos);
		break;
		case onequarterjump1:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump0:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump2:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump3:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump4:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump5:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump6:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onequarterjump7:
		newnumber = onequarterjump(number, rxpos);
		break;
		case onehalfjump0:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump1:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump2:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump3:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump4:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump5:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump6:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onehalfjump7:
		newnumber = onehalfjump(number, rxpos);
		break;
		case onethreequartersjump0:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump1:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump2:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump3:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump4:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump5:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		case onethreequartersjump6:
		newnumber = onethreequartersjump(number, rxpos);
		break;
		default:
		break;
	}
	return newnumber;
}



//////////////////////////////////////////////////////////////////////////	
/*							Diplay-Task									*/
//////////////////////////////////////////////////////////////////////////

void vDisplay(void* pvParameters){
	for (;;) {
		vDisplayClear();
		vDisplayWriteStringAtPos(0,0, "%f 'C", reconstructedFloat);
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}