/*
 * QAMDec.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : chaos
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

#include "qamgen.h"
#include "qamdec.h"


extern void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	xTaskCreate(vQuamDec, NULL, configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
	//xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
	
	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	vDisplayWriteStringAtPos(2,0,"QAMDEC-Base");
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskStartScheduler();
	return 0;
}


void vControl(void* pvParameters) {
uint8_t ucCommand;
uint8_t ucReceivedDataBytes;
uint8_t ucDataArray[32] = {};

    while(1)
    {
        if(ucQAMGetData(&ucCommand, &ucReceivedDataBytes, ucDataArray) == pdTRUE)
        {
            vDisplayWriteStringAtPos(1, 0, "Command:   %d", ucCommand);
            vDisplayWriteStringAtPos(2, 0, "DataBytes: %d", ucReceivedDataBytes);
            for (uint8_t ucDataPrintCounter = 0; ucDataPrintCounter < ucReceivedDataBytes; ++ucDataPrintCounter)
            {
                vDisplayWriteStringAtPos(3, ucDataPrintCounter * 3, "%x", ucDataArray[ucDataPrintCounter]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }   
}    