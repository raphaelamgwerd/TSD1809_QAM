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

#include "qamdec.h"


extern void vApplicationIdleHook( void );
void vControl(void* pvParameters);

volatile TaskHandle_t xQAMdecHandle;
volatile TaskHandle_t xProtocolTaskHandle;

void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	xTaskCreate(vQAMDec, (const char *) "QAMDecoder", configMINIMAL_STACK_SIZE+800, NULL, 3, NULL/*&xQAMdecHandle*/);
    xTaskCreate(vControl, NULL, configMINIMAL_STACK_SIZE+000, NULL, 2, NULL);
	
	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"QAM 4 Decoder  R: %d", reason);
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
            if (ucCommand != 0)
            {
                vDisplayWriteStringAtPos(1, 0, "Command:   %d ", ucCommand);
                vDisplayWriteStringAtPos(2, 0, "DataBytes: %d ", ucReceivedDataBytes);
                for (uint8_t ucDataPrintCounter = 0; ucDataPrintCounter < ucReceivedDataBytes; ++ucDataPrintCounter)
                {
                    if (ucDataPrintCounter < ucReceivedDataBytes)
                    {
                        vDisplayWriteStringAtPos(3, ucDataPrintCounter * 2, "%x", ucDataArray[ucDataPrintCounter]);
                    }
                }
                ucCommand = 0;
                ucReceivedDataBytes = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }   
}    