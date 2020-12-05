/*
 * QAMGen.c
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
#include "ButtonHandler.h"

#define BUTTON1SHORTPRESSEDMASK     0x01
#define BUTTON2SHORTPRESSEDMASK     0x02
#define BUTTON3SHORTPRESSEDMASK     0x04
#define BUTTON4SHORTPRESSEDMASK     0x08
#define BUTTON1LONGPRESSEDMASK      0x10
#define BUTTON2LONGPRESSEDMASK      0x20
#define BUTTON3LONGPRESSEDMASK      0x40
#define BUTTON4LONGPRESSEDMASK      0x80



extern void vApplicationIdleHook( void );


void vSteuertask(void *pvParameters);
void vButtonTask(void *pvParameters);


void vApplicationIdleHook( void )
{	
	
}

TaskHandle_t xSteuertask;
TaskHandle_t xButtonTaskHandle;


int main(void)//Hauptprogramm
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
    
 

	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+500, NULL, 2, NULL);
    xTaskCreate(vSteuertask, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, &xSteuertask);
    xTaskCreate(vButtonTask, (const char *) "ButtonTask", configMINIMAL_STACK_SIZE, NULL, 2, &xButtonTaskHandle);

    

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	vDisplayWriteStringAtPos(2,0,"QAMGEN-Base");
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskStartScheduler();
	return 0;
}

void vSteuertask(void *pvParameters)
{
    (void) pvParameters;
    uint32_t Buttonvalue;
    char DataString[33];
    
    while(1)
    {
        //if (Buttonvalue&BUTTON1SHORTPRESSEDMASK)
        //{
            DataString[0] = 0x83;   // Command + Amount of Data 0bXXXY'YYYY
            DataString[1] = 0xAB;
            DataString[2] = 0x37;
            DataString[3] = 0x85;
            vsendCommand(DataString);
            vTaskDelay(500/portTICK_RATE_MS);
       // }
    
    }    
    
}

void vButtonTask(void *pvParameters) {
    initButtons();

    for(;;) {
        updateButtons();
        
        if(getButtonPress(BUTTON1) == SHORT_PRESSED) {
            
            xTaskNotify(vSteuertask,BUTTON1SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
            
        }
        if(getButtonPress(BUTTON2) == SHORT_PRESSED) {
            
            xTaskNotify(vSteuertask,BUTTON2SHORTPRESSEDMASK,eSetValueWithOverwrite);
        }
        if(getButtonPress(BUTTON3) == SHORT_PRESSED) {
            
            xTaskNotify(vSteuertask,BUTTON3SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
        }
        if(getButtonPress(BUTTON4) == SHORT_PRESSED) {
            
            xTaskNotify(vSteuertask,BUTTON4SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
        }

        vTaskDelay((1000/BUTTON_UPDATE_FREQUENCY_HZ)/portTICK_RATE_MS);
    }

}


    

