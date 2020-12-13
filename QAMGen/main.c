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

typedef enum
{
	smalldata,
	middledata,
	bigdata,
	writedata,
} eSteuerungStates;




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
    
 

	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
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
	
	eSteuerungStates Steuerung = smalldata;
    
    while(1)
    {
        
        xTaskNotifyWait(0, 0xffffffff, &Buttonvalue, pdMS_TO_TICKS(200));
        
		switch(Steuerung)
		{
			
		
		
			case smalldata:
			{
				if (Buttonvalue&BUTTON2SHORTPRESSEDMASK)
				{
					Steuerung=middledata;
				}
			
				if (Buttonvalue&BUTTON3SHORTPRESSEDMASK)
				{
					Steuerung=bigdata;
				}
			
				if (Buttonvalue&BUTTON1SHORTPRESSEDMASK)
				{
					DataString[0] = 0x83;   // Command + Amount of Data 0bXXXY'YYYY
					DataString[1] = 0xAB;
					DataString[2] = 0x37;
					DataString[3] = 0x85;
				
				    Steuerung = writedata;
				}
			break;	
			}
		
			case middledata:
			{
				if (Buttonvalue&BUTTON2SHORTPRESSEDMASK)
				{
                    DataString[0] = 0x2F;
					DataString[1] = 0x57;
					DataString[2] = 0x6F;
					DataString[3] = 0x2D;
					DataString[4] = 0x62;
					DataString[5] = 0x69;
					DataString[6] = 0x6E;
					DataString[7] = 0x2D;
					DataString[8] = 0x69;
					DataString[9] = 0x63;
					DataString[10] = 0x68;
					DataString[11] = 0x2D;
					DataString[12] = 0x68;
					DataString[13] = 0x69;
					DataString[14] = 0x65;
					DataString[15] = 0x72;
					Steuerung = writedata;
				}
				break;
			}
		
			case bigdata:
			{
				if (Buttonvalue&BUTTON3SHORTPRESSEDMASK)
				{
    				DataString[0] = 0x5F;
					DataString[1] = 0x4D;
					DataString[2] = 0x65;
					DataString[3] = 0x69;
					DataString[4] = 0x6E;
					DataString[5] = 0x2D;
					DataString[6] = 0x4E;
					DataString[7] = 0x61;
					DataString[8] = 0x6D;
					DataString[9] = 0x65;
					DataString[10] = 0x2D;
					DataString[11] = 0x69;
					DataString[12] = 0x73;
					DataString[13] = 0x74;
					DataString[14] = 0x2D;
					DataString[15] = 0x51;
					DataString[16] = 0x41;
					DataString[17] = 0x4D;
					DataString[18] = 0x2D;
					DataString[19] = 0x56;
					DataString[20] = 0x65;
					DataString[21] = 0x72;
					DataString[22] = 0x73;
					DataString[23] = 0x69;
					DataString[24] = 0x6F;
					DataString[25] = 0x6E;
					DataString[26] = 0x31;
					DataString[27] = 0x2E;
					DataString[28] = 0x30;
					DataString[29] = 0x2E;
					DataString[30] = 0x30;
					DataString[31] = 0x2D;
				
					Steuerung = writedata;
				
				
				}
				break;
			}
		
			case writedata:
			{
				vsendCommand(DataString);
				Steuerung = smalldata;
				break;
			}
		}
    
    }    
    
}

void vButtonTask(void *pvParameters) {
    initButtons();

    for(;;) {
        updateButtons();
        
        if(getButtonPress(BUTTON1) == SHORT_PRESSED) {
            
            xTaskNotify(xSteuertask,BUTTON1SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
            
        }
        if(getButtonPress(BUTTON2) == SHORT_PRESSED) {
            
            xTaskNotify(xSteuertask,BUTTON2SHORTPRESSEDMASK,eSetValueWithOverwrite);
        }
        if(getButtonPress(BUTTON3) == SHORT_PRESSED) {
            
            xTaskNotify(xSteuertask,BUTTON3SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
        }
        if(getButtonPress(BUTTON4) == SHORT_PRESSED) {
            
            xTaskNotify(xSteuertask,BUTTON4SHORTPRESSEDMASK,eSetValueWithOverwrite);
            
        }

        vTaskDelay((1000/BUTTON_UPDATE_FREQUENCY_HZ)/portTICK_RATE_MS);
    }

}


    

