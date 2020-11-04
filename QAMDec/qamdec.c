/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
*  Author: Chaos
*/ 

#include <stdlib.h>

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

#include "qamdec.h"

#define COMPLETESAMPLECOUNT             4
#define DECODERSAMPLECOUNT              32UL

#define QAMLEVELS                       4       // QAM4 is implemented.

#define DECODER_FREQUENCY_INITIAL_VALUE		1000UL * DECODERSAMPLECOUNT

uint16_t usQAMHalfMedianLevels[QAMLEVELS] = {123, 256, 781, 1236};

uint16_t adcBuffer0[DECODERSAMPLECOUNT];
uint16_t adcBuffer1[DECODERSAMPLECOUNT];

QueueHandle_t decoderQueue;

void initADC(void) {
	ADCA.CTRLA = 0x01;
	ADCA.CTRLB = 0x00;
	ADCA.REFCTRL = 0x10; //INTVCC = 2V as Reference
	ADCA.PRESCALER = 0x03;
	ADCA.EVCTRL = 0x39; //Event Channel 7 triggers Channel 0 Conversion
	ADCA.CH0.CTRL = 0x01; //Singleended positive input without gain
	ADCA.CH0.MUXCTRL = 0x55; //Input = ADC10 on ADCA = Pin PB2 = DAC-Output
	ADCA.CH0.INTCTRL = 0x00;
}

void initADCTimer(void) {
	TC1_ConfigClockSource(&TCD1, TC_CLKSEL_DIV1_gc);
	TC1_ConfigWGM(&TCD1, TC_WGMODE_SINGLESLOPE_gc);
	TC_SetPeriod(&TCD1, 32000000/(DECODER_FREQUENCY_INITIAL_VALUE));
	EVSYS.CH7MUX = EVSYS_CHMUX_TCD1_OVF_gc;
}

void initDecDMA(void) {
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);

	DMA.CTRL = DMA_ENABLE_bm | DMA_DBUFMODE_CH23_gc;

	DMA.CH2.REPCNT = 0;
	DMA.CH2.CTRLB |= 0x01;
	DMA.CH2.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
	DMA.CH2.ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
	DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_ADCA_CH0_gc;
	DMA.CH2.TRFCNT = DECODERSAMPLECOUNT*2;
	DMA.CH2.SRCADDR0 = ((uint16_t)(&ADCA.CH0.RES) >> 0) & 0xFF;
	DMA.CH2.SRCADDR1 = ((uint16_t)(&ADCA.CH0.RES) >> 8) & 0xFF;
	DMA.CH2.SRCADDR2 = 0x00;
	DMA.CH2.DESTADDR0 = ((uint16_t)(&adcBuffer0[0]) >> 0) & 0xFF;
	DMA.CH2.DESTADDR1 = ((uint16_t)(&adcBuffer0[0]) >> 8) & 0xFF;
	DMA.CH2.DESTADDR2 = 0x00;

	DMA.CH3.REPCNT = 0;
	DMA.CH3.CTRLB |= 0x01;
	DMA.CH3.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
	DMA.CH3.ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
	DMA.CH3.TRIGSRC = DMA_CH_TRIGSRC_ADCA_CH0_gc;
	DMA.CH3.TRFCNT = DECODERSAMPLECOUNT*2;
	DMA.CH3.SRCADDR0 = ((uint16_t)(&ADCA.CH0.RES) >> 0) & 0xFF;
	DMA.CH3.SRCADDR1 = ((uint16_t)(&ADCA.CH0.RES) >> 8) & 0xFF;
	DMA.CH3.SRCADDR2 = 0x00;
	DMA.CH3.DESTADDR0 = ((uint16_t)(&adcBuffer1[0]) >> 0) & 0xFF;
	DMA.CH3.DESTADDR1 = ((uint16_t)(&adcBuffer1[0]) >> 8) & 0xFF;
	DMA.CH3.DESTADDR2 = 0x00;

	DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
	DMA.CH3.CTRLA |= DMA_CH_ENABLE_bm;
}



int sCompare (const void * a, const void * b)
{
    return ( *(int*)a - *(int*)b );
}

uint16_t usMedian(uint16_t usArray[], uint8_t ucElements)
{
uint16_t usReturnValue;

    /* First sort the array. */
    qsort(usArray, ucElements, sizeof(uint16_t), sCompare);
    
    
    /* Check for even case. */
    if (ucElements % 2 != 0)
    {
        usReturnValue = usArray[ucElements / 2];
    }
    else
    {
        usReturnValue = (usArray[(ucElements - 1) / 2] + usArray[ucElements / 2]) / 2;
    }
    
    return usReturnValue;
}

uint8_t ucAllocateValue(uint16_t usMedianValue)
{
uint8_t ucQAMLevel = 0;
uint16_t usMedianDifference;
uint16_t usCompareValue = 0xFFFF;
    
    for(uint8_t ucQAMLevelCounter; ucQAMLevelCounter < QAMLEVELS; ++ucQAMLevelCounter)
    {
        /* Calculate difference of median value and reference value of actual iteration. */
        usMedianDifference = abs(usMedianValue - usQAMHalfMedianLevels[ucQAMLevelCounter]);
        
        /* If newly calculated difference is smaller than previous difference. */
        if (usMedianDifference < usCompareValue)
        {
            usCompareValue = usMedianDifference;
            ucQAMLevel = ucQAMLevelCounter;
        }
    }
    return ucQAMLevel;
}

void vQuamDec(void* pvParameters) {
uint16_t usReceiveArray[2 * DECODERSAMPLECOUNT] = {};
uint8_t ucArrayReference = 0;           // reference state of array
    
	decoderQueue = xQueueCreate(COMPLETESAMPLECOUNT, DECODERSAMPLECOUNT*sizeof(int16_t));
	
	initDecDMA();
	initADC();
	initADCTimer();
	for(;;) {
		if (!uxQueueMessagesWaiting(decoderQueue))
		{
    		xQueueReceive(decoderQueue, &usReceiveArray[ucArrayReference], pdMS_TO_TICKS(0));
            ucAllocateValue(usMedian(usReceiveArray, DECODERSAMPLECOUNT / 2));
            /* Find new reference value and copy new array */
		}
        else
        {
            vTaskDelay(2/portTICK_RATE_MS);
        }
	}
}

void fillDecoderQueue(uint16_t buffer[DECODERSAMPLECOUNT]) {
	BaseType_t xTaskWokenByReceive = pdFALSE;
	xQueueSendFromISR(decoderQueue, &buffer[0], &xTaskWokenByReceive);
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;
	fillDecoderQueue(&adcBuffer0[0]);
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB|=0x10;
	fillDecoderQueue(&adcBuffer1[0]);
}