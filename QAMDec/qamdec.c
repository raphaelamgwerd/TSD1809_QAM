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

#include "qamdec.h"

#define NR_OF_DECODER_SAMPLES         32UL

#define DECODER_FREQUENCY_INITIAL_VALUE		1000UL * NR_OF_DECODER_SAMPLES


uint16_t adcBuffer0[NR_OF_DECODER_SAMPLES];
uint16_t adcBuffer1[NR_OF_DECODER_SAMPLES];

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
	DMA.CH2.TRFCNT = NR_OF_DECODER_SAMPLES*2;
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
	DMA.CH3.TRFCNT = NR_OF_DECODER_SAMPLES*2;
	DMA.CH3.SRCADDR0 = ((uint16_t)(&ADCA.CH0.RES) >> 0) & 0xFF;
	DMA.CH3.SRCADDR1 = ((uint16_t)(&ADCA.CH0.RES) >> 8) & 0xFF;
	DMA.CH3.SRCADDR2 = 0x00;
	DMA.CH3.DESTADDR0 = ((uint16_t)(&adcBuffer1[0]) >> 0) & 0xFF;
	DMA.CH3.DESTADDR1 = ((uint16_t)(&adcBuffer1[0]) >> 8) & 0xFF;
	DMA.CH3.DESTADDR2 = 0x00;

	DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
	DMA.CH3.CTRLA |= DMA_CH_ENABLE_bm;
}

void vQuamDec(void* pvParameters) {
	decoderQueue = xQueueCreate(4, NR_OF_DECODER_SAMPLES*sizeof(int16_t));
	
	initDecDMA();
	initADC();
	initADCTimer();
	for(;;) {
		
		vTaskDelay(2/portTICK_RATE_MS);
	}
}

void fillDecoderQueue(uint16_t buffer[NR_OF_DECODER_SAMPLES]) {
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