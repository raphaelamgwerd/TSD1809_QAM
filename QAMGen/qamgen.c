/*
 * qamgen.c
 *
 * Created: 05.05.2020 16:24:59
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
#include "qamgen.h"

#define NR_OF_GENERATOR_SAMPLES					32UL
#define GENERATOR_FREQUENCY_INITIAL_VALUE		1000UL

const int16_t sinLookup100[NR_OF_GENERATOR_SAMPLES] = {  0x800,0x0850,0x089d,0x08e4,0x0922,0x0955,0x097a,0x0992,
                                                         0x099a,0x0992,0x097a,0x0955,0x0922,0x08e4,0x089d,0x0850,
                                                         0x0800,0x07b0,0x0763,0x071c,0x06de,0x06ab,0x0686,0x066e,
                                                         0x0666,0x066e,0x0686,0x06ab,0x06de,0x071c,0x0763,0x07b0
                                                         };
                                                         
const int16_t sinLookup200[NR_OF_GENERATOR_SAMPLES] = {  0x800,0x0850,0x089d,0x08e4,0x0922,0x0955,0x097a,0x0992,
                                                         0x099a,0x0992,0x097a,0x0955,0x0922,0x08e4,0x089d,0x0850,
                                                         0x0800,0x07b0,0x0763,0x071c,0x06de,0x06ab,0x0686,0x066e,
                                                         0x0666,0x066e,0x0686,0x06ab,0x06de,0x071c,0x0763,0x07b0
                                                         };

static uint16_t dacBuffer0[NR_OF_GENERATOR_SAMPLES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint16_t dacBuffer1[NR_OF_GENERATOR_SAMPLES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                        
static uint16_t dacBuffer2[NR_OF_GENERATOR_SAMPLES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint16_t dacBuffer3[NR_OF_GENERATOR_SAMPLES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void initDAC(void) {
	DACB.CTRLA = DAC_CH0EN_bm | DAC_CH1EN_bm;
	DACB.CTRLB = DAC_CHSEL1_bm | DAC_CH0TRIG_bm | DAC_CH1TRIG_bm;
	DACB.CTRLC = DAC_REFSEL0_bm; // Reference Voltage = AVCC
	DACB.EVCTRL = 0x00;
	DACB.CTRLA |= DAC_ENABLE_bm;
	PORTB.DIRSET = 0x04;
    PORTB.DIRSET = 0x08;
}
    
void initGenDMA(void) {
	DMA.CTRL = 0;
	DMA.CTRL = DMA_RESET_bm;
	while ((DMA.CTRL & DMA_RESET_bm) != 0);

	DMA.CTRL = DMA_ENABLE_bm | DMA_DBUFMODE_CH01CH23_gc;

	DMA.CH0.REPCNT = 0;
	DMA.CH0.CTRLB|=0x01;
	DMA.CH0.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;					// ADC result is 2 byte 12 bit word
	DMA.CH0.ADDRCTRL =	DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_INC_gc;	// reload source after every burst, reload dest after every transaction
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_DACB_CH0_gc;
	DMA.CH0.TRFCNT = NR_OF_GENERATOR_SAMPLES*2;	// always the number of bytes, even if burst length > 1
	DMA.CH0.SRCADDR0 = ((uint16_t)(&dacBuffer0[0]) >> 0) & 0xFF;
	DMA.CH0.SRCADDR1 = ((uint16_t)(&dacBuffer0[0]) >>  8) & 0xFF;
	DMA.CH0.SRCADDR2 =0;
	DMA.CH0.DESTADDR0 = ((uint16_t)(&DACB.CH0DATA) >> 0) & 0xFF;
	DMA.CH0.DESTADDR1 = ((uint16_t)(&DACB.CH0DATA) >> 8) & 0xFF;
	DMA.CH0.DESTADDR2 = 0;

	DMA.CH1.REPCNT = 0;
	DMA.CH1.CTRLB |= 0x01;
	DMA.CH1.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_INC_gc;
	DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_DACB_CH0_gc;
	DMA.CH1.TRFCNT = NR_OF_GENERATOR_SAMPLES*2;
	DMA.CH1.SRCADDR0 = ((uint16_t)(&dacBuffer1[0]) >> 0) & 0xFF;
	DMA.CH1.SRCADDR1 = ((uint16_t)(&dacBuffer1[0]) >>  8) & 0xFF;
	DMA.CH1.SRCADDR2 =0;
	DMA.CH1.DESTADDR0 = ((uint16_t)(&DACB.CH0DATA) >> 0) & 0xFF;
	DMA.CH1.DESTADDR1 = ((uint16_t)(&DACB.CH0DATA) >> 8) & 0xFF;
	DMA.CH1.DESTADDR2 = 0;
    
    DMA.CH2.REPCNT = 0;
    DMA.CH2.CTRLB|=0x01;
    DMA.CH2.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;					// ADC result is 2 byte 12 bit word
    DMA.CH2.ADDRCTRL =	DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_INC_gc;	// reload source after every burst, reload dest after every transaction
    DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_DACB_CH0_gc;
    DMA.CH2.TRFCNT = NR_OF_GENERATOR_SAMPLES*2;	// always the number of bytes, even if burst length > 1
    DMA.CH2.SRCADDR0 = ((uint16_t)(&dacBuffer2[0]) >> 0) & 0xFF;
    DMA.CH2.SRCADDR1 = ((uint16_t)(&dacBuffer2[0]) >>  8) & 0xFF;
    DMA.CH2.SRCADDR2 =0;
    DMA.CH2.DESTADDR0 = ((uint16_t)(&DACB.CH1DATA) >> 0) & 0xFF;
    DMA.CH2.DESTADDR1 = ((uint16_t)(&DACB.CH1DATA) >> 8) & 0xFF;
    DMA.CH2.DESTADDR2 = 0;
    
    DMA.CH3.REPCNT = 0;
    DMA.CH3.CTRLB |= 0x01;
    DMA.CH3.CTRLA = DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
    DMA.CH3.ADDRCTRL = DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_INC_gc;
    DMA.CH3.TRIGSRC = DMA_CH_TRIGSRC_DACB_CH0_gc;
    DMA.CH3.TRFCNT = NR_OF_GENERATOR_SAMPLES*2;
    DMA.CH3.SRCADDR0 = ((uint16_t)(&dacBuffer3[0]) >> 0) & 0xFF;
    DMA.CH3.SRCADDR1 = ((uint16_t)(&dacBuffer3[0]) >>  8) & 0xFF;
    DMA.CH3.SRCADDR2 =0;
    DMA.CH3.DESTADDR0 = ((uint16_t)(&DACB.CH1DATA) >> 0) & 0xFF;
    DMA.CH3.DESTADDR1 = ((uint16_t)(&DACB.CH1DATA) >> 8) & 0xFF;
    DMA.CH3.DESTADDR2 = 0;
	
	DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;
	DMA.CH1.CTRLA |= DMA_CH_ENABLE_bm; 
    DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
    DMA.CH3.CTRLA |= DMA_CH_ENABLE_bm;// later, not yet tested!
}
void initDACTimer(void) {
	TC0_ConfigClockSource(&TCD0, TC_CLKSEL_DIV1_gc);
	TC0_ConfigWGM(&TCD0, TC_WGMODE_SINGLESLOPE_gc);
	TC_SetPeriod(&TCD0, 32000000/(GENERATOR_FREQUENCY_INITIAL_VALUE*NR_OF_GENERATOR_SAMPLES));
	EVSYS.CH0MUX = EVSYS_CHMUX_TCD0_OVF_gc; //Setup Eventsystem when timer TCD0 overflows
}

void vQuamGen(void *pvParameters) {
	initDAC();
	initDACTimer();
	initGenDMA();
	
	for(;;) {
		vTaskDelay(10/portTICK_RATE_MS);
	}
}

void fillBuffer(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
	for(int i = 0; i < NR_OF_GENERATOR_SAMPLES;i++) {
		buffer[i] = (sinLookup100[i]);
	}
}

void fillBuffer_1(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
    for(int i = 0; i < NR_OF_GENERATOR_SAMPLES;i++) {
       buffer[i] = (sinLookup200[i]);
    }
}

ISR(DMA_CH0_vect)
{
	//static signed BaseType_t test;
	
	DMA.CH0.CTRLB|=0x10;
	fillBuffer(&dacBuffer0[0]);
}

ISR(DMA_CH1_vect)
{
	DMA.CH1.CTRLB|=0x10;
	fillBuffer(&dacBuffer1[0]);
}

ISR(DMA_CH2_vect)
{
    
    DMA.CH2.CTRLB|=0x10;
    fillBuffer_1(&dacBuffer2[0]);
}

ISR(DMA_CH3_vect)
{
    DMA.CH3.CTRLB|=0x10;
    fillBuffer_1(&dacBuffer3[0]);
}