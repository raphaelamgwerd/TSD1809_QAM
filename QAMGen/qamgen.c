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

#define AMPLITUDE_1 0x01
#define AMPLITUDE_2 0x02
#define DATEN_AUFBEREITET 0x04
#define NR_OF_DATA_SAMPLES 32UL

#define NR_OF_GENERATOR_SAMPLES					32UL
#define GENERATOR_FREQUENCY_INITIAL_VALUE		1000UL

/* Flags to show QAM channel ready for new amplitude level. */
#define QAMCHANNEL_1_READY      0x01
#define QAMCHANNEL_2_READY      0x02

/* Defines which bits are representing the amount of data bytes to be sent. */
#define DATABYTETOSENDMASK      0x1F

void vsendCommand(uint8_t Data[]);
void vsendFrame(void *pvParameters);

typedef enum
{
    Idle,
    sendSyncByte,
    sendDatenbuffer,
    sendChecksum
} eProtokollStates;

TaskHandle_t xsendFrame;

EventGroupHandle_t xQAMchannel_1;
EventGroupHandle_t xQAMchannel_2;

QueueHandle_t xQueue_Data;

const int16_t sinLookup1000[NR_OF_GENERATOR_SAMPLES] = {   0,   80,  157,  228,  290,  341,  378,  402,
                                                         410,  402,  378,  341,  290,  228,  157,   80,
                                                           0,  -80, -157, -228, -290, -341, -378, -402, 
                                                        -410, -402, -378, -341, -290, -228, -157,  -80
                                                         };
                                                         
const int16_t sinLookup2000[NR_OF_GENERATOR_SAMPLES] = { 0,  157,  290,  378,  410,  378,  290,  157,
                                                         0, -157, -290, -378, -410, -378, -290, -157,
                                                         0,  157,  290,  378,  410,  378,  290,  157,    
                                                         0, -157, -290, -378, -410, -378, -290, -157
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
    
       xQAMchannel_1=xEventGroupCreate();
       xQAMchannel_2=xEventGroupCreate();
       xQueue_Data = xQueueCreate( NR_OF_DATA_SAMPLES, sizeof( uint8_t ) );
    
        xTaskCreate(vsendFrame, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, &xsendFrame);
	
	for(;;) {
		vTaskDelay(10/portTICK_RATE_MS);
	}
}

void fillBuffer(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
    
    uint16_t Amp=1;
    uint8_t  EventGroupBits= xEventGroupGetBitsFromISR(xQAMchannel_1);
    if (EventGroupBits&AMPLITUDE_1)
    {
        Amp=1;
    }
    
    else if (EventGroupBits&AMPLITUDE_2)
    {
        Amp=2;
    }
	for(int i = 0; i < NR_OF_GENERATOR_SAMPLES;i++) {
		buffer[i] = 0x800 + (Amp*sinLookup1000[i]);
	}
        xEventGroupSetBits(xQAMchannel_1,DATEN_AUFBEREITET);
}

// Mit ISR EventGroups arbeiten, da der Interrupt Buffer füllt
void fillBuffer_1(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
    uint8_t Amp_1=1;
    uint8_t  EventGroupBits= xEventGroupGetBitsFromISR(xQAMchannel_2);
    
    if (EventGroupBits&AMPLITUDE_1)
    {
        Amp_1=1;
    }

    else if (EventGroupBits&AMPLITUDE_2)
    {
        Amp_1=2;
    }
            
    for(int i = 0; i < NR_OF_GENERATOR_SAMPLES;i++) {
       buffer[i] = 0x800 + (Amp_1*sinLookup2000[i]); 
    }
       xEventGroupSetBits(xQAMchannel_2,DATEN_AUFBEREITET);
   
}

ISR(DMA_CH0_vect)
{
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

void vsendCommand(uint8_t Data[])
{

    if( xQueue_Data != 0 )
    {
        xQueueSend(xQueue_Data, (void *)&Data,pdMS_TO_TICKS(10));
    }    
}

void vsendFrame(void *pvParameters)
{
(void) pvParameters;
    
uint8_t ucSendByteValue;                    // Variable current byte to send is stored
uint8_t ucSendBitPackageCounter = 0;        // Counts the sent bit packages, for QAM4 for 1 byte there are 4 packages (4 x 2bit = 8bit)
uint8_t ucReadyForNewDataByte = 1;          // Indicates if new data byte can be provided.
uint8_t ucNewDataByteValue = 0;             // Stores the new data byte value, which should be sent next.
uint8_t ucDataByteCounter = 0;              // Counts the sent data of the data array.
uint8_t ucDataBytesToSend;                  // Stores the value of how many data bytes should be sent. Extracted from command byte (1st array in data queue).
uint8_t ucQAMChannelsReady = 0;             // Status register to check if both QAM channels got the new amplitude levels.
uint8_t ucIdleSendByteFlag = 0;             // Byte for differentiation of idle bytes to be sent between 0xAF and 0x05.
uint8_t ucChecksumValue = 0;                // Used for checksum calculation.
uint8_t Data[NR_OF_DATA_SAMPLES + 1] = {};  // Data bytes received from queue.                

    
    eProtokollStates Protokoll = Idle;
    
    while(1)
    {
        
        /************************************************************************/
        /* State machine for setting up data.                                   */
        /************************************************************************/
        switch(Protokoll)
        {
            case Idle:
            {
                if (ucIdleSendByteFlag == 0)
                {
                    /* Send first idle byte. */
                    if (ucReadyForNewDataByte)
                    {
                        ucIdleSendByteFlag = 1;
                        ucNewDataByteValue = 0xAF;
                        ucReadyForNewDataByte = 0;
                    }
                }
                else
                {
                    /* Send second idle byte. */
                    if (ucReadyForNewDataByte)
                    {
                        ucIdleSendByteFlag = 0;
                        ucNewDataByteValue = 0x05;
                        ucReadyForNewDataByte = 0;
                        
                        
                        /* Check if new Data was received. */
                        if (xQueueReceive(xQueue_Data, (void*)&Data, pdMS_TO_TICKS(0)) == pdTRUE)
                        {
                            ucDataBytesToSend = Data[0] & DATABYTETOSENDMASK;
                            Protokoll = sendSyncByte;
                        }                                
                    }
                }
                break;
            }
            
            case sendSyncByte:
            {
                if (ucReadyForNewDataByte)
                {
                    ucNewDataByteValue = 0xFF;
                    ucReadyForNewDataByte = 0;
                    Protokoll = sendDatenbuffer;
                }
                
                break;
            }                          
            
            case sendDatenbuffer:
            {  
                if (ucReadyForNewDataByte)
                {
                    if (ucDataByteCounter < ucDataBytesToSend)
                    {
                        ucNewDataByteValue = Data[ucDataByteCounter];
                        
                        /* Continuous checksum calculation over all sent data bytes. */
                        ucChecksumValue ^= ucNewDataByteValue;
                        ucReadyForNewDataByte = 0;
                        ucDataByteCounter++;
                    } 
                    else
                    {
                        ucDataByteCounter = 0;
                        Protokoll = sendChecksum;
                    }
                }

                break;
            }
            case sendChecksum:
            {
                if (ucReadyForNewDataByte)
                {
                    ucNewDataByteValue = ucChecksumValue;
                    ucChecksumValue = 0;
                    ucReadyForNewDataByte = 0;
                    Protokoll = Idle;
                }
                break;
            }
            default:
            {
                Protokoll = Idle;
                break;
            }

        }
        
        
        /************************************************************************/
        /* Data send part.                                                      */
        /************************************************************************/
        
        /* Check if all data bit packages of one byte were sent. */
        if (ucSendBitPackageCounter > 3)
        {
            /* Then the new data byte can be loaded. */
            ucSendByteValue = ucNewDataByteValue;
            ucSendBitPackageCounter = 0;
            ucReadyForNewDataByte = 1;
        }
        
        /* Check if QAM channel 1 got amplitude value. */
        if (xEventGroupGetBits(xQAMchannel_1) & DATEN_AUFBEREITET)
        {
            /* Then flag can be deleted an status can be stored temporarily. */
            xEventGroupClearBits(xQAMchannel_1, DATEN_AUFBEREITET);
            ucQAMChannelsReady |= QAMCHANNEL_1_READY;
            
            /* New amplitude level for next transmission can be prepared. */
            if (ucSendByteValue & 0b00000001)
            {
                xEventGroupSetBits(xQAMchannel_1, AMPLITUDE_2);
                xEventGroupClearBits(xQAMchannel_1, AMPLITUDE_1);
            } 
            else
            {
                xEventGroupSetBits(xQAMchannel_1, AMPLITUDE_1);
                xEventGroupClearBits(xQAMchannel_1, AMPLITUDE_2);
            }
        }
        
        /* Check if QAM channel 2 got amplitude value. */
        if (xEventGroupGetBits(xQAMchannel_2)&DATEN_AUFBEREITET)
        {
            /* Then flag can be deleted an status can be stored temporarily. */
            xEventGroupClearBits(xQAMchannel_2,DATEN_AUFBEREITET);
            ucQAMChannelsReady |= QAMCHANNEL_2_READY;
            
            /* New amplitude level for next transmission can be prepared. */
            if (ucSendByteValue & 0b00000010)
            {
                xEventGroupSetBits(xQAMchannel_2, AMPLITUDE_2);
                xEventGroupClearBits(xQAMchannel_2, AMPLITUDE_1);
            }
            else
            {
                xEventGroupSetBits(xQAMchannel_2, AMPLITUDE_1);
                xEventGroupClearBits(xQAMchannel_2, AMPLITUDE_2);
            }
        }        
        
        /* Check if both channels got the new amplitude level. */
        if((ucQAMChannelsReady & QAMCHANNEL_1_READY) && (ucQAMChannelsReady & QAMCHANNEL_2_READY))
        {
            /* Then new bit package can be prepared. */
            ucQAMChannelsReady = 0;
            ucSendBitPackageCounter++;
            ucSendByteValue = ucSendByteValue >> 2;
        }
        
    }
}    