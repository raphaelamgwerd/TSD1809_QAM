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


#ifndef __OPTIMIZE__
# warning "Compiler optimizations disabled; functions from qamdec.c won't work as designed"
#endif

#define COMPLETESAMPLECOUNT             4
#define DECODERSAMPLECOUNT              32UL
#define MEDIANSAMPLECOMPARECOUNT        10 //DECODERSAMPLECOUNT / 4

#define DATABYTEQUEUELENGTH             1  // Bytes which can be stored in received data bytes queue.

#define TRANSITIONNOTFOUND              0 // indicates that no new complete signal was found
#define TRANSITIONFOUND                 1 // indicates that a new complete signal was found

#define QAMLEVELS                       4       // QAM4 is implemented.
#define QAMPACKAGESPERBYTE              4       // 4 bit packages of 2 bits for one byte.

/* Protocol defines */
#define SYNCHRONISATIONBYTE             0xFF
#define CALIBRATIONSIGNAL               0xAF05
#define COMMANDMASK                     0xE0
#define COMMANDBITPOSITION              5
#define DATABYTESMASK                   0x1F
#define MAXRECEIVEDATABYTES             32
#define PROTOCOLSTOSTORE                1   /* can store 16 received messages without getting them */
#define COMMANDBYTEPOSITION             0   /* position in protocol queue */
#define DATABYTECOUNTPOSITION           1
#define DATABYTESTARTPOSITION           2


#define DECODER_FREQUENCY_INITIAL_VALUE		1000UL * DECODERSAMPLECOUNT

/* Declaration of state machine */
typedef enum
{
    Idle,
    Command,
    Data,
    Checksum
} eProtocolDecoderStates;


uint16_t ucLongTimeMaxValue = 750;  // This value will be adjusted to the absolute maximum value of QAM Level 3.
uint16_t ucAbsoulteMaxValue = 1438; // This value is the maximum value of compare array of QAM Level 3. (see below)
int16_t sDataReference0[DECODERSAMPLECOUNT] = {   0,  237,  447,  606,  700,  719,  668,  559,  410,  245,   88,  -37, -120, -150, -133,  -77,    0,   77,  133,  150,  120,   37,  -88, -245, -410, -559, -668, -719, -700, -606, -447, -237};
int16_t sDataReference1[DECODERSAMPLECOUNT] = {   0,  317,  603,  833,  989, 1059, 1047,  960,  819,  646,  467,  303,  169,   77,   23,    3,    0,   -3,  -23,  -77, -169, -303, -467, -646, -819, -960, -1047, -1059, -989, -833, -603, -317};
int16_t sDataReference2[DECODERSAMPLECOUNT] = {   0,  393,  736,  985, 1109, 1098,  957,  715,  410,   89, -201, -416, -529, -529, -422, -233,    0,  233,  422,  529,  529,  416,  201,  -89, -410, -715, -957, -1098, -1109, -985, -736, -393};
int16_t sDataReference3[DECODERSAMPLECOUNT] = {   0,  473,  892, 1212, 1398, 1438, 1336, 1116,  819,  490,  178,  -76, -240, -302, -266, -153,    0,  153,  266,  302,  240,   76, -178, -490, -819, -1116, -1336, -1438, -1398, -1212, -892, -473};

uint16_t adcBuffer0[DECODERSAMPLECOUNT];
uint16_t adcBuffer1[DECODERSAMPLECOUNT];

uint8_t ucQAMDataBytes[MAXRECEIVEDATABYTES + 2] = {};

QueueHandle_t decoderQueue;
volatile QueueHandle_t receivedByteQueue;
volatile QueueHandle_t receivedProtocolQueue;
EventGroupHandle_t receivedProtocolEventGroup;

void initADC(void) {
	ADCA.CTRLA = 0x01;
	ADCA.CTRLB = 0x00;
	ADCA.REFCTRL = 0x20;  //INTVCC = External reference from AREF pin on PORT A (Vcc)
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



uint8_t bGetReceivedData(uint16_t usReceiveArray[], uint8_t* ucActualArrayPos, uint8_t* ucLastTransEnd, uint16_t* usSignalOffsetLevel, uint8_t* ucQAMValue)
{
uint8_t ucDataCounter = 0;
uint8_t ucTransitionFound = TRANSITIONNOTFOUND;
uint16_t usLowerHalfSignalOffset = 0;
uint16_t usUpperHalfSignalOffset = 0;
uint8_t ucMaxValuePos;
uint8_t ucMinValuePos;
uint8_t ucMinMaxPosDifference;
uint16_t ucMaxValue;
uint16_t ucMinValue;
uint16_t ucMinMaxDifference;
volatile int16_t sSignalDifference[4];
float fScaleFactor;

    
    ucDataCounter = (*ucLastTransEnd > 5) ? *ucLastTransEnd - 5 : 0;
    
    do {
        if (usReceiveArray[ucDataCounter + 1] > *usSignalOffsetLevel)
        {
            if (usReceiveArray[ucDataCounter] <= *usSignalOffsetLevel)
            {
                ucMaxValuePos = 0;
                ucMinValuePos = 0;
                ucMaxValue = 0;
                ucMinValue = 0;
                for (uint8_t ucPosCounter = 0; ucPosCounter < DECODERSAMPLECOUNT; ++ucPosCounter)
                {
                    /* Add all values in two variables to prevent overflow. Needed for mean value calculation. */
                    if (ucPosCounter < (DECODERSAMPLECOUNT / 2))
                    {
                        usLowerHalfSignalOffset += usReceiveArray[ucDataCounter + ucPosCounter];
                    }
                    else
                    {
                        usUpperHalfSignalOffset += usReceiveArray[ucDataCounter + ucPosCounter];
                    }
                    
                    /* Get the max and the min values and their positions of the sampled data. */
                    if (usReceiveArray[ucDataCounter + ucPosCounter] > usReceiveArray[ucDataCounter + ucMaxValuePos])
                    {
                        ucMaxValuePos = ucPosCounter;
                        ucMaxValue = usReceiveArray[ucDataCounter + ucPosCounter];
                    }
                    if (usReceiveArray[ucDataCounter + ucPosCounter] < usReceiveArray[ucDataCounter + ucMinValuePos])
                    {
                        ucMinValuePos = ucPosCounter;
                        ucMinValue = usReceiveArray[ucDataCounter + ucPosCounter];
                    }
                }
                
                /* Calculate the new reference offset value. */
                usLowerHalfSignalOffset /= (DECODERSAMPLECOUNT / 2);
                usUpperHalfSignalOffset /= (DECODERSAMPLECOUNT / 2);
                *usSignalOffsetLevel = ((*usSignalOffsetLevel * 5) + ((usLowerHalfSignalOffset + usUpperHalfSignalOffset) / 2)) / 6;
    
                if (ucMaxValuePos < ucMinValuePos)
                {
                    ucMinMaxPosDifference = (ucMinValuePos + ucMaxValuePos) / 2;
                    if ((ucMinMaxPosDifference >= 12) && (ucMinMaxPosDifference <= 18))
                    {
                        ucMinMaxDifference = (ucMinValue + ucMaxValue) / 2;
                        if ((ucMinMaxDifference >= (*usSignalOffsetLevel - 500)) && (ucMinMaxDifference <= (*usSignalOffsetLevel + 500)))
                        {
                            /* Calculate scale factor to adjust the initial reference array to the actual waveform scale. */
                            fScaleFactor = (float)ucAbsoulteMaxValue / (float)ucLongTimeMaxValue;
                            
                            sSignalDifference[0] = 0;
                            sSignalDifference[1] = 0;
                            sSignalDifference[2] = 0;
                            sSignalDifference[3] = 0;
                            int16_t sActualPositionDifference;
                            for (uint8_t ucPosCounter = 0; ucPosCounter < DECODERSAMPLECOUNT; ++ucPosCounter)
                            {
                                /* Calculate the difference of each waveform over the full sampling period. */
                                sActualPositionDifference = (int16_t)usReceiveArray[ucDataCounter + ucPosCounter] - *usSignalOffsetLevel;
                                sSignalDifference[0] = sSignalDifference[0] + abs((int16_t)((float)(sActualPositionDifference) * fScaleFactor) - sDataReference0[ucPosCounter]);
                                sSignalDifference[1] = sSignalDifference[1] + abs((int16_t)((float)(sActualPositionDifference) * fScaleFactor) - sDataReference1[ucPosCounter]);
                                sSignalDifference[2] = sSignalDifference[2] + abs((int16_t)((float)(sActualPositionDifference) * fScaleFactor) - sDataReference2[ucPosCounter]);
                                sSignalDifference[3] = sSignalDifference[3] + abs((int16_t)((float)(sActualPositionDifference) * fScaleFactor) - sDataReference3[ucPosCounter]);
                            }
                            
                            /* Search the waveform with the smallest difference to the actual signal. */
                            *ucQAMValue = 0;
                            for (uint8_t ucQAMLevelCounter = 0; ucQAMLevelCounter < (QAMLEVELS - 1); ++ucQAMLevelCounter)
                            {
                                *ucQAMValue = (abs(sSignalDifference[ucQAMLevelCounter + 1]) < abs(sSignalDifference[*ucQAMValue])) ? ucQAMLevelCounter + 1 : *ucQAMValue;
                            }
                            ucTransitionFound = TRANSITIONFOUND;
                            
                            /* If it's a QAM level 3, then adjust the absolute maximum signal value. (For scale factor calculation needed) */
                            if (*ucQAMValue == 3)
                            {
                                ucLongTimeMaxValue = ((ucLongTimeMaxValue * 5) + ((int16_t)ucMaxValue - *usSignalOffsetLevel)) / 6;
                            }
                        }                        
                    }
                }
                
                
            }
        }
        ++ucDataCounter;
        /* Repeat as long as no transition (start of signal = sine-transition) was found
           or no complete signal could be found. */
    } while ((ucTransitionFound == TRANSITIONNOTFOUND) && ((ucDataCounter - 1) < (*ucActualArrayPos)));
    
    --ucDataCounter; // correct position of found transition
    
    
    if (ucTransitionFound == TRANSITIONFOUND)
    {
        /* Copy array values. Extract received data and modify receive data buffer. */
        
        /* Set new reference position in receive array. */
        if (*ucActualArrayPos <= (6 * DECODERSAMPLECOUNT))
        {
            *ucActualArrayPos += DECODERSAMPLECOUNT;
            *ucLastTransEnd = ucDataCounter + DECODERSAMPLECOUNT;
        }
        else
        {
            uint8_t ucArrayDifference = *ucActualArrayPos - ucDataCounter;
            for (uint8_t ucArrayCopyCounter = 0; ucArrayCopyCounter < (ucArrayDifference + 4); ++ucArrayCopyCounter)
            {
                usReceiveArray[ucArrayCopyCounter] = usReceiveArray[ucDataCounter + DECODERSAMPLECOUNT - 4 + ucArrayCopyCounter];
            }
            *ucActualArrayPos = ucArrayDifference + 4;
            *ucLastTransEnd = 0;
        }
    }
    else
    { // If no complete signal was detected, increase new reference position in array. */
        if (*ucActualArrayPos <= (6 * DECODERSAMPLECOUNT))
        {
            *ucActualArrayPos += DECODERSAMPLECOUNT;
        }
        else
        {
            *ucActualArrayPos = 0;
            *ucLastTransEnd = 0;
        }
    }
    return ucTransitionFound;
}

uint8_t ucQAMGetData(uint8_t* ucCommand, uint8_t* ucDataBytes, uint8_t ucDataArray[])
{
//uint8_t ucQueueBytes[MAXRECEIVEDATABYTES + 2] = {};
uint8_t ucReturnValue = pdFALSE;
uint8_t queueByte = 0;


    //receivedProtocolQueue = xQueueCreate(PROTOCOLSTOSTORE, sizeof(uint8_t) * 3);//(MAXRECEIVEDATABYTES + 2));
    queueByte = uxQueueMessagesWaiting(receivedProtocolQueue);
    if(queueByte)
    {
    if (xEventGroupClearBits(receivedProtocolEventGroup, 0x01) & 0x01)
        //xQueueReceive(receivedProtocolQueue, &ucQueueBytes, pdMS_TO_TICKS(1));
        *ucCommand = ucQAMDataBytes[COMMANDBYTEPOSITION];
        *ucDataBytes = ucQAMDataBytes[DATABYTECOUNTPOSITION];
        for (uint8_t ucDataByteCounter = 0; ucDataByteCounter < *ucDataBytes; ++ucDataByteCounter)
        {
            ucDataArray[ucDataByteCounter] = ucQAMDataBytes[DATABYTESTARTPOSITION + ucDataByteCounter];
        }
        ucReturnValue = pdTRUE;
    }
    return ucReturnValue;
}

void xProtocolDecoder(void* pvParameters) {
eProtocolDecoderStates eProtocolDecoder = Idle;
uint8_t ucReceivedByte;
volatile uint8_t ucDataBytesReceived;
volatile uint8_t ucDataBytesCounter = 0;
uint8_t ucCommandByte;
uint8_t ucChecksumCalculated;
//uint8_t ucQueueBytes[MAXRECEIVEDATABYTES + 2] = {};


    /*if (receivedProtocolQueue == NULL)
    {
        receivedProtocolQueue = xQueueCreate(PROTOCOLSTOSTORE, sizeof(uint8_t) * 4);//(MAXRECEIVEDATABYTES + 2));
    }*/
    receivedProtocolEventGroup = xEventGroupCreate();
    
    while(1)
    {
        
        if(uxQueueMessagesWaiting(receivedByteQueue))
        {
            xQueueReceive(receivedByteQueue, &ucReceivedByte, pdMS_TO_TICKS(1));
            switch (eProtocolDecoder)
            {
                case Idle:
                {
                    if (ucReceivedByte == SYNCHRONISATIONBYTE)
                    {
                        eProtocolDecoder = Command;
                    }
                    break;
                }
                case Command:
                {
                    ucCommandByte = (ucReceivedByte & COMMANDMASK) >> COMMANDBITPOSITION;
                    ucQAMDataBytes[COMMANDBYTEPOSITION] = ucCommandByte;
                    ucDataBytesReceived = ucReceivedByte & DATABYTESMASK;
                    ucQAMDataBytes[DATABYTECOUNTPOSITION] = ucDataBytesReceived;
                    
                    ucChecksumCalculated = ucReceivedByte; 
                    if (ucDataBytesReceived > 0)
                    {
                        eProtocolDecoder = Data;
                    }
                    else
                    {
                        eProtocolDecoder = Checksum;
                    }
                    break;
                }
                case Data:
                {
                    ucQAMDataBytes[DATABYTESTARTPOSITION + ucDataBytesCounter] = ucReceivedByte;
                    ucChecksumCalculated ^= ucReceivedByte;
                    if (++ucDataBytesCounter >= ucDataBytesReceived)
                    {
                        eProtocolDecoder = Checksum;
                    }
                    break;
                }
                case Checksum:
                {
                    if (ucReceivedByte == ucChecksumCalculated)
                    { // If the calculated checksum matches the received checksum.
                        //xQueueSend(receivedProtocolQueue, (void *)ucQueueBytes, pdMS_TO_TICKS(0));
                        xEventGroupSetBits(receivedProtocolEventGroup, 0x01);
                    }
                    ucDataBytesCounter = 0;
                    eProtocolDecoder = Idle;
                    break;
                }
                default:
                {
                    eProtocolDecoder = Idle;
                    break;
                }
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void vQAMDec(void* pvParameters) {
uint16_t usReceiveArray[8 * DECODERSAMPLECOUNT] = {};
uint8_t ucArrayReference = 0;           // reference state of array
uint8_t ucLastReceiveEnd = 0;

uint8_t ucActualQAMValue;

uint16_t usQAMCalibTriggerValue = 0;


uint8_t ucDataByte = 0;                     // received data byte
uint8_t ucReceivedBitPackageCounter = 0;    // Counts amount of bit packages.

uint16_t usSignalOffsetLevel = 2000;       // Offset Voltage (Must be in a guilty range for beginning)
    
	decoderQueue = xQueueCreate(1, DECODERSAMPLECOUNT*sizeof(int16_t));
    receivedByteQueue = xQueueCreate(DATABYTEQUEUELENGTH, sizeof(uint8_t));
    receivedProtocolQueue = xQueueCreate(PROTOCOLSTOSTORE, sizeof(uint8_t) * 4);//(MAXRECEIVEDATABYTES + 2));
	
	initDecDMA();
	initADC();
	initADCTimer();
    //xTaskCreate(xProtocolDecoder, NULL, configMINIMAL_STACK_SIZE+400, NULL, 2, NULL);
    
	for(;;) {
        
		if (uxQueueMessagesWaiting(decoderQueue))
		{
    		xQueueReceive(decoderQueue, &usReceiveArray[ucArrayReference], pdMS_TO_TICKS(0));
            if(bGetReceivedData(&usReceiveArray[0], &ucArrayReference, &ucLastReceiveEnd, &usSignalOffsetLevel, &ucActualQAMValue) == TRANSITIONFOUND)
            { /* If data signal was received. */
                
                
                ucDataByte = (ucDataByte >> 2) | (ucActualQAMValue << 6);      // fill data byte by left shift (LSB protocol)
                usQAMCalibTriggerValue = (usQAMCalibTriggerValue >> 2) | (ucActualQAMValue << 14);
                
                
                if (++ucReceivedBitPackageCounter >= QAMPACKAGESPERBYTE)
                {
                    ucReceivedBitPackageCounter = 0;
                    xQueueSend(receivedByteQueue, (void*)&ucDataByte, pdMS_TO_TICKS(0));
                    ucDataByte = 0;
                }
                
                if (usQAMCalibTriggerValue == CALIBRATIONSIGNAL)
                {
                    /* Calibration signal received. Define this to the start of a byte. */
                    usQAMCalibTriggerValue = 0;
                    ucReceivedBitPackageCounter = 0;
                }
                
            }         
		}
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
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