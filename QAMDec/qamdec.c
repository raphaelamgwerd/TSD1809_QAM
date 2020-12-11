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
#define MEDIANSAMPLECOMPARECOUNT        DECODERSAMPLECOUNT / 4

#define DATABYTEQUEUELENGTH             50  // Bytes which can be stored in received data bytes queue.

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

uint16_t usQAMHalfMedianLevels[QAMLEVELS] = {2760, 2870, 2850, 2930};//{2631, 2895, 2945, 3212}; // TODO rank order: High freq / Low freq: LowVolt/LowVolt, LowVolt/HighVolt, HighVolt/LowVolt, High/High

uint16_t adcBuffer0[DECODERSAMPLECOUNT];
uint16_t adcBuffer1[DECODERSAMPLECOUNT];

QueueHandle_t decoderQueue;
QueueHandle_t receivedByteQueue;
QueueHandle_t receivedProtocolQueue;

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



int sCompare (const void* ValueOne, const void* ValueTwo)
{
    return ( *(int*)ValueOne - *(int*)ValueTwo );
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
    
    /* Check for each QAM level, which median level is nearest. */
    for(uint8_t ucQAMLevelCounter = 0; ucQAMLevelCounter < QAMLEVELS; ++ucQAMLevelCounter)
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

void vOffsetLevelAdjust(uint16_t usReceivedValueArray[], uint16_t* usSignalOffsetLevel)
{
uint16_t usTempValueArray[DECODERSAMPLECOUNT] = {};  // Array to copy received data
uint16_t usTempSignalOffset;

    for (uint8_t ucArrayCopyCounter = 0; ucArrayCopyCounter < DECODERSAMPLECOUNT; ++ucArrayCopyCounter)
    {
        usTempValueArray[ucArrayCopyCounter] = usReceivedValueArray[ucArrayCopyCounter];
    }
    usTempSignalOffset = usMedian(usTempValueArray, DECODERSAMPLECOUNT);
    *usSignalOffsetLevel = ((*usSignalOffsetLevel * 5) + usTempSignalOffset) / 6;
}

uint8_t bGetReceivedData(uint16_t usReceiveArray[], uint8_t* ucActualArrayPos, uint16_t* usSignalOffsetLevel, uint16_t usReceivedValueArray[])
{
uint8_t ucDataCounter = 0;
uint8_t ucTransitionFound = TRANSITIONNOTFOUND;

    /* Find new reference value and copy new array */
    vOffsetLevelAdjust(&usReceiveArray[*ucActualArrayPos], usSignalOffsetLevel);
    
    ucDataCounter = (*ucActualArrayPos >= DECODERSAMPLECOUNT) ? (*ucActualArrayPos - DECODERSAMPLECOUNT) : 0;
    
    do {
        if (usReceiveArray[ucDataCounter + 1] > *usSignalOffsetLevel)
        {
            if (usReceiveArray[ucDataCounter] <= *usSignalOffsetLevel)
            {
                if (usReceiveArray[ucDataCounter + 4] > usReceiveArray[ucDataCounter + 4 + (DECODERSAMPLECOUNT / 2)])
                {
                    ucTransitionFound = TRANSITIONFOUND;
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
        for (uint8_t ucArrayCopyCounter = 0; ucArrayCopyCounter < DECODERSAMPLECOUNT; ++ucArrayCopyCounter)
        {
            usReceivedValueArray[ucArrayCopyCounter] = usReceiveArray[ucDataCounter + ucArrayCopyCounter];
        }
        if (*ucActualArrayPos <= (5 * DECODERSAMPLECOUNT))
        {
            *ucActualArrayPos += DECODERSAMPLECOUNT;
        }
        else
        {
            uint8_t ucArrayDifference = *ucActualArrayPos - ucDataCounter;
            for (uint8_t ucArrayCopyCounter = 0; ucArrayCopyCounter < ucArrayDifference; ++ucArrayCopyCounter)
            {
                usReceiveArray[ucArrayCopyCounter] = usReceiveArray[ucDataCounter + DECODERSAMPLECOUNT - 4 + ucArrayCopyCounter];
            }
            *ucActualArrayPos = ucArrayDifference;
        }
        /* Set new reference position in receive array. */
        /*if (ucDataCounter == 0)
        {
            *ucActualArrayPos = 0;
        }
        else
        {
            *ucActualArrayPos -= ucDataCounter;
        }*/
    }
    else
    { // If no complete signal was detected, increase new reference position in array. */
        *ucActualArrayPos = (*ucActualArrayPos <= (5 * DECODERSAMPLECOUNT)) ? *ucActualArrayPos + DECODERSAMPLECOUNT : 0;
    }
    return ucTransitionFound;
}

uint8_t ucQAMGetData(uint8_t* ucCommand, uint8_t* ucDataBytes, uint8_t ucDataArray[])
{
uint8_t ucQueueBytes[MAXRECEIVEDATABYTES + 2] = {};
uint8_t ucReturnValue = pdFALSE;

    if (xQueueReceive(receivedProtocolQueue, &ucQueueBytes, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        *ucCommand = ucQueueBytes[COMMANDBYTEPOSITION];
        *ucDataBytes = ucQueueBytes[DATABYTECOUNTPOSITION];
        for (uint8_t ucDataByteCounter = 0; ucDataByteCounter < *ucDataBytes; ++ucDataByteCounter)
        {
            ucDataArray[ucDataByteCounter] = ucQueueBytes[DATABYTESTARTPOSITION + ucDataByteCounter];
        }
        ucReturnValue = pdTRUE;
    }
    return ucReturnValue;
}

void xProtocolDecoder(void* pvParameters) {
eProtocolDecoderStates eProtocolDecoder = Idle;
uint8_t ucReceivedByte;
uint8_t ucDataBytesReceived;
uint8_t ucDataBytesCounter = 0;
uint8_t ucCommandByte;
uint8_t ucChecksumCalculated;
uint8_t ucQueueBytes[MAXRECEIVEDATABYTES + 2] = {};

    receivedProtocolQueue = xQueueCreate(PROTOCOLSTOSTORE, sizeof(uint8_t) * (MAXRECEIVEDATABYTES + 2));

    while(1)
    {
        
        if(xQueueReceive(receivedByteQueue, &ucReceivedByte, pdMS_TO_TICKS(1)) == pdTRUE)
        {
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
                    ucQueueBytes[COMMANDBYTEPOSITION] = ucCommandByte;
                    ucDataBytesReceived = ucReceivedByte & DATABYTESMASK;
                    ucQueueBytes[DATABYTECOUNTPOSITION] = ucDataBytesReceived;
                    
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
                    ucQueueBytes[DATABYTESTARTPOSITION + ucDataBytesCounter] = ucReceivedByte;
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
                        xQueueSend(receivedProtocolQueue, (void*)&ucQueueBytes, pdMS_TO_TICKS(0));
                        ucDataBytesCounter = 0;
                        eProtocolDecoder = Idle;
                    }
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

void vQuamDec(void* pvParameters) {
uint16_t usReceiveArray[7 * DECODERSAMPLECOUNT] = {};
uint8_t ucArrayReference = 0;           // reference state of array
uint16_t usReceivedValueArray[DECODERSAMPLECOUNT] = {};  // current received data

uint16_t usMedianCompareValue;
uint8_t ucActualQAMValue;

uint8_t ucQAMLevelCalibArrayCounter = 0;
uint16_t usQAMLevelsForCalibration[16] = {};
uint16_t usQAMCalibTriggerValue = 0;
uint8_t ucByteswoCalibration = 0;

uint8_t ucDataByte = 0;                     // received data byte
uint8_t ucReceivedBitPackageCounter = 0;    // Counts amount of bit packages.

uint16_t usSignalOffsetLevel = 2500;       // Offset Voltage
    
	decoderQueue = xQueueCreate(1, DECODERSAMPLECOUNT*sizeof(int16_t));
    receivedByteQueue = xQueueCreate(DATABYTEQUEUELENGTH, sizeof(uint8_t));
	
	initDecDMA();
	initADC();
	initADCTimer();
    xTaskCreate(xProtocolDecoder, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    
	for(;;) {
		if (uxQueueMessagesWaiting(decoderQueue))
		{
    		xQueueReceive(decoderQueue, &usReceiveArray[ucArrayReference], pdMS_TO_TICKS(0));
            if(bGetReceivedData(&usReceiveArray[0], &ucArrayReference, &usSignalOffsetLevel, &usReceivedValueArray[0]) == TRANSITIONFOUND)
            { /* If data signal was received. */
                
                usMedianCompareValue = usMedian(usReceivedValueArray, MEDIANSAMPLECOMPARECOUNT);
                ucActualQAMValue = ucAllocateValue(usMedianCompareValue);
                
                ucDataByte = (ucDataByte >> 2) | (ucActualQAMValue << 6);      // fill data byte by left shift (LSB protocol)
                usQAMCalibTriggerValue = (usQAMCalibTriggerValue >> 2) | (ucActualQAMValue << 14);
                usQAMLevelsForCalibration[ucQAMLevelCalibArrayCounter] = usMedianCompareValue;
                ucQAMLevelCalibArrayCounter = (ucQAMLevelCalibArrayCounter < 15) ? ucQAMLevelCalibArrayCounter + 1 : 0; 
                
                if (++ucReceivedBitPackageCounter >= QAMPACKAGESPERBYTE)
                {
                    ucReceivedBitPackageCounter = 0;
                    xQueueSend(receivedByteQueue, (void*)&ucDataByte, pdMS_TO_TICKS(0));
                    ucDataByte = 0;
                }
                
                if (usQAMCalibTriggerValue == CALIBRATIONSIGNAL)
                {
                    usQAMCalibTriggerValue = 0;
                    ucReceivedBitPackageCounter = 0;
                    /* Set new reference values for all QAM levels. */
                    uint8_t ucQAMLevelArrayPos = 1;
                    for (int8_t ucSetMedianRefCounter = 0; ucSetMedianRefCounter < 8; ucSetMedianRefCounter+=2)
                    {
                        if (ucSetMedianRefCounter + ucQAMLevelCalibArrayCounter < 8)
                        {
                            usQAMHalfMedianLevels[ucQAMLevelArrayPos] = (usQAMLevelsForCalibration[ucSetMedianRefCounter + ucQAMLevelCalibArrayCounter] + usQAMLevelsForCalibration[ucSetMedianRefCounter + 1 + ucQAMLevelCalibArrayCounter]) / 2;
                        }
                        else
                        {
                            usQAMHalfMedianLevels[ucQAMLevelArrayPos] = (usQAMLevelsForCalibration[ucSetMedianRefCounter + ucQAMLevelCalibArrayCounter - 8] + usQAMLevelsForCalibration[ucSetMedianRefCounter + 1 + ucQAMLevelCalibArrayCounter - 8]) / 2;
                        }
                        switch (ucQAMLevelArrayPos)
                        {
                            case 0: ucQAMLevelArrayPos=3; break;
                            case 1: ucQAMLevelArrayPos=0; break;
                            case 2: ucQAMLevelArrayPos=1; break;
                            case 3: ucQAMLevelArrayPos=2; break;
                            default: break;
                        }
                    }
                }
                else
                {
                    if (++ucByteswoCalibration >= (16))
                    {
                        ucByteswoCalibration = 0;
                    }
                }
                /* Find new reference value and copy new array */
                //vOffsetLevelAdjust(usReceivedValueArray, &usSignalOffsetLevel);
                
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