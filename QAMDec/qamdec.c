/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
*  Author: Martin Burger (base version, ADC, DMA and timer handling)
*          Raphael Amgwerd (data decoding and protocol implementation)
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
/* The use of this library only works with code optimisation enabled.
   Otherwise the data decoding will be too slow. */
# warning "Compiler optimizations disabled; functions from qamdec.c won't work as designed"
#endif

/* Defines how many data is sampled within one DMA cycle. */
#define DECODERSAMPLECOUNT              32UL
/* Defines the frequency of the ADC. DMA frequency depends on decoder sample count. */
#define ADCFREQUENCY	                1000UL * DECODERSAMPLECOUNT

/* Bytes which can be stored in received data bytes queue. */
#define DATABYTEQUEUELENGTH             8

/* Amount of DMA transitions which can be stored temporarily. (Must be greater than 3) */
#define RECEIVEARRAYSIZE                4

/* Margin in which the max. and the min. position can differ from half of the sample amount. */
#define POSDIFFMARGIN                   3

/* Margin in which difference of max. and min. value from offset level can be. */
#define OFFSETLEVELMARGIN               800

/* Indicates if a new complete signal was found. */
#define SIGNALNOTFOUND                  0
#define COMPLETESIGNALFOUND             1

/* QAM level definitions. */
#define QAMLEVELS                       4       // QAM 4 is implemented.
#define QAMPACKAGESPERBYTE              4       // 4 bit packages of 2 bits for one byte.

/* Protocol defines */
#define SYNCHRONISATIONBYTE             0xFF
#define CALIBRATIONSIGNAL               0xAF05
#define COMMANDMASK                     0xE0    // Command + Amount of Data 0bXXXy'yyyy.
#define COMMANDBITPOSITION              5
#define DATABYTESMASK                   0x1F
#define MAXRECEIVEDATABYTES             32      // Maximum amount of data bytes received.
#define COMMANDBYTEPOSITION             0       // Position of command byte in data byte array.
#define DATABYTECOUNTPOSITION           1       // Position of data byte count byte in data byte array.
#define DATABYTESTARTPOSITION           2       // Start position of data bytes in data byte array.
#define PROTOCOLCOMPLETEFLAG            0x01    // Flag that indicates a protocol was completely received.


/* Declaration of protocol decoder state machine */
typedef enum
{
    Idle,
    Command,
    Data,
    Checksum
} eProtocolDecoderStates;

/* Reference values for decoding. */
uint16_t usLongTimeMaxValue = 750;  // This value will be adjusted to the absolute maximum value of QAM Level 3 (dynamically during runtime).
uint16_t usAbsoulteMaxValue = 1438; // This value is the maximum value of compare array of QAM Level 3. (see below)
int16_t sDataReference0[DECODERSAMPLECOUNT] = {    0,   237,   447,   606,   700,   719,   668,   559,
                                                 410,   245,    88,   -37,  -120,  -150,  -133,   -77,
                                                   0,    77,   133,   150,   120,    37,   -88,  -245,
                                                -410,  -559,  -668,  -719,  -700,  -606,  -447,  -237};
                                                
int16_t sDataReference1[DECODERSAMPLECOUNT] = {    0,   317,   603,   833,   989,  1059,  1047,   960,
                                                 819,   646,   467,   303,   169,    77,    23,     3,
                                                   0,    -3,   -23,   -77,  -169,  -303,  -467,  -646,
                                                -819,  -960, -1047, -1059,  -989,  -833,  -603,  -317};
                                                
int16_t sDataReference2[DECODERSAMPLECOUNT] = {    0,   393,   736,   985,  1109,  1098,   957,   715,
                                                 410,    89,  -201,  -416,  -529,  -529,  -422,  -233,
                                                   0,   233,   422,   529,   529,   416,   201,   -89,
                                                -410,  -715,  -957, -1098, -1109,  -985,  -736,  -393};
                                                
int16_t sDataReference3[DECODERSAMPLECOUNT] = {    0,   473,   892,  1212,  1398,  1438,  1336,  1116,
                                                 819,   490,   178,   -76,  -240,  -302,  -266,  -153,
                                                   0,   153,   266,   302,   240,    76,  -178,  -490,
                                                -819, -1116, -1336, -1438, -1398, -1212,  -892,  -473};

/* Data buffers for ADC values. */
uint16_t usADCBuffer0[DECODERSAMPLECOUNT];
uint16_t usADCBuffer1[DECODERSAMPLECOUNT];

/* Data array for received data bytes. */
uint8_t ucQAMDataBytes[MAXRECEIVEDATABYTES + 2] = {};

/* Decoder queue for sampled ADC data. */
QueueHandle_t decoderQueue;
/* Received bytes to send to protocol decoder. */
volatile QueueHandle_t receivedByteQueue;
/* Event group to indicate if protocol was completely received. */
EventGroupHandle_t receivedProtocolEventGroup;

void initADC(void) {
	ADCA.CTRLA = 0x01;
	ADCA.CTRLB = 0x00;
	ADCA.REFCTRL = 0x20;        // INTVCC = External reference from AREF pin (PA0) on PORT A (must be connected to Vcc)
	ADCA.PRESCALER = 0x03;
	ADCA.EVCTRL = 0x39;         // Event Channel 7 triggers Channel 0 Conversion
	ADCA.CH0.CTRL = 0x01;       // Single ended positive input without gain
	ADCA.CH0.MUXCTRL = 0x55;    // Input = ADC10 on ADCA = Pin PB2 = DAC-Output
	ADCA.CH0.INTCTRL = 0x00;
}

void initADCTimer(void) {
	TC1_ConfigClockSource(&TCD1, TC_CLKSEL_DIV1_gc);
	TC1_ConfigWGM(&TCD1, TC_WGMODE_SINGLESLOPE_gc);
	TC_SetPeriod(&TCD1, 32000000/(ADCFREQUENCY));
    
    /* Setup event system with timer TCD1 overflow. */
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
	DMA.CH2.DESTADDR0 = ((uint16_t)(&usADCBuffer0[0]) >> 0) & 0xFF;
	DMA.CH2.DESTADDR1 = ((uint16_t)(&usADCBuffer0[0]) >> 8) & 0xFF;
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
	DMA.CH3.DESTADDR0 = ((uint16_t)(&usADCBuffer1[0]) >> 0) & 0xFF;
	DMA.CH3.DESTADDR1 = ((uint16_t)(&usADCBuffer1[0]) >> 8) & 0xFF;
	DMA.CH3.DESTADDR2 = 0x00;

	DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
	DMA.CH3.CTRLA |= DMA_CH_ENABLE_bm;
}



uint8_t bGetReceivedData(uint16_t usReceiveArray[], uint8_t* ucActualArrayPos, uint8_t* ucLastTransEnd, uint16_t* usSignalOffsetLevel, uint8_t* ucQAMValue)
{
/* Counter for current data byte position. */
uint8_t ucDataCounter = 0;

/* Flag to indicate if new signal was found. */
uint8_t ucTransitionFound = SIGNALNOTFOUND;

/* Variables for mean value calculation. */
uint16_t usLowerHalfSignalOffset = 0;
uint16_t usUpperHalfSignalOffset = 0;

/* Variables for signal shape detection. */
uint8_t ucMaxValuePos;
uint8_t ucMinValuePos;
uint8_t ucMinMaxPosDifference;
uint16_t ucMaxValue;
uint16_t ucMinValue;
uint16_t ucMinMaxDifference;
int16_t sActualPositionDifference;
volatile int16_t sSignalDifference[4];
float fScaleFactor;


    /* Set data counter to last end position of buffer. */
    ucDataCounter = (*ucLastTransEnd > 5) ? *ucLastTransEnd - 5 : 0;
    
    do {
        /* The following checks are needed to detect one complete signal. */
        
        /* Check for offset level transition. */
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
                    if ((ucMinMaxPosDifference >= ((DECODERSAMPLECOUNT / 2) - 1 - POSDIFFMARGIN)) && (ucMinMaxPosDifference <= ((DECODERSAMPLECOUNT / 2) - 1 + POSDIFFMARGIN)))
                    {
                        ucMinMaxDifference = (ucMinValue + ucMaxValue) / 2;
                        if ((ucMinMaxDifference >= (*usSignalOffsetLevel - OFFSETLEVELMARGIN)) && (ucMinMaxDifference <= (*usSignalOffsetLevel + OFFSETLEVELMARGIN)))
                        {
                            /* Calculate scale factor to adjust the initial reference array to the actual waveform scale. */
                            fScaleFactor = (float)usAbsoulteMaxValue / (float)usLongTimeMaxValue;
                            
                            sSignalDifference[0] = 0;
                            sSignalDifference[1] = 0;
                            sSignalDifference[2] = 0;
                            sSignalDifference[3] = 0;
                            for (uint8_t ucPosCounter = 0; ucPosCounter < 32; ++ucPosCounter)
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
                            ucTransitionFound = COMPLETESIGNALFOUND;
                            
                            /* If it's a QAM level 3, then adjust the absolute maximum signal value. (Needed for scale factor calculation) */
                            if (*ucQAMValue == 3)
                            {
                                usLongTimeMaxValue = ((usLongTimeMaxValue * 5) + ((int16_t)ucMaxValue - *usSignalOffsetLevel)) / 6;
                            }
                        }                        
                    }
                }
                
                
            }
        }
        ++ucDataCounter;
        /* Repeat as long as no transition (start of signal = sine-transition) was found
           or no complete signal could be found. */
    } while ((ucTransitionFound == SIGNALNOTFOUND) && ((ucDataCounter - 1) < (*ucActualArrayPos)));
    
    --ucDataCounter; // correct position of found transition
    
    
    if (ucTransitionFound == COMPLETESIGNALFOUND)
    {
        /* Copy array values. Extract received data and modify receive data buffer. */
        
        /* Set new reference position in receive array. */
        if (*ucActualArrayPos <= ((RECEIVEARRAYSIZE - 2) * DECODERSAMPLECOUNT))
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
    {
        /* If no complete signal was detected, increase new reference position in array. */
        if (*ucActualArrayPos <= ((RECEIVEARRAYSIZE - 2) * DECODERSAMPLECOUNT))
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

/* With this function the last received protocol can be retrieved.
   Returns pdTRUE in case a message was received, otherwise pdFALSE. */
uint8_t ucQAMGetData(uint8_t* ucCommand, uint8_t* ucDataBytes, uint8_t ucDataArray[])
{
uint8_t ucReturnValue = pdFALSE;

    /* Check if protocol is complete and message was received. */
    if (xEventGroupClearBits(receivedProtocolEventGroup, PROTOCOLCOMPLETEFLAG) & PROTOCOLCOMPLETEFLAG)
    {
        /* EventGroup is used here, because if turned required optimization on,
           queue was optimized away. A workaround is as here implemented with an EventGroup.
           The positive effect is that just one data array must be defined
           which is shared with the xProtocolDecoder task. */
        
        /* Copy data values to variables. */
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

/* This function must be executed as a task.
   The tasks decodes the received bytes
   checks if a complete message within a protocol was received. */
void xProtocolDecoder(void* pvParameters) {
/* Protocol state machine initialization. */
eProtocolDecoderStates eProtocolDecoder = Idle;

/* Variable to store data bytes received from vQAMdec task. */
uint8_t ucReceivedByte;

/* Variables to check amount of data bytes which should be received
   and counter of data bytes received. */
volatile uint8_t ucDataBytesReceived;
volatile uint8_t ucDataBytesCounter = 0;

/* Variable to calculate checksum. */
uint8_t ucChecksumCalculated;

    /* Event group creation to indicate if complete message was received. */
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
                    /* In idle state, wait for synchronisation byte, which indicates the start of a message transmission. */
                    if (ucReceivedByte == SYNCHRONISATIONBYTE)
                    {
                        eProtocolDecoder = Command;
                        xEventGroupClearBits(receivedProtocolEventGroup, PROTOCOLCOMPLETEFLAG);
                    }
                    break;
                }
                case Command:
                {
                    /* Command byte extraction. */
                    ucQAMDataBytes[COMMANDBYTEPOSITION] = (ucReceivedByte & COMMANDMASK) >> COMMANDBITPOSITION;
                    
                    /* Amount of data bytes extraction. */
                    ucDataBytesReceived = ucReceivedByte & DATABYTESMASK;
                    ucQAMDataBytes[DATABYTECOUNTPOSITION] = ucDataBytesReceived;
                    
                    ucChecksumCalculated = ucReceivedByte; 
                    if (ucDataBytesReceived > 0)
                    {
                        eProtocolDecoder = Data;
                    }
                    else
                    {
                        /* If no data bytes should be received, go to checksum check. */
                        eProtocolDecoder = Checksum;
                    }
                    break;
                }
                case Data:
                {
                    /* Get data byte and store to data byte array. */
                    ucQAMDataBytes[DATABYTESTARTPOSITION + ucDataBytesCounter] = ucReceivedByte;
                    
                    /* Calculate checksum. */
                    ucChecksumCalculated ^= ucReceivedByte;
                    if (++ucDataBytesCounter >= ucDataBytesReceived)
                    {
                        /* If all data bytes are received, go to checksum check. */
                        eProtocolDecoder = Checksum;
                    }
                    break;
                }
                case Checksum:
                {
                    /* Compare calculated checksum with received checksum. */
                    if (ucReceivedByte == ucChecksumCalculated)
                    {
                        /* If the calculated checksum matches the received checksum,
                           set the flag, to indicate complete message was received. */
                        xEventGroupSetBits(receivedProtocolEventGroup, PROTOCOLCOMPLETEFLAG);
                    }
                    else
                    {
                        ucQAMDataBytes[COMMANDBYTEPOSITION] = 0x07;
                        ucQAMDataBytes[DATABYTECOUNTPOSITION] = 0x00;
                        xEventGroupSetBits(receivedProtocolEventGroup, PROTOCOLCOMPLETEFLAG);
                        
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
            vTaskDelay(pdMS_TO_TICKS(4));
        }
    }
}

/* This function must be executed as a task.
   The task handles the QAM decoding functions
   and sends the decoded bytes to the protocol decoder task
   via a queue.
   This task must be executed with one of the highest priorities,
   otherwise loss of data is expected. */
void vQAMDec(void* pvParameters) {

/* Receive buffer for received ADC values. */
uint16_t usReceiveArray[RECEIVEARRAYSIZE * DECODERSAMPLECOUNT] = {};

/* Current position of received buffer. */
uint8_t ucArrayReference = 0;
/* Position of last sine transition of received signal. Saved for next signal recognition. */
uint8_t ucLastReceiveEnd = 0;

/* Decoded QAM value. */
uint8_t ucActualQAMValue;

/* Used for idle signal detection, which is used to recognize the beginning of a byte. */
uint16_t usQAMCalibTriggerValue = 0;

/* Received data byte. */
uint8_t ucDataByte = 0;

/* Counts amount of received bit packages. */
uint8_t ucReceivedBitPackageCounter = 0;

/* Offset voltage (must be in a guilty range at startup). */
uint16_t usSignalOffsetLevel = 2000;

    
	decoderQueue = xQueueCreate(1, DECODERSAMPLECOUNT*sizeof(int16_t));
    receivedByteQueue = xQueueCreate(DATABYTEQUEUELENGTH, sizeof(uint8_t));
	
	initDecDMA();
	initADC();
	initADCTimer();
	xTaskCreate(xProtocolDecoder, (const char *) "ProtocolDecoder", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    
	while (1)
    {
        /* Check if ADC values received and in queue. */
		if (uxQueueMessagesWaiting(decoderQueue))
		{
    		xQueueReceive(decoderQueue, &usReceiveArray[ucArrayReference], pdMS_TO_TICKS(0));
            /* Check if signal is complete and extract bit package of the sampled ADC values. */
            if (bGetReceivedData(&usReceiveArray[0], &ucArrayReference, &ucLastReceiveEnd, &usSignalOffsetLevel, &ucActualQAMValue) == COMPLETESIGNALFOUND)
            {
                /* If data signal was received. */
                
                /* Fill data byte by left shift (LSB protocol). */
                ucDataByte = (ucDataByte >> 2) | (ucActualQAMValue << 6);
                usQAMCalibTriggerValue = (usQAMCalibTriggerValue >> 2) | (ucActualQAMValue << 14);
                
                /* Check if a full byte was received. */
                if (++ucReceivedBitPackageCounter >= QAMPACKAGESPERBYTE)
                {
                    /* If a complete byte was received, push it into the queue. */
                    ucReceivedBitPackageCounter = 0;
                    xQueueSend(receivedByteQueue, (void*)&ucDataByte, pdMS_TO_TICKS(0));
                    ucDataByte = 0;
                }
                
                if (usQAMCalibTriggerValue == CALIBRATIONSIGNAL)
                {
                    /* Calibration signal received. Define this as the start of a byte. */
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

/* This function fills the decoderQueue with the received ADC data. */
void fillDecoderQueue(uint16_t usDataBuffer[DECODERSAMPLECOUNT]) {
	BaseType_t xTaskWokenByReceive = pdFALSE;
	xQueueSendFromISR(decoderQueue, &usDataBuffer[0], &xTaskWokenByReceive);
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;
	fillDecoderQueue(&usADCBuffer0[0]);
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB|=0x10;
	fillDecoderQueue(&usADCBuffer1[0]);
}