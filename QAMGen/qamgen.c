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

#define DATABYTETOSENDMASK      0x1F

void vsendCommand(uint8_t Data[]);
void vsendFrame(void *pvParameters);

typedef enum
{
    Idle,
    sendSyncByte,
    sendDatenbuffer
    
} eProtokollStates;

TaskHandle_t xsendFrame;

EventGroupHandle_t xQuamgenal_1;
EventGroupHandle_t xQuamgenal_2;

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
    
       xQuamgenal_1=xEventGroupCreate();
       xQuamgenal_2=xEventGroupCreate();
       xQueue_Data = xQueueCreate( NR_OF_DATA_SAMPLES, sizeof( uint8_t ) );
    
        xTaskCreate(vsendFrame, NULL, configMINIMAL_STACK_SIZE+100, NULL, 2, &xsendFrame);
	
	for(;;) {
		vTaskDelay(10/portTICK_RATE_MS);
	}
}

void fillBuffer(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
    
    uint16_t Amp=1;
    uint8_t  EventGroupBits= xEventGroupGetBitsFromISR(xQuamgenal_1);
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
        xEventGroupSetBits(xQuamgenal_1,DATEN_AUFBEREITET);
}

// Mit ISR EventGroups arbeiten, da der Interrupt Buffer füllt
void fillBuffer_1(uint16_t buffer[NR_OF_GENERATOR_SAMPLES]) {
    uint8_t Amp_1=1;
    uint8_t  EventGroupBits= xEventGroupGetBitsFromISR(xQuamgenal_2);
    
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
       xEventGroupSetBits(xQuamgenal_2,DATEN_AUFBEREITET);
   
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
    
    uint8_t SendByteValue;
    uint8_t SendBitPaketCounter;
    uint8_t CheckVar=0;
    uint8_t TempValue;
    uint8_t DataByteCounter = 0;
    uint8_t DataBytesToSend;
    uint8_t BitCounter=0;
    uint8_t IdleSendByte=0;     // Difference between 0xAF and 0x05
    uint8_t Data[NR_OF_DATA_SAMPLES + 1] = {};
    QueueHandle_t xQueue_Data;

    
    eProtokollStates Protokoll = Idle;
    
    while(1)
    {
        

        
        switch(Protokoll)
        {
            case Idle:
            {
                if (IdleSendByte == 0)
                {
                    if (CheckVar >= 1)
                    {
                        IdleSendByte = 1;
                        TempValue = 0xAF;
                        CheckVar=0;
                    }
                }
                else
                {
                    if (CheckVar >= 1)
                    {
                        IdleSendByte = 0;
                        TempValue = 0x05;
                        CheckVar=0;
                            
                        if (xQueueReceive(xQueue_Data,(void*)&Data,pdMS_TO_TICKS(0))==pdTRUE)
                        {
                            DataBytesToSend = Data[0] & DATABYTETOSENDMASK;
                            Protokoll=sendSyncByte;
                        }                                
                    }
                }
                
                //Receive Queue (uint8) 33Byts
                //Check Temp Var in jedem Case
                //Wechsel sobald send Data wieder Idle
                break;
            }
            
            case sendSyncByte:
            {
                if (CheckVar>=1)
                {
                    TempValue=0xFF;
                    CheckVar=0;
                    Protokoll=sendDatenbuffer;
                }
                
                break;
            }                          
            
            case sendDatenbuffer:
            {  
                if (CheckVar>=1)
                {
                    if (DataByteCounter < DataBytesToSend)
                    {
                        TempValue = Data[DataByteCounter];
                        CheckVar=0;
                        DataByteCounter++;
                    } 
                    else
                    {
                        DataByteCounter = 0;
                        Protokoll=Idle;
                    }
                }

                break;
            }
            default:
            {
                Protokoll=Idle;
                break;
            }

        }
        if (SendBitPaketCounter >= 3)
        {
            SendByteValue = TempValue;
            SendBitPaketCounter = 0;
            CheckVar=1;
        }
         
        if (xEventGroupGetBits(xQuamgenal_1)&DATEN_AUFBEREITET)
        {
             xEventGroupClearBits(xQuamgenal_1,DATEN_AUFBEREITET);
             BitCounter+=1;
             
            if (SendByteValue& 0b00000001)
            {
                xEventGroupSetBits(xQuamgenal_1,AMPLITUDE_2);
                xEventGroupClearBits(xQuamgenal_1,AMPLITUDE_1);
            } 
            else
            {
                xEventGroupSetBits(xQuamgenal_1,AMPLITUDE_1);
                xEventGroupClearBits(xQuamgenal_1,AMPLITUDE_2);
            }
        }
                
        if (xEventGroupGetBits(xQuamgenal_2)&DATEN_AUFBEREITET)
        {
            xEventGroupClearBits(xQuamgenal_2,DATEN_AUFBEREITET);
            BitCounter+=1;
            
            if (SendByteValue& 0b00000010)
            {
                xEventGroupSetBits(xQuamgenal_2,AMPLITUDE_2);
                xEventGroupClearBits(xQuamgenal_2,AMPLITUDE_1);
            }
            else
            {
                xEventGroupSetBits(xQuamgenal_2,AMPLITUDE_1);
                xEventGroupClearBits(xQuamgenal_2,AMPLITUDE_2);
            }
        }        
        
        if(BitCounter>=2)
        {
            BitCounter=0;
            SendBitPaketCounter++;
            SendByteValue = SendByteValue >> 2;
        }
        
    }
}    