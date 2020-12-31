/*
 * qamdec.h
 *
 * Created: 05.05.2020 16:38:15
 *  Author: Chaos
 */ 


#ifndef QAMDEC_H_
#define QAMDEC_H_

void xProtocolDecoder(void* pvParameters);
void vQAMDec(void* pvParameters);

uint8_t ucQAMGetData(uint8_t* ucCommand, uint8_t* ucDataBytes, uint8_t ucDataArray[]);


#endif /* QAMDEC_H_ */