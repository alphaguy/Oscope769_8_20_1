/*
 * normal.h
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#ifndef UTIL_NORMAL_H_
#define UTIL_NORMAL_H_

union ADC_data{
	int16_t ADCreadback[800];
	uint32_t ADCstart[400];
};



void NormalMode(void const * argument);


#endif /* UTIL_NORMAL_H_ */
