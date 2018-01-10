/*
 * TS_util.h
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#ifndef UTIL_TS_UTIL_H_
#define UTIL_TS_UTIL_H_

//#include "ft5336.h"


/** @brief With FT5336 : maximum 5 touches detected simultaneously
  */
#define TS_MAX_NB_TOUCH                 ((uint32_t) FT5336_MAX_DETECTABLE_TOUCH)

#define TS_NO_IRQ_PENDING               ((uint8_t) 0)
#define TS_IRQ_PENDING                  ((uint8_t) 1)

#define TS_SWAP_NONE                    ((uint8_t) 0x01)
#define TS_SWAP_X                       ((uint8_t) 0x02)
#define TS_SWAP_Y                       ((uint8_t) 0x04)
#define TS_SWAP_XY                      ((uint8_t) 0x08)




void TS_Init(void);

void TouchHandlerRTOS(void const * params);
/*
//void TS_IO_Delay(int ms);
void TS_IO_Init(void);
uint8_t TS_IO_Read(uint16_t devaddr, uint16_t regaddr);
void TS_IO_Write(uint16_t devaddr, uint16_t regaddr, uint8_t writedata);

*/
#endif /* UTIL_TS_UTIL_H_ */
