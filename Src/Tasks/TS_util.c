/*
 * TS_util.c
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#include "oscope.h"

extern I2C_HandleTypeDef hi2c4;
static uint8_t gotstart[2]={0,0};//, gotend[2]={0,0};
//static uint16_t lastX[2], lastY[2],lastStatus[2];



void PressTimeout(void const * argument);// function prototype extern in main()

void TS_Init(void){
  if(BSP_TS_Init(800, 480) !=0){}//startup TouchScreen controller
//    printf("Touch Screen error on Init\r\n");
    if(BSP_TS_ITConfig() != 0)  {}
//    printf("Touch Screen error on Interrupt config\r\n");
}



void TouchHandlerRTOS(void const * params)
{
#define newX TS_State.touchX[0]
#define newY TS_State.touchY[0]
#define mindistance 20


  TS_StateTypeDef TS_State;
  GUI_PID_STATE PID_State = {
          .Pressed = 1,
          .Layer = 0
  };

  extern osSemaphoreId ButtPressedHandle;
  extern osTimerId PressCheckHandle;

  osSemaphoreWait(ButtPressedHandle,0);// clear semaphore from touch INT

  while(1){
    // wait on semaphore from touch INT
    osSemaphoreWait(ButtPressedHandle,osWaitForever);
    osTimerStart(PressCheckHandle,80);

    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected){
      if(gotstart[0] == 0){
        gotstart[0] = 1;
        PID_State.x = (int)newX;
        PID_State.y = (int)newY;
        PID_State.Pressed = 1;
        GUI_PID_StoreState(&PID_State);
        WM_Exec();
      }
    }
  }
}



void PressTimeout(void const * argument){
#define newX TS_State.touchX[0]
#define newY TS_State.touchY[0]

	GUI_PID_STATE PID_State = {
		.Pressed = 0,
	};

	GUI_PID_GetCurrentState(&PID_State);
	PID_State.Pressed = 0;
	GUI_PID_StoreState(&PID_State);
        gotstart[0] = 0;

}

