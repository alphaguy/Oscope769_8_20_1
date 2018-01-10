/*
 * buttons.c
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#include "oscope.h"



BUTTON_Handle ButtHandles[7];
//extern struct ButtonParams ButtonStates[totalstates][7];
extern osMessageQId EventQueueHandle;

void* ButtCallbkPtr[7] ={
    (void*)BUTTON0_My_Callback, (void*)BUTTON1_My_Callback, (void*)BUTTON2_My_Callback,
    (void*)BUTTON3_My_Callback, (void*)BUTTON4_My_Callback, (void*)BUTTON5_My_Callback,
    (void*)BUTTON6_My_Callback
};




void button_init(void){
#define X1 10
#define X2 130
#define Y2 50

  const int Y[7] =
  {36, 95, 154, 213, 271, 330, 392};

  int i,j=0;
  BUTTON_Handle hWin;

  hWin = WM_GetDesktopWindow();

  // Create the 7 buttons
  for(i=0;i<7;i++){
    ButtHandles[i] = BUTTON_CreateEx(
                    X1, Y[i], X2, Y2,
                    hWin, WM_CF_SHOW,0,GUI_ID_OK+j++);
    WM_SetCallback(ButtHandles[i], (WM_CALLBACK*)ButtCallbkPtr[i]);
    BUTTON_SetFont(ButtHandles[i],&ButtonFont);
//    WM_ShowWindow(ButtHandles[i]);
  }
}





/*
 * Button Callback routines
 */

void BUTTON0_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 1 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(0);
                {
                  osMessagePut(EventQueueHandle, 1, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON1_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 2 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(1);
                {
                  osMessagePut(EventQueueHandle, 2, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON2_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 3 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(2);
                {
                  osMessagePut(EventQueueHandle, 3, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON3_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 4 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(3);
                {
                  osMessagePut(EventQueueHandle, 4, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON4_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 5 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(4);
                {
                  osMessagePut(EventQueueHandle, 5, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON5_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 6 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(5);
                {
                  osMessagePut(EventQueueHandle, 6, 10);
                }
	}
	BUTTON_Callback(pMsg);
}

void BUTTON6_My_Callback(WM_MESSAGE *pMsg){
	static GUI_PID_STATE PID_State;

	GUI_PID_GetState(&PID_State);
	if((pMsg->MsgId == WM_TOUCH)){
//		printf("Pressed Button 7 callback, pressed = %d\r\n",PID_State.Pressed);
		if(PID_State.Pressed == 0)// wait till released
//			buttonpressed(6);
                {
                  osMessagePut(EventQueueHandle, 7, 10);
                }
	}
	BUTTON_Callback(pMsg);

}
