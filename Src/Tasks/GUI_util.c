/*
 * GUI_util.c
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#include "oscope.h"

GUI_MEMDEV_Handle GraphField;
QueueHandle_t ScreenQueue = NULL;

void GUI_Task(void){
  struct GUI_Message command;
  extern osSemaphoreId DrawDoneHandle;
  extern struct ChannelTypeDef Chan1, Chan2;
  extern union ADC_data data1, data3;
  GUI_MEMDEV_Handle TempHandle;

  ScreenQueue = xQueueCreate(4, sizeof(command));
  if(ScreenQueue==NULL){
    printf("Could not create ScreenQueue\r\n");
  }

  GUI_Start();

  WM_Exec();
  while(1){

    while(xQueueReceive(ScreenQueue,&command,0) == errQUEUE_EMPTY){
            WM_Exec();
            osDelay(20);
    }

    switch(command.op){
    case changestate:
      ButtonChangeState(command.num, command.heading);
      break;
    case writedata:
      TempHandle = GUI_MEMDEV_Select(GraphField); //select the memdev drawing area and
      DrawGrid(); // redraw the grid with memory device

      if(Chan1.active){
        GUI_SetColor(Chan1.color);//put it on screen
        GUI_DrawGraph(data1.ADCreadback, 799, 150,240);
      }
      if(Chan2.active){
        GUI_SetColor(Chan2.color);//put it on screen
        GUI_DrawGraph(data3.ADCreadback, 799, 150,240);
      }
      GUI_MEMDEV_CopyToLCD(GraphField); //put the memdev data on the screen

      GUI_MEMDEV_Select(TempHandle);
      WM_Exec();
      osSemaphoreRelease(DrawDoneHandle);

      break;
    }

  }

}

