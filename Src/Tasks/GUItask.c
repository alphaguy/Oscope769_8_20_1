#include "oscope.h"

extern BUTTON_Handle ButtHandles[7];

int GUIready = 0;

GUI_MEMDEV_Handle GraphField;




const uint32_t GUIcolors[] = 
{
  GUI_BLACK,
  GUI_BLUE,
  GUI_CYAN,
  GUI_GREEN,
  GUI_RED,
  GUI_WHITE,
  GUI_YELLOW
};


void GUItaskStart(void const * argument)
{
  struct ParamList * ptrParams;
  extern osMessageQId GUIQueueHandle;
  extern osSemaphoreId DrawDoneHandle;
  extern struct ChannelTypeDef Chan1, Chan2;
  extern union ADC_data data1, data3;
  GUI_MEMDEV_Handle TempHandle;
  osEvent Qresult;
  
  GUI_Start();
  WM_Exec();    
  GUIready = 1; //release StateTask to start up
  
  /* Infinite loop */
  for(;;)
  {
    Qresult = osMessageGet(GUIQueueHandle,60);
    /* If valid message received, sequence State Machine */
    if((Qresult.status == osOK) | (Qresult.status == osEventMessage)
       | (Qresult.status == osEventMail))
    {
      Qresult.status = osErrorOS;
      ptrParams = Qresult.value.p;
      switch(ptrParams->operation)
      {
      case SetButton:
        if(ptrParams->param3.uParam != 0){ // exists is true

            WM_ShowWindow(ButtHandles[((ptrParams->param5.iParam) -1)]);

            //load TextColorPressed value
            BUTTON_SetTextColor(ButtHandles[ptrParams->param5.iParam -1],
                                BUTTON_CI_PRESSED,GUIcolors[ptrParams->param4.uParam]);
            //load TextColorUnpressed value
            BUTTON_SetTextColor(ButtHandles[ptrParams->param5.iParam -1],
                                BUTTON_CI_UNPRESSED,GUIcolors[ptrParams->param4.uParam]);
            //load text string
            BUTTON_SetText(ButtHandles[ptrParams->param5.iParam -1],
                           getLabelPtr(ptrParams->param2.uParam, 
                           ptrParams->param5.iParam - 1));
        }
        else 
        {
          WM_HideWindow(ButtHandles[ptrParams->param5.iParam -1]);
          if(getLabelPtr(ptrParams->param2.uParam, 
                           ptrParams->param5.iParam - 1) != NULL)
          {
            //Draw Heading
            GUI_SetBkColor(GUI_CYAN);
            GUI_SetColor(GUI_RED);
            GUI_DispStringAt(getLabelPtr(ptrParams->param2.uParam, 
                           ptrParams->param5.iParam - 1), 33, 55);
          }
        }
       
        break;
      case ClrButtonMenu:
        // Clear Button menu field
	GUI_SetBkColor(GUI_CYAN);
	GUI_ClearRect(0,27,149,452);

        break;
      case Writedata:
        TempHandle = GUI_MEMDEV_Select(GraphField); //select the memdev drawing area and
        DrawGrid(); // redraw the grid with memory device

        if(Chan1.active){
                GUI_SetColor(Chan1.color);//put it on screen
                GUI_DrawGraph((signed short*)data1.ADCreadback, 650, 150,240);
        }
        if(Chan2.active){
                GUI_SetColor(Chan2.color);//put it on screen
                GUI_DrawGraph((signed short*)data3.ADCreadback, 650, 150,240);
        }
        GUI_MEMDEV_CopyToLCD(GraphField); //put the memdev data on the screen

        GUI_MEMDEV_Select(TempHandle);
        osSemaphoreRelease(DrawDoneHandle);

        break;
      case WriteString:
        // param1 X start position, param2 Y start position,
        // param3 pointer to string, param4 Text color
         
        GUI_SetBkColor(GUI_BLUE);
        GUI_SetColor(ptrParams->param4.uParam);
	GUI_SetFont(&HeadingFont);
	GUI_DispStringAt((char*)ptrParams->param3.pParam,ptrParams->param1.uParam,
                         ptrParams->param2.uParam);

        break;
      case WriteValue:
        // param1 X start position, param2 Y start position,
        // param3 Value to print, param4 Text color

        GUI_SetBkColor(GUI_BLUE);
        GUI_SetColor(ptrParams->param4.uParam);
	GUI_SetFont(&HeadingFont);
        GUI_GotoXY(ptrParams->param1.iParam, ptrParams->param2.iParam);
	GUI_DispDecSpace(ptrParams->param3.iParam, (uint8_t)3);

        break;
      case WriteChar:
        // param1 X start position, param2 Y start position,
        // param3 Character to print, param4 Text color

        GUI_SetBkColor(GUI_BLUE);
        GUI_SetColor(ptrParams->param4.uParam);
	GUI_SetFont(&HeadingFont);
        GUI_GotoXY(ptrParams->param1.iParam, ptrParams->param2.iParam);
	GUI_DispChar((uint16_t)ptrParams->param3.uParam);

        break;
      default:
        break;
      }
    }
    osDelay(10);
    WM_Exec();
  }
}


void GUI_Start(void){
//   osDelay(100);
   __HAL_RCC_CRC_CLK_ENABLE();
  
 /* Enable Back up SRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();

 
  GUI_Init();
  osDelay(100);
//  HAL_Delay(100);
  WM_MULTIBUF_Enable(1);
  GUI_SetLayerVisEx (0, 1);
  GUI_SetLayerVisEx (1, 0);
  GUI_SelectLayer(1);
  GUI_Clear();

  GUI_SelectLayer(0);

  GUI_Clear();

    //Create a memory device for scope screen updates
  GraphField = GUI_MEMDEV_Create(150,27,650,426);
//  if(GraphField == 0)
    // Memory device failed to be created
//    printf("GraphField failed to be created\r\n");

  GUI_MEMDEV_Select(0); // make sure still writing direct for now




// Set up initial screen config
//Top border
  GUI_SetBkColor(GUI_BLUE);
  GUI_ClearRect(0,0,799,26);
//  GUI_SetFont(&GUI_Font16_1HK);
  GUI_SetFont(&HeadingFont);  
  GUI_DispStringAt("uSCOPE V0.1",5,2);
//Bottom Border
  GUI_ClearRect(0,479-27,799,479);
//Initial Button Pattern
  button_init();
  WM_Exec();
//Button menu field
  GUI_SetBkColor(GUI_CYAN);
  GUI_ClearRect(0,27,149,479-27);
  DrawGrid();
}


void DrawGrid(void){
  int i;

  GUI_SetBkColor(GUI_BLACK);
  GUI_ClearRect(150,27,799,479-27);
  GUI_SetPenSize(1);
  GUI_SetLineStyle(GUI_LS_DASH); // set style to dotted line
  GUI_SetColor(GUI_YELLOW);  // set the grid color to Yellow
  for(i=215; i<=799; i +=65)
          GUI_DrawLine(i,27,i,479-27); // draw vertical grid lines
  for(i=45; i< 479-27; i+=65) //draw horizontal grid lines
          if(i != 240)
                  GUI_DrawLine(150,i,799,i);
  GUI_SetColor(GUI_CYAN);
  GUI_DrawHLine(240,150,799);  // center line

}

