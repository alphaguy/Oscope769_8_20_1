#include "oscope.h"

#define SES_OKAY  0

#define CODER_CLASSIC

void SetButtonParams (VS_UINT Label, VS_UINT StateNum, VS_UINT Present, VS_UINT LabelColor, VS_INT ButtonNumber);
void SetChannelColor (VS_INT CHANNEL, VS_UINT COLOR);
void SetMode (VS_INT RunMode);
void SetTimePerDiv(VS_INT TimePerDiv);
void SetTriggerMode(VS_INT TriggerMode);
void SetVperDiv(VS_INT VperDiv, VS_INT Channel);
void SetChannelEnable(unsigned int Channel, unsigned int enab);
void ClrMenu(void);

struct ParamList MsgParams[16];
static int i=0;

extern osMessageQId GUIQueueHandle;
extern osMessageQId EventQueueHandle;

extern struct ChannelTypeDef Chan1, Chan2;
extern const uint32_t GUIcolors[];


void incIndex(void);

void incIndex(void)
{
  i++;
  if(i == 16)
  {
    i=0;
  }
}


void HandleError (uint8_t ccArg)
{
}



/* StatesStart function */
void StatesStart(void const * argument)
{
  extern void VSInitAll(void);
  extern int GUIready;
  
  uint8_t cc;
  osEvent Qresult;
  
  /* Initialize the State Machine once on task entry*/
  while(GUIready == 0) // don't start until GUI is initialized
  {
    osDelay(10);
  }
  VSInitAll();
  cc = VSDeduct(SE_RESET);
  
  // Initialize Time scale amd Voltage Ranges
  osDelay(100);
  SetTimePerDiv (3000);
  SetVperDiv(3000000, 1);
  SetVperDiv(3000000, 2);
  
  /* Infinite loop */
  for(;;)
  {
    /* Wait on message sent in the Queue */
    Qresult = osMessageGet(EventQueueHandle,100);
    /* If valid message received, sequence State Machine */
    if((Qresult.status == osOK) | (Qresult.status == osEventMessage)
       | (Qresult.status == osEventMail))
    {
      cc = VSDeduct(Qresult.value.v);
      if (cc != SES_OKAY)
      {
        HandleError(cc);
      }
    }
    osDelay(1);
  }
}


void SetButtonParams(VS_UINT Label,VS_UINT StateNum, VS_UINT Present, VS_UINT LabelColor, VS_INT ButtonNumber)
{
  MsgParams[i].operation = SetButton;
  MsgParams[i].param1.iParam = Label;
  MsgParams[i].param2.uParam = StateNum;
  MsgParams[i].param3.uParam = Present;
  MsgParams[i].param4.uParam = LabelColor;
  MsgParams[i].param5.iParam = ButtonNumber;
  osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
               osWaitForever);
  incIndex();
}

void SetChannelColor (VS_INT CHANNEL, VS_UINT COLOR)
{
  if(CHANNEL == 1)
  {
    Chan1.color = GUIcolors[COLOR];
  }
  else if(CHANNEL == 2)
  {
    Chan2.color = GUIcolors[COLOR];
  }
}

void SetMode (VS_INT RunMode)
{
}

void SetTimePerDiv (VS_INT TimePerDiv)
{
  extern TIM_HandleTypeDef htim1;
  uint32_t prescale=24, divider, decade;
  
//  printf("Time per division will now be %d\r\n",TimePerDiv);
  
  if((TimePerDiv < 30) || (TimePerDiv > 3000000))
  {
    return;
  }
  
  divider = (TimePerDiv/10) -1;
  if(TimePerDiv > 300000)
  {
    divider = (divider/10);
    prescale = (prescale * 10) + 9;
  }
  
  //stop the timer
  HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
  
  //load the prescale and divider values
  htim1.Init.Prescaler = prescale;
  htim1.Init.Period = divider;
  HAL_TIM_Base_Init(&htim1);

  //start the timer
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  
    // Write the new Time/Div scale value to border
  MsgParams[i].operation = WriteString;
  MsgParams[i].param1.uParam = 20;
  MsgParams[i].param2.uParam = 455;
  MsgParams[i].param3.pParam = "Time Scale              Secs/Div";
  MsgParams[i].param4.uParam = GUI_WHITE;
  osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
               osWaitForever);
  incIndex();

  MsgParams[i].operation = WriteValue;
  MsgParams[i].param1.uParam = 135;
  MsgParams[i].param2.uParam = 455;
  MsgParams[i].param4.uParam = GUI_WHITE;
  if(TimePerDiv < 1000)
  {
    MsgParams[i].param3.uParam = TimePerDiv;
    decade = 'u';
  }
  else if(TimePerDiv < 1000000)
  {
    MsgParams[i].param3.uParam = TimePerDiv/1000;
    decade = 'm';
  }
  else
  {
    MsgParams[i].param3.uParam = TimePerDiv/1000000;
    decade = ' ';
  }
  osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
               osWaitForever);
  incIndex();

  MsgParams[i].operation = WriteChar;
  MsgParams[i].param1.uParam = 172;
  MsgParams[i].param2.uParam = 455;
  MsgParams[i].param3.uParam = decade;
  MsgParams[i].param4.uParam = GUI_WHITE;
  osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
               osWaitForever);
  incIndex();
 
  return;    
}


void SetTriggerMode (VS_INT TriggerMode)
{
}


void SetVperDiv (VS_INT VperDiv, VS_INT Channel)
{
  uint32_t decade;
  
//  printf("Setting Volts per Div for CH%d to %d\r\n",Channel, VperDiv);
  
  if(Channel == 1)
  {
    Chan1.vrange = ((float)VperDiv)/1000000;
    
    // Write the new V/Div scale value to border
    MsgParams[i].operation = WriteString;
    MsgParams[i].param1.uParam = 350;
    MsgParams[i].param2.uParam = 5;
    MsgParams[i].param3.pParam = "CH1              Volts/Div";
    MsgParams[i].param4.uParam = GUI_YELLOW;
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();    
    
    MsgParams[i].operation = WriteValue;
    MsgParams[i].param1.uParam = 400;
    MsgParams[i].param2.uParam = 5;
    MsgParams[i].param4.uParam = GUI_YELLOW;
    
    if(VperDiv < 1000)
    {
      MsgParams[i].param3.uParam = VperDiv;
      decade = 'u';
    }
    else if(VperDiv < 1000000)
    {
      MsgParams[i].param3.uParam = VperDiv/1000;
      decade = 'm';
    }
    else
    {
      MsgParams[i].param3.uParam = VperDiv/1000000;
      decade = ' ';
    }
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();

    MsgParams[i].operation = WriteChar;
    MsgParams[i].param1.uParam = 440;
    MsgParams[i].param2.uParam = 5;
    MsgParams[i].param3.uParam = decade;
    MsgParams[i].param4.uParam = GUI_YELLOW;
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();
  }
  
  else if(Channel == 2)
  {
    Chan2.vrange = ((float)VperDiv)/1000000;
    
    // Write the new V/Div scale value to border
    MsgParams[i].operation = WriteString;
    MsgParams[i].param1.uParam = 350;
    MsgParams[i].param2.uParam = 455;
    MsgParams[i].param3.pParam = "CH2              Volts/Div";
    MsgParams[i].param4.uParam = GUI_YELLOW;
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();
    
    MsgParams[i].operation = WriteValue;
    MsgParams[i].param1.uParam = 400;
    MsgParams[i].param2.uParam = 455;
    MsgParams[i].param4.uParam = GUI_YELLOW;
    if(VperDiv < 1000)
    {
      MsgParams[i].param3.uParam = VperDiv;
      decade = 'u';
    }
    else if(VperDiv < 1000000)
    {
      MsgParams[i].param3.uParam = VperDiv/1000;
      decade = 'm';
    }
    else
    {
      MsgParams[i].param3.uParam = VperDiv/1000000;
      decade = ' ';
    }
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();

    MsgParams[i].operation = WriteChar;
    MsgParams[i].param1.uParam = 440;
    MsgParams[i].param2.uParam = 455;
    MsgParams[i].param3.uParam = decade;
    MsgParams[i].param4.uParam = GUI_YELLOW;
    osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
                 osWaitForever);
    incIndex();
  }
}


void SetChannelEnable (unsigned int Channel, unsigned int enab)
{
  if(Channel == 1)
  {
    Chan1.active = enab;
  }
  else if(Channel == 2)
  {
    Chan2.active = enab;
  }
}

void ClrMenu(void)
{
  MsgParams[i].operation = ClrButtonMenu;
  osMessagePut(GUIQueueHandle, (uint32_t)(void*)&MsgParams[i].operation, 
               osWaitForever);
  incIndex();
}


