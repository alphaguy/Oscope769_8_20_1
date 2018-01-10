/* This file contains an initialization routine which registers
the communication objects used by the application to perform
FreeRTOS sync and messaging functions.  Registration is not 
required for the operation of the RTOS however it is needed
to enable plugins and RTOS aware debuggers and utilities to
find these structures and monitor their behavior.*/

#include "FreeRTOS.h"
#include "RTOSinit.h"
#include "CMSIS_os.h"
#include <stdarg.h>

extern osMessageQId GUIQueueHandle;
extern osMessageQId EventQueueHandle;
extern osMessageQId printQueueHandle;

extern osSemaphoreId ButtPressedHandle;
extern osSemaphoreId TimerDoneHandle;
extern osSemaphoreId TriggerHandle;
extern osSemaphoreId ADCdoneHandle;
extern osSemaphoreId DrawDoneHandle;

void RTOSinitPlugin(void)
{
  /* Register Queue and Semaphores for debugger */
  
  vQueueAddToRegistry(GUIQueueHandle, "GUI_Queue");
  vQueueAddToRegistry(EventQueueHandle, "EventQueue");
  vQueueAddToRegistry(printQueueHandle, "PrintQueue");
  
  vQueueAddToRegistry(ButtPressedHandle, "B_Pressed");
  vQueueAddToRegistry(TimerDoneHandle, "B_Time_Done");
  vQueueAddToRegistry(TriggerHandle, "Trigger");
  vQueueAddToRegistry(ADCdoneHandle, "ADC_Done");  
  vQueueAddToRegistry(DrawDoneHandle, "DrawDone");  
}
