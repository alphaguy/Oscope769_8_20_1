#include "oscope.h"
#include "usbd_cdc_if.h"
#include "printTask.h"



void StartPrintTask(void const * argument)
{
  osEvent Qresult;
  struct printMessages* ptrParams;
  extern void MX_USB_DEVICE_Init(void);
  extern osMessageQId printQueueHandle;
  
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  osDelay(1000);

  /* USER CODE BEGIN 5 */
//  int counter=0;
  /* Infinite loop */
  for(;;)
  { 
    Qresult = osMessageGet(printQueueHandle,osWaitForever);
  
    ptrParams = Qresult.value.p;    
    while(CDC_Transmit_HS((uint8_t*)(ptrParams->Buf), ptrParams->len) == USBD_BUSY)
    {
      osDelay(1);
    }
  }
  /* USER CODE END 5 */ 
}

