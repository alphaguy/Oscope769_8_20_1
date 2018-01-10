/*
 * INT_util.c
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#include "oscope.h"
#include "INT_util.h"
#define uint uint32_t

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern osSemaphoreId ADCdoneHandle;
extern TIM_HandleTypeDef htim1;
extern struct ChannelTypeDef Chan1, Chan2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;

int DMA1done = 0, DMA3done = 0, ADCDMAdone = 0; // flags to indicate conversions done



/*
 * This routine is usd to service all interrupts that occur on GPIO
 * oins programmed as interrupt inputs.  The pin which caused the
 * interrupt event is passed into this handler so that different actions
 * can be performed based on the meaning of the occurring interrupt.
 *
 * Pin I3 is an interrupt input for the Trigger signal to start a scan
 * when the Scope is in External Trigger mode.
 *
 * Pin I13 is an external interrupt connected to the TouchScreen controller
 * indicating action needed
 */

void HAL_GPIO_EXTI_Callback(uint16_t gpioPIN){
    extern osSemaphoreId TriggerHandle;
    extern osSemaphoreId ButtPressedHandle;
    extern int GUIready;

    if(gpioPIN == GPIO_PIN_4){  // this is the trigger to start collecting 400 ADC samples
        if(GUIready !=0)   //if OS and GUI not started yet, don't send semaphore
        {  
//          HAL_NVIC_DisableIRQ(EXTI4_IRQn);
          osSemaphoreRelease(TriggerHandle);// signal interrupt Sync Trigger  
        }
        return;
    }


    else if(gpioPIN == GPIO_PIN_13){ // TS Interrupt input on Pin I13
        osSemaphoreRelease(ButtPressedHandle);// signal interrupt for TS Handler
        return;
    }
}


/*
* This routine is called by the HAL interrupt handler if an interrupt
* occurs triggered by a conversion cycle being complete in an acive ADC.
*
* As used by the Scope, the ADC channels are programmed in DMA mode, so
* this interrupt occurs when a complete screen scan number of conversions
* is completed.  Foe example, if a scan consists of 400 samples, transferred
* by the DMA to memory, when 400 values are inmemory this interrupt occurs.
*/

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc){

  if(!Chan1.active)
  {
    DMA1done = 1;
  }
  if(!Chan2.active)
  {
    DMA3done = 1;
  }

  if (hadc->Instance == hadc1.Instance){
          DMA1done = 1;
          HAL_ADC_Stop_DMA(&hadc1);  // stop the ADC til next time
          if(DMA3done == 1){
                  ADCDMAdone = 1;
          }
  }
  else if (hadc->Instance == hadc3.Instance){
          DMA3done = 1;
          HAL_ADC_Stop_DMA(&hadc3);  //stop the ADC til next time
          if(DMA1done == 1){
                  ADCDMAdone = 1;
          }
  }
  if(ADCDMAdone){
          HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1); // Stop the ADC conversion clock
          DMA1done = 0;
          DMA3done = 0;
          ADCDMAdone = 0;
          osSemaphoreRelease(ADCdoneHandle);// start task to display data

  }  
  return;
}


void ADC1_DMA_Done(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(&hdma_adc1, DMA_IT_HT) == RESET)
  {
    DMA1done = 1;
    HAL_ADC_Stop_DMA(&hadc1);  // stop the ADC til next time
    if((DMA3done == 1) || (!Chan2.active)){
      HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1); // Stop the ADC conversion clock
      DMA1done = 0;
      DMA3done = 0;
      osSemaphoreRelease(ADCdoneHandle);// start task to display data
    }
  }
}



void ADC3_DMA_Done(void)
{
  if(__HAL_DMA_GET_IT_SOURCE(&hdma_adc3, DMA_IT_HT) == RESET)
  {
    DMA3done = 1;
    HAL_ADC_Stop_DMA(&hadc3);  //stop the ADC til next time
    if((DMA1done == 1) || (!Chan1.active)){
      HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1); // Stop the ADC conversion clock
      DMA1done = 0;
      DMA3done = 0;
      osSemaphoreRelease(ADCdoneHandle);// start task to display data
    }
  }
}



/*
* This routine is called by the HAL interrupt handler if an interrupt
* occurs triggered by an error condition in an acive ADC.
*/

void HAL_ADC_ErrorCallback (ADC_HandleTypeDef * hadc)
{
  if (hadc->Instance == hadc1.Instance)
  {
//    printf("ADC1 error\r\n");
  }
  if (hadc->Instance == hadc3.Instance)
  {
//    printf("ADC3 error\r\n");
  }
}


/*
* This routine is called by the HAL interrupt handler if an interrupt
* occurs triggered by an ADC value being read which is outside of the
* Values defined by the upper and lower threshold values when the ADC
* is in Analog Watchdog mode.
*
* This is used when the Scope is in Channel 1 or Channel 2 trigger mode.
*
* If triggering on a channel, thatA/D is set in continuous conversion mode,
* to sample as fast as possible.  A threshold is set within the operating
* range of the A/D so that this interrupt is triggered when the input 
* signal passes that threshold indicating a recurring trigger.  This will
* be used to start a normal A/D conversion cycle for one screen of data.
*/

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
  extern osSemaphoreId TriggerHandle;

  if (hadc->Instance == hadc1.Instance)
  {
    // Set semaphore to signal trigger occurred
    osSemaphoreRelease(TriggerHandle);// Channel 1 Sync Trigger
  }
  if (hadc->Instance == hadc3.Instance)
  {
    osSemaphoreRelease(TriggerHandle);// Channel 2 Sync Trigger
  }
}


