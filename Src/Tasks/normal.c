/*
 * normal.c
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#include "oscope.h"


//int triggered = 0;

//These are data storage areas that are made global to avoid blowing
//up the task stack storage requirements.  The first are two unions that
//represent arrays of 200 32 bit words of data for the DMA to write to
// during capture, and arrays of 400 16 bit values when read back for
// processing.  The next is an array of 400 normalized data points
//to be written to the screen.  This can be optimized by re-using the
//capture arrays to write to the screen


  union ADC_data   data1 __attribute__((aligned(4))), 
                   data3 __attribute__((aligned(4)));

 // int16_t DataArray[650];
        

/*
 * for interrupt service routine that starts the ADC DMA see INT_util.c
 * for the Trigger interrupt
 *
 */



void NormalMode(void const * argument){
  extern osSemaphoreId TriggerHandle;
  extern osSemaphoreId ADCdoneHandle;
  extern osSemaphoreId DrawDoneHandle;
  extern int TriggerMode;
  int i;
  float tempreal;

  extern ADC_HandleTypeDef hadc1;
  extern ADC_HandleTypeDef hadc3;
  extern TIM_HandleTypeDef htim1;
  extern struct ChannelTypeDef Chan1, Chan2;

  struct ParamList GUIcmd;

  extern osMessageQId GUIQueueHandle;


  GUIcmd.operation = Writedata;

  while(1){// do forever loop.  This is the program of the Normal Mode task
    // start up the trigger interrupt to wait for next sync signal
    
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);// clear pending IRQ if there
    
    osSemaphoreWait(TriggerHandle,0); // Clear trigger semaphore

//    if(TriggerMode == ExternalTrigger)
    {
      HAL_NVIC_EnableIRQ(EXTI4_IRQn); // turn on trigger interrupt
    }
//    osSemaphoreWait(TriggerHandle,0); // Clear trigger semaphore

// Now wait for semaphore sent by Trigger interrupt ISR

    osSemaphoreWait(TriggerHandle,osWaitForever); // Wait for trigger semaphore

// disable trigger int so we don't get another while collecting data

//    if(TriggerMode == ExternalTrigger)
    {
//      HAL_NVIC_DisableIRQ(EXTI4_IRQn); // turn off trigger int until scan complete
    }
    
//    osSemaphoreWait(TriggerHandle,0); // Clear trigger semaphore

    osSemaphoreWait(ADCdoneHandle,0);// clear ADC semaphore

    if(Chan1.active)
    {
      HAL_ADC_Start_DMA(&hadc1,data1.ADCstart, 1300);  // start a DMA set of conversions on channel 1
    }
    if(Chan2.active)
    {
      HAL_ADC_Start_DMA(&hadc3,data3.ADCstart, 1300);  // start a DMA set of conversions on channel 3   
    }
    if(Chan1.active | Chan2.active)
    {
      HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); //start the timer if either is active
      osSemaphoreWait(ADCdoneHandle,osWaitForever); //Wait till set by Int
    }
    
    // A/Ds are running.  now wait for A/D conversion complete interrupt



// If Channel 1 is on, collect the data, scale and display it
    if(Chan1.active){
      for(i = 0; i<650; i++){ //scale the data and put in array to send to screen
        tempreal = (float)data1.ADCreadback[i];
        tempreal = -(tempreal*3.3*65)/(256 * Chan1.vrange);
        data1.ADCreadback[i] = (I16)tempreal - ((int)(Chan1.offset * 65));
        if(data1.ADCreadback[i]>210) data1.ADCreadback[i] = 210;
        if(data1.ADCreadback[i]<-210) data1.ADCreadback[i] = -210;
      }
    }

// If Channel 2 is on, collect the data, scale and display it
    if(Chan2.active){
      for(i = 0; i<650; i++){
        tempreal = (float)data3.ADCreadback[i];
        tempreal = -(tempreal*3.3*65)/(256 * Chan2.vrange);
        data3.ADCreadback[i] = (I16)tempreal - ((int)(Chan2.offset * 65));
        if(data3.ADCreadback[i]>210) data3.ADCreadback[i] = 210;
        if(data3.ADCreadback[i]<-210) data3.ADCreadback[i] = -210;
      }
    }
    osSemaphoreWait(DrawDoneHandle,0); //clear the draw complete semaphore

    //wait til drawing is done before we continue on to
    //enable the interrupt and get another scan.
    if(Chan1.active | Chan2.active)
    {
      osMessagePut(GUIQueueHandle, (uint32_t)(void*)&GUIcmd.operation,osWaitForever);
      osSemaphoreWait(DrawDoneHandle,osWaitForever);
    }
  }
}






