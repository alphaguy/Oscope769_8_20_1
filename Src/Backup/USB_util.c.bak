#include "USB_util.h"
#include "printTask.h"
#include "cmsis_os.h"

uint8_t RxData[RxBufSize];
volatile uint32_t data_received= 0;

extern uint8_t UserRxBufferFS[RxBufSize];
struct printMessages msg[16];
int indx=0;
extern osMessageQId printQueueHandle;
   
int __write(int fd, char *pBuffer, int size)
{

  
  msg[indx].Buf = pBuffer;
  msg[indx++].len = size;
  if(indx >15)
  {
    indx = 0;
  }
//  osMessagePut(printQueueHandle, (uint32_t)&msg[indx], osWaitForever);
  while(CDC_Transmit_HS((uint8_t*)pBuffer, size) == USBD_BUSY)  
  {
    osDelay(1);
  }
  
  return size;
}



int __read(int fd, char *pBuffer, int size)
{
  int retval = size;
  volatile int kbstatus=0;
  
  kbstatus = kbhit();
  while(kbstatus == 0)
    kbstatus = kbhit();
  if (size >= data_received)
  {
    memcpy(pBuffer, (char*)RxData, data_received);
    retval = data_received;
    data_received = 0;
  }
  else
  {
    memcpy(pBuffer, (char*)RxData, size);
  }
  return retval;
}

int __dwrite(int fd, char* pBuffer, int size)
{
  
  msg[indx].Buf = pBuffer;
  msg[indx++].len = size;
  if(indx > 15)
  {
    indx = 0;
  }
//  osMessagePut(printQueueHandle, (uint32_t)&msg[indx], osWaitForever);
  while(CDC_Transmit_HS((uint8_t*)pBuffer, size) == USBD_BUSY)  
  {
    osDelay(1);
  }
  return size;
}

int __close(int fd)
{
  return 0;
}

int __lseek(int fd, int offset)
{
  return 0;
}

int remove(char const* str)
{
  return 0;
}

int kbhit(void)
{
  if(data_received > 0)
    return 1;
  else return 0;
}

int getc(FILE * fd)
{
  while(!kbhit());
  data_received = 0;
  return (int)RxData[0];
}


int putc(int chr, FILE * fd)
{
  while(CDC_Transmit_HS((uint8_t*)&chr, 1) == USBD_BUSY);
  return 1;
}
  
void putch(char chr)
{
  while(CDC_Transmit_HS((uint8_t*)&chr, 1) == USBD_BUSY);
}
  
int getch(void)
{
  while(!kbhit());
  data_received = 0;
  return (int)RxData[0];
}