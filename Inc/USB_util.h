

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_UTIL_H
#define __USB_UTIL_H

#include "usb_device.h"
#include "usbd_cdc_if.h"


#define RxBufSize       256


int kbhit(void);
void putch(char chr);
int getch(void);
int putc(int chr, FILE * fd);
int getc(FILE * fd);

#endif /* __USB_UTIL_H */