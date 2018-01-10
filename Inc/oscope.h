#ifndef OSCOPE_H
#define OSCOPE_H


#define HeadingFont     GUI_FontArial22
#define ButtonFont      GUI_FontArial22

#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include "cmsis_os.h"

#include "GUI.h"
#include "OTM8009A.h"
#include "WM.h"
#include "GUItask.h"
#include "BUTTON.h"
#include "buttons.h"
#include "settings.h"
#include "normal.h"
#include "modes.h"
#include "labels.h"
#include "System1.h"
#include "scope.h"

extern GUI_CONST_STORAGE GUI_FONT GUI_FontArial18;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontArial22;

#endif //OSCOPE_H
