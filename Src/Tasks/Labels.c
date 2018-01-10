#include "oscope.h"

const char LabelColor[] = "Color";
const char LabelCursor[] = "Cursor";
const char LabelDisable[] = "Disable";
const char LabelEnable[] = "Enable";
const char LabelOffset[] = "Offset";
const char LabelVolts[] = "Volts";

const char LabelBLACK[] = "BLACK";
const char LabelBLUE[] = "BLUE";
const char LabelCYAN[] = "CYAN";
const char LabelGREEN[] = "GREEN";
const char LabelRED[] = "RED";
const char LabelWHITE[] = "WHITE";
const char LabelYELLOW[] = "YELLOW";

const char LabelSec[] = "Sec";
const char LabelmSec[] = "mSec";
const char LabeluSec[] = "uSec";

const char LabelOne[] = "1";
const char LabelOneHundred[] = "100";
const char LabelTen[] = "10";
const char LabelThirty[] = "30";
const char LabelThree[] = "3";
const char LabelThreeHundred[] = "300";

const char LabelDemo[] = "Demo";
const char LabelNormal[] = "Normal";
const char LabelTest[] = "Test";
const char LabelTouchCal[] = "TouchCal";

const char LabelChannel1[] = "Channel1";
const char LabelChannel2[] = "Channel2";
const char LabelMode[] = "MODE";
const char LabelTimeScale[] = "TimeScale";
const char LabelTrigger[] = "Trigger";

const char LabelAutoTrig[] = "Auto";
const char LabelChann1[] = "CH1";
const char LabelChann2[] = "CH2";
const char LabelExternal[] = "External";

const char LabelV[] = "V";
const char LabelmV[] = "mV";
const char LabeluV[] = "uV";

const char LabelBack[] = "Back";
const char LabelTimePerDiv[] = "Time/Div";
const char LabelVoltsPerDiv[] = "V/Div";


const char * LabelPointers[][7] =
{
  // State 0 Labels Top State
  LabelChannel1, LabelChannel2, LabelTimeScale, LabelTrigger, NULL, NULL, LabelMode,
  // State 1 Labels Channel State
  NULL, NULL, LabelVolts, LabelColor, LabelOffset, LabelCursor,  LabelBack,
  // State 2 Labels Major Time State
  LabelTimePerDiv, LabeluSec, LabelmSec, LabelSec, NULL, NULL, LabelBack,  
  // State 3 Labels Major Volts State
  LabelVoltsPerDiv, LabeluV, LabelmV, LabelV, NULL, NULL, LabelBack,
  // State 4 Labels Minor State
  LabelOne, LabelThree, LabelTen, LabelThirty, LabelOneHundred, LabelThreeHundred, LabelBack,
  // State 5 Labels Color State
  NULL, LabelRED, LabelGREEN, LabelBLUE, LabelYELLOW, LabelCYAN, LabelWHITE,
  // State 6 Labels Trigger State
  LabelTrigger, LabelChann1, LabelChann2, LabelExternal, LabelAutoTrig, NULL, LabelBack,
  // State 7 Labels Mode State
  LabelMode, LabelNormal, LabelDemo, LabelTouchCal, LabelTest, NULL, LabelBack,
  // State 8 Labels Channel 1 State
  LabelChannel1, NULL, NULL, NULL, NULL, NULL, NULL,
  // State 9 Labels Channel 2 State
  LabelChannel2, NULL, NULL, NULL, NULL, NULL, NULL,
  // State 10 Labels Channel 1 Enabled State
  NULL, LabelDisable, NULL, NULL, NULL, NULL, NULL,
  // State 11 Labels Channel 1 Disabled State
  NULL, LabelEnable, NULL, NULL, NULL, NULL, NULL,
  // State 12 Labels Channel 2 Enabled State
  NULL, LabelDisable, NULL, NULL, NULL, NULL, NULL,
  // State 13 Labels Channel 2 Disabled State
  NULL, LabelEnable, NULL, NULL, NULL, NULL, NULL
};


const char * getLabelPtr(int State, int LabelNum)
{
  return LabelPointers[State][LabelNum];
}
