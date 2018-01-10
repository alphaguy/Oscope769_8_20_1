#ifndef GUITASK_H
#define GUITASK_H

#include "stm32f7xx_hal.h"
/*
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "Scope.h"
#include "System1.h"
#include "StateTask.h"
*/

typedef struct ParamList
{
  uint32_t operation;
  union
  {
    uint32_t    uParam;
    int32_t     iParam;
    float       fParam;
    void *      pParam;
  }param1;
  
  union
  {
    uint32_t    uParam;
    int32_t     iParam;
    float       fParam;
    void *      pParam;
  }param2;
  
  union
  {
    uint32_t    uParam;
    int32_t     iParam;
    float       fParam;
    void *      pParam;
  }param3;
  
  union
  {
    uint32_t    uParam;
    int32_t     iParam;
    float       fParam;
    void *      pParam;
  }param4;
  
  union
  {
    uint32_t    uParam;
    int32_t     iParam;
    float       fParam;
    void *      pParam;
  }param5;
}ParamList;


typedef enum
{
  SetButton,
  Writedata,
  WriteString,
  WriteValue,
  WriteChar,
  ClrButtonMenu
}ActionFunctions;

struct GUI_Message{
	uint16_t op;
	uint16_t num;
	char heading[12];
};

enum cmdfield{
	changestate,
	writedata
};


void GUI_Start(void);
void DrawGrid(void);


#endif //GUITASK_H