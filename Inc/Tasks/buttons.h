/*
 * buttons.h
 *
 *  Created on: Nov 12, 2016
 *      Author: Aaron
 */

#ifndef UTIL_BUTTONS_H_
#define UTIL_BUTTONS_H_

#define totalstates 14

struct ButtonParams{
	int exists;
	int X1;
	int Y1;
	int X2;
	int Y2;
	GUI_COLOR BkColorUnPressed;
	GUI_COLOR BkColorPressed;
	GUI_COLOR TextColorUnPressed;
	GUI_COLOR TextColorPressed;
	char ButtonText[12];
};


void ButtonChangeState( int nextstate, char *heading);

void BUTTON0_My_Callback(WM_MESSAGE *pMsg);
void BUTTON1_My_Callback(WM_MESSAGE *pMsg);
void BUTTON2_My_Callback(WM_MESSAGE *pMsg);
void BUTTON3_My_Callback(WM_MESSAGE *pMsg);
void BUTTON4_My_Callback(WM_MESSAGE *pMsg);
void BUTTON5_My_Callback(WM_MESSAGE *pMsg);
void BUTTON6_My_Callback(WM_MESSAGE *pMsg);
void button_check(void *params);


void button_init(void);

#endif /* UTIL_BUTTONS_H_ */
