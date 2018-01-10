/*
 * settings.c
 *
 *  Created on: May 15, 2016
 *      Author: Aaron
 */

#include "oscope.h"


   
int TriggerMode = ExternalTrigger;


//set initialized values at startup
struct ChannelTypeDef
	Chan1 = {
		 .vrange = 3,
		 .color = GUI_RED,
		 .active = 1,
		 .offset = 1
	};
struct ChannelTypeDef
	Chan2 ={
		.vrange = 3,
		.color =GUI_GREEN,
		.active = 1,
		.offset = -2.9
	};


float curtrange = .0003, curvrange1 = 1, curvrange2 = 1;

modes runmode = testmode, prevmode = demomode;

