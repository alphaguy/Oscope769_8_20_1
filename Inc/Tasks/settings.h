/*
 * settings.h
 *
 *  Created on: May 15, 2016
 *      Author: Aaron
 */

#ifndef INC_UTIL_SETTINGS_H_
#define INC_UTIL_SETTINGS_H_


/*
enum {
	vrange0,vrange1,vrange2,vrange3,
	vrange4,vrange5,vrange6,vrange7,
	vrange8,vrange9
};

enum {
	trange0,trange1,trange2,trange3,
	trange4,trange5,trange6,trange7,
	trange8,trange9,trange10,trange11
};
*/

enum
{
  ExternalTrigger,
  Channel1Trigger,
  Channel2Trigger,
  AutoTrigger
};


struct ChannelTypeDef{
	float vrange;
	GUI_COLOR color;
	int active;
	float offset;
};


#endif /* INC_UTIL_SETTINGS_H_ */
