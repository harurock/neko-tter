/*
 * event.h
 *
 *  Created on: 2013/01/10
 *      Author: seigo13
 */

#ifndef APP_EVENT_H_
#define APP_EVENT_H_

#include "ToCoNet_event.h"

typedef enum
{
  E_EVENT_APP_BASE = ToCoNet_EVENT_APP_BASE,
  E_EVENT_APP_TX_COMPLETE
} teEventApp;

// STATE MACHINE
typedef enum
{
  E_STATE_APP_BASE = ToCoNet_STATE_APP_BASE,
  E_STATE_APP_NW_BOOT,
  E_STATE_APP_RUNNING,
  E_STATE_APP_WAIT_TX,
  E_STATE_APP_SLEEP
} teStateApp;

#endif /* EVENT_H_ */
