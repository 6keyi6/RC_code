#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H

#include "freertos.h"

#include "cmsis_os.h"

void Can1Receives(void const *argument);
void Can2Receives(void const *argument);

#endif
