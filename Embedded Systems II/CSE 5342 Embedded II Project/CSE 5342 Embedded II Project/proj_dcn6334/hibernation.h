#ifndef HIBERNATION_H_
#define HIBERNATION_H_

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initHibernationModule();
void hibernate();
bool checkIfConfigured();
bool rtcCausedWakeUp();
bool wakePinCausedWakeUp();
void waitUntilWriteComplete();

#endif
