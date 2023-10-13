/*
 * Author: Ben Westcott
 * Date created: 8/1/23
 */

#include <ml_ac.h>

void AC_sync(void) 
{ 
    while(AC->SYNCBUSY.reg); 
}