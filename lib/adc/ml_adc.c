/*
 * Author: Ben Westcott
 * Date created: 8/1/23
 */

#include <ml_adc.h>

void ADC_sync(Adc *instance) 
{ 
    while(instance->SYNCBUSY.reg); 
}
