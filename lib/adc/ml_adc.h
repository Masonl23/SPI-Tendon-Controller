/*
 * Author: Ben Westcott
 * Date created: 3/6/23
 */

#ifndef ML_ADC_H
#define ML_ADC_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ML_ADC0_AIN0 0x0
#define ML_ADC0_AIN1 0x1
#define ML_ADC0_AIN2 0x2
#define ML_ADC0_AIN3 0x3
#define ML_ADC0_AIN4 0x4
#define ML_ADC0_AIN5 0x5
#define ML_ADC0_AIN6 0x6

#define ML_ADC1_AIN0 0x0
#define ML_ADC1_AIN1 0x1
#define ML_ADC1_AIN2 0x2
#define ML_ADC1_AIN3 0x3
#define ML_ADC1_AIN4 0x4
#define ML_ADC1_AIN5 0x5
#define ML_ADC1_AIN6 0x6

#define ML_ADC_ENABLE(instance) (instance->CTRLA.reg |= ADC_CTRLA_ENABLE)

#define ML_ADC_DISABLE(instance) (instance->CTRLA.reg &= ~ADC_CTRLA_ENABLE)

#define ML_ADC_SWRST(instance) (instance->CTRLA.reg |= ADC_CTRLA_SWRST)

#define ML_ADC_SWTRIG_START(instance) (instance->SWTRIG.reg |= ADC_SWTRIG_START)

void ADC_sync(Adc *instance);

#ifdef __cplusplus
}
#endif

#endif // ML_ADC_H