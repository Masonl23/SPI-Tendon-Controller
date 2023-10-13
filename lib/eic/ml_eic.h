/*
 * Author: Ben Westcott
 * Date created: 7/31/23
 */

#ifndef ML_EIC_H
#define ML_EIC_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define EIC_CLR_INTFLAG(n) (EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT(n))

void eic_init(void);
void eic_enable(void);

void hardware_int_init(void);

#ifdef __cplusplus
}
#endif

#endif // ML_EIC_H