/*
 * Author: Ben Westcott
 * Date created: 7/31/23
 */

#include <ml_eic.h>
#include <ml_clocks.h>
#include <ml_port.h>

void eic_init(void)
{
    //ML_SET_GCLK7_PCHCTRL(EIC_GCLK_ID);

    EIC->CTRLA.reg &= ~EIC_CTRLA_ENABLE;
    EIC->CTRLA.reg |= EIC_CTRLA_SWRST;
    while(EIC->SYNCBUSY.bit.ENABLE & EIC->SYNCBUSY.bit.SWRST);

    // CLK_ULP32K = 0x1, else GCLK
    EIC->CTRLA.reg |= EIC_CTRLA_CKSEL;

    // Set all EXTINTs to trigger syncrhonously
    // I believe this is default, i.e., this register is 
    // initialized to a value of 0x0
    // EIC->ASYNCH.reg &= ~EIC_ASYNCH_ASYNCH(0);
    
}

void eic_enable(void)
{
    EIC->CTRLA.bit.ENABLE = 0x01;
    while(EIC->SYNCBUSY.bit.ENABLE);
}

/*
 * PA16 --> EXTINT[0]
 *   - IB: D0
 *   - GC: D37
 */
/*
 * PA02 --> EXTINT[2]
 *   - IB: A0
 *   - GC: A0
 */
const ml_pin_settings ml_hardware_int_pin = 
{
    PORT_GRP_A, 2, PF_A, PP_EVEN, INPUT_PULL_DOWN, DRIVE_OFF
};

void hardware_int_init(void)
{
    // Rising edge detection
    EIC->CONFIG[0].bit.SENSE2 = 0x01;
    // Take multiple samples
    // Disabled because we will debounce the input
    //EIC->CONFIG[0].bit.FILTEN0 = 0x01;

    // debounce sample for 50ms? Seems reasonable ig
    
    EIC->DEBOUNCEN.reg |= (1 << EIC_DEBOUNCEN_DEBOUNCEN(2));
    // sampled on CLK_ULP32K = 0x01, else CLK_EIC
    EIC->DPRESCALER.bit.TICKON = 0x01;
    // F//128 ==> 0x06
    EIC->DPRESCALER.bit.PRESCALER0 = 0x06;
    // Sample 3 times per state change
    EIC->DPRESCALER.bit.STATES0 = 0x00;

    EIC->INTENSET.reg |= (1 << EIC_INTENSET_EXTINT(2));
    NVIC_EnableIRQ(EIC_2_IRQn);
    NVIC_SetPriority(EIC_2_IRQn, 0);

    peripheral_port_init(&ml_hardware_int_pin);
}


