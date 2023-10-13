/*
 * Author: Ben Westcott, Jayson De La Vega
 * Date created: 8/11/23
 *
 * Editted: Mason Lopez 10/13/2023
 */

#include <ml_spi_common.h>

/**
 * enables SPI communication for the given Sercom pad,
 * hangs until the system acknowledges the change.
 */
void spi_enable(Sercom *coms)
{
    coms->SPI.CTRLA.bit.ENABLE = 0x01;
    while (coms->SPI.SYNCBUSY.bit.ENABLE != 0U)
    {
        /* Wait for sync */
    }
}

/**
 * diables SPI communication for the gicen Sercom pad,
 * hangs until the system acknowledges the change.
 */
void spi_disable(Sercom *coms)
{
    coms->SPI.CTRLA.bit.ENABLE = 0x00;
    while (coms->SPI.SYNCBUSY.bit.ENABLE != 0U)
    {
        /* Wait for sync */
    }
}

/**
 * Software reset on the sercom pad.
 */
void spi_swrst(Sercom *coms)
{
    coms->SPI.CTRLA.bit.SWRST = 0x01;
    while (coms->SPI.SYNCBUSY.bit.SWRST != 0U)
    {
        /* Wait for sync */
    }
}

/**
 * Enables recieving data for SPI on sercom pad
 */
void spi_reciever_enable(Sercom *coms)
{
    coms->SPI.CTRLB.bit.RXEN = true;
    while (coms->SPI.SYNCBUSY.bit.CTRLB != 0U)
    {
        /* Wait for sync */
    }
}

/**
 * Disables recieving data for SPI on sercom pad.
 */
void spi_reciever_disable(Sercom *coms)
{
    coms->SPI.CTRLB.bit.RXEN = false;
    while (coms->SPI.SYNCBUSY.bit.CTRLB != 0U)
    {
        /* Wait for sync */
    }
}

uint32_t spi_read32b(Sercom *coms, uint32_t data)
{
    while (coms->SPI.INTFLAG.bit.DRE == 0)
    {
        /* Wait for data ready flag */
    }
    coms->SPI.DATA.reg = data;
    while (coms->SPI.INTFLAG.bit.RXC == 0)
    {
        /* Wait for recieve complete intflag */
    }
    return (uint32_t)coms->SPI.DATA.reg;
}

uint8_t spi_read8b(Sercom *coms, uint8_t data)
{
    while (coms->SPI.INTFLAG.bit.DRE == 0)
    {
        /* Wait for data ready flag */
    }
    coms->SPI.DATA.reg = data;
    while (coms->SPI.INTFLAG.bit.RXC == 0)
    {
        /* Wait for recieve complete intflag */
    }
    return (uint8_t)coms->SPI.DATA.reg;
}

/**
 * Setups DMAC for transmitting
*/
void spi_dmac_tx_init(
    ml_dmac_s *dmac_s,
    Sercom *coms_inst,
    DmacDescriptor *cpy)
{
    // get channel number for transfer channel
    ml_dmac_chnum_t chnum = dmac_s->ex_chnum;

    // init DMAC
    DMAC_channel_init(
        chnum,
        dmac_s->chan_settings,
        dmac_s->chan_prilvl);

    uint32_t btsize = DMAC_extract_btsize(dmac_s->descriptor_settings);

    // setup of descriptor 
    DMAC_descriptor_init(
        dmac_s->descriptor_settings,
        dmac_s->ex_len,
        (uint32_t)dmac_s->ex_ptr + (btsize * dmac_s->ex_len),
        (uint32_t)&coms_inst->SPI.DATA.reg,
        (uint32_t)cpy, cpy);

    // setup DMAC interrupt on data transfer and set priority
    DMAC_channel_intenset(
        chnum,
        dmac_s->irqn,
        dmac_s->intmsk,
        dmac_s->irqn_prilvl);
}

/**
 * Sets up DMAC for recieving
*/
void spi_dmac_rx_init(
    ml_dmac_s *dmac_s,
    Sercom *coms_inst,
    DmacDescriptor *cpy)
{
    // get channel number
    ml_dmac_chnum_t chnum = dmac_s->ex_chnum;

    // init channel and set priority
    DMAC_channel_init(
        chnum,
        dmac_s->chan_settings,
        dmac_s->chan_prilvl);

    uint32_t btsize = DMAC_extract_btsize(dmac_s->descriptor_settings);

    // set descriptor
    DMAC_descriptor_init(
        dmac_s->descriptor_settings,
        dmac_s->ex_len,
        (uint32_t)&coms_inst->SPI.DATA.reg,
        (uint32_t)dmac_s->ex_ptr + (btsize * dmac_s->ex_len),
        (uint32_t)cpy, cpy);

    // setup interrupts for DMAC
    DMAC_channel_intenset(
        chnum,
        dmac_s->irqn,
        dmac_s->intmsk,
        dmac_s->irqn_prilvl);
}
