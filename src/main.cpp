#include <Arduino.h>
#include <ml_tcc.h>
#include <ml_eic.h>
#include <ml_encoder.hpp>
#include <ml_clocks.h>
#include <TendonMotor.h>
#include <ml_dmac.h>
#include <ml_spi_common.h>
#include <ml_sercom_1.h>
#include <stdbool.h>

/// @brief  SPI STUFF
static DmacDescriptor base_descriptor[3] __attribute__((aligned(16)));
static volatile DmacDescriptor wb_descriptor[3] __attribute__((aligned(16)));

// allocated space for RX and TX buffers
#define SPI_RX_BUFFER_LEN 15
volatile uint8_t spi_rx_buffer[SPI_RX_BUFFER_LEN] = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

#define SPI_TX_BUFFER_LEN 15
volatile uint8_t spi_tx_buffer[SPI_TX_BUFFER_LEN] =
    {
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
};

char serial_buf[SPI_RX_BUFFER_LEN];

// create SPI object
ml_spi_s spi_s = sercom1_spi_dmac_slave_prototype;

// get DMAC channel numbers for rx and tx
const uint8_t rx_dmac_chnum = spi_s.rx_dmac_s.ex_chnum;
const uint8_t tx_dmac_chnum = spi_s.tx_dmac_s.ex_chnum;

void dstack_a_init(void)
{
  ML_SET_GCLK7_PCHCTRL(TCC0_GCLK_ID);

  TCC_DISABLE(TCC0);
  TCC_SWRST(TCC0);
  TCC_sync(TCC0);

  TCC0->CTRLA.reg =
      (TCC_CTRLA_PRESCALER_DIV2 |
       TCC_CTRLA_PRESCSYNC_PRESC);

  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;

  TCC_set_period(TCC0, 6000);

  // default output matrix configuration (pg. 1829)
  TCC0->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(0x00);

  for (uint8_t i = 0; i < 6; i++)
  {
    TCC0->CC[i].reg |= TCC_CC_CC(6000 / 2);
  }

  /*
   * Peripheral function "F"
   *
   * CC0 -> PC16 (D25),
   * CC1 -> PC17 (D24)
   * CC2 -> PC18 (D2)
   * CC3 -> PC19 (D3)
   * CC4 -> PC20 (D4)
   * CC5 -> PC21 (D5)
   */
}

// create bunch of tendons
#define NUM_TENDONS 7

int16_t target_motor_angles[NUM_TENDONS] = {
    0, 0, 0, 0, 0, 0, 0};

TendonController tendons[NUM_TENDONS] = {
    TendonController("motor 1"),
    TendonController("motor 2"),
    TendonController("motor 3"),
    TendonController("motor 4"),
    TendonController("motor 5"),
    TendonController("motor 6"),
    TendonController("motor 7")};

void attach_tendons()
{
  // motor 1
  tendons[0].Attach_Drive_Pin(PORT_GRP_C, 20, PF_F, 4);
  tendons[0].Attach_Direction_Pin(PORT_GRP_B, 16, PF_B);
  tendons[0].Attach_EncA_Pin(PORT_GRP_C, 13, PF_A);
  tendons[0].Attach_EncB_Pin(PORT_GRP_C, 12, PF_A);

  // motor 2
  tendons[1].Attach_Drive_Pin(PORT_GRP_C, 21, PF_F, 5);
  tendons[1].Attach_Direction_Pin(PORT_GRP_B, 17, PF_B);
  tendons[1].Attach_EncA_Pin(PORT_GRP_C, 15, PF_A);
  tendons[1].Attach_EncB_Pin(PORT_GRP_C, 14, PF_A);

  // motor 3
  tendons[2].Attach_Drive_Pin(PORT_GRP_C, 16, PF_F, 0);
  tendons[2].Attach_Direction_Pin(PORT_GRP_B, 20, PF_B);
  tendons[2].Attach_EncA_Pin(PORT_GRP_C, 11, PF_A);
  tendons[2].Attach_EncB_Pin(PORT_GRP_C, 10, PF_A);

  // motor 4
  tendons[3].Attach_Drive_Pin(PORT_GRP_C, 17, PF_F, 1);
  tendons[3].Attach_Direction_Pin(PORT_GRP_B, 21, PF_B);
  tendons[3].Attach_EncA_Pin(PORT_GRP_C, 7, PF_A);
  tendons[3].Attach_EncB_Pin(PORT_GRP_C, 6, PF_A);

  // motor 5
  tendons[4].Attach_Drive_Pin(PORT_GRP_C, 19, PF_F, 3);
  tendons[4].Attach_Direction_Pin(PORT_GRP_C, 22, PF_B);
  tendons[4].Attach_EncA_Pin(PORT_GRP_C, 4, PF_A);
  tendons[4].Attach_EncB_Pin(PORT_GRP_C, 5, PF_A);

  // motor 6
  tendons[5].Attach_Drive_Pin(PORT_GRP_C, 18, PF_F, 2);
  tendons[5].Attach_Direction_Pin(PORT_GRP_C, 23, PF_B);
  tendons[5].Attach_EncA_Pin(PORT_GRP_A, 23, PF_A);
  tendons[5].Attach_EncB_Pin(PORT_GRP_D, 8, PF_A);

  // motor 7
  tendons[6].Attach_Drive_Pin(PORT_GRP_A, 12, PF_F, 6);
  tendons[6].Attach_Direction_Pin(PORT_GRP_B, 24, PF_B);
  tendons[6].Attach_EncA_Pin(PORT_GRP_C, 0, PF_A);
  tendons[6].Attach_EncB_Pin(PORT_GRP_C, 1, PF_A);
}


void uart_controlled()
{
  if (Serial.available() >= SPI_RX_BUFFER_LEN)
  {
    Serial.println("Got data..");
    Serial.readBytes(serial_buf, SPI_RX_BUFFER_LEN);

    // if first byte is not a zero then we need to reset an encoders position
    if (serial_buf[0] != 0)
    {
      tendons[uint8_t(serial_buf[0]) - 1].Reset_Encoder_Zero();
    }

    target_motor_angles[0] = int16_t(serial_buf[1] << 8 | serial_buf[2]);
    target_motor_angles[1] = int16_t(serial_buf[3] << 8 | serial_buf[4]);
    target_motor_angles[2] = int16_t(serial_buf[5] << 8 | serial_buf[6]);
    target_motor_angles[3] = int16_t(serial_buf[7] << 8 | serial_buf[8]);
    target_motor_angles[4] = int16_t(serial_buf[9] << 8 | serial_buf[10]);
    target_motor_angles[5] = int16_t(serial_buf[11] << 8 | serial_buf[12]);
    target_motor_angles[6] = int16_t(serial_buf[13] << 8 | serial_buf[14]);

    // debugging print the angles
    for (int i = 0; i < NUM_TENDONS; i++)
    {
      Serial.print("Motor: ");
      Serial.print(target_motor_angles[i]);
      Serial.print(" ");
    }
  }
}

void setup()
{
  // start serial comm for debugging
  Serial.begin(115200);

  Serial.println("Starting");

  // start clocks
  MCLK_init();
  GCLK_init();

  // start the encoders
  eic_init();
  encoder_extint_init();
  eic_enable();

  // init TCC0 timer
  dstack_a_init();
  TCC_ENABLE(TCC0);
  TCC_sync(TCC0);

  // attach pins to tendon object
  attach_tendons();

  // intialize objects
  for (int i = 0; i < NUM_TENDONS; i++)
  {
    tendons[i].init_peripheral();
    tendons[i].Set_Direction(OFF);
    tendons[i].Set_PID_Param(900, 0, 10);
  }

  // good measure why not start the TCC0 again..
  TCC_ENABLE(TCC0);
  TCC_sync(TCC0);

  /**
   * SPI STUFF
   */
  // start the DMAC
  DMAC_init(&base_descriptor[0], &wb_descriptor[0]);

  // enable the SERCOM1 pad for SPI mode
  sercom1_spi_init(OPMODE_SLAVE);

  // setup DMAC for receiving data, pointing where data should be stored
  spi_s.rx_dmac_s.ex_ptr = &spi_rx_buffer[0];
  spi_s.rx_dmac_s.ex_len = SPI_RX_BUFFER_LEN;
  spi_dmac_rx_init(&spi_s.rx_dmac_s, SERCOM1, &base_descriptor[rx_dmac_chnum]);

  // setup DMAC for transmitting data, pointing where data should be sent from
  spi_s.tx_dmac_s.ex_ptr = &spi_tx_buffer[0];
  spi_s.tx_dmac_s.ex_len = SPI_TX_BUFFER_LEN;
  spi_dmac_tx_init(&spi_s.tx_dmac_s, SERCOM1, &base_descriptor[tx_dmac_chnum]);

  // enable DMAC and turn respective channels on
  ML_DMAC_ENABLE();
  ML_DMAC_CHANNEL_ENABLE(rx_dmac_chnum);
  ML_DMAC_CHANNEL_ENABLE(tx_dmac_chnum);

  // enable spi on SERCOM1 pad
  spi_enable(SERCOM1);
  spi_reciever_enable(SERCOM1);
}

// when select pin has been pulled low this means the master wants to communicat
_Bool ssl_intflag = false;
void SERCOM1_3_Handler(void)
{
  ssl_intflag = true;
  ML_SERCOM_SPI_SSL_CLR_INTFLAG(SERCOM1);
}

// interrupt for reciever DMAC
// when transfer is complete this is called
_Bool dmac_rx_intflag = false;
void DMAC_0_Handler(void)
{
  if (ML_DMAC_CHANNEL_TCMPL_INTFLAG(rx_dmac_chnum))
  {
    ML_DMAC_CHANNEL_CLR_TCMPL_INTFLAG(rx_dmac_chnum);
    dmac_rx_intflag = true;

    // check if we need to reset an encoder zero
    if (spi_rx_buffer[0] != 0)
    {
      tendons[spi_rx_buffer[0] - 1].Reset_Encoder_Zero();
    }

    // set the new angles from commanded
    target_motor_angles[0] = int16_t(spi_rx_buffer[1] << 8 | spi_rx_buffer[2]);
    target_motor_angles[1] = int16_t(spi_rx_buffer[3] << 8 | spi_rx_buffer[4]);
    target_motor_angles[2] = int16_t(spi_rx_buffer[5] << 8 | spi_rx_buffer[6]);
    target_motor_angles[3] = int16_t(spi_rx_buffer[7] << 8 | spi_rx_buffer[8]);
    target_motor_angles[4] = int16_t(spi_rx_buffer[9] << 8 | spi_rx_buffer[10]);
    target_motor_angles[5] = int16_t(spi_rx_buffer[11] << 8 | spi_rx_buffer[12]);
    target_motor_angles[6] = int16_t(spi_rx_buffer[13] << 8 | spi_rx_buffer[14]);
  }
}

// iterrupt for transmitter DMAC
// when transfer is complete this is called
_Bool dmac_tx_intflag = false;
void DMAC_1_Handler(void)
{
  if (ML_DMAC_CHANNEL_TCMPL_INTFLAG(tx_dmac_chnum))
  {
    ML_DMAC_CHANNEL_CLR_TCMPL_INTFLAG(tx_dmac_chnum);
    dmac_tx_intflag = true;
  }
}


void loop()
{
  // to test
  // uart_controlled();

  // set the target angle
  tendons[0].Set_Angle(target_motor_angles[0]);
  tendons[1].Set_Angle(target_motor_angles[1]);
  tendons[2].Set_Angle(target_motor_angles[2]);
  tendons[3].Set_Angle(target_motor_angles[3]);
  tendons[4].Set_Angle(target_motor_angles[4]);
  tendons[5].Set_Angle(target_motor_angles[5]);
  tendons[6].Set_Angle(target_motor_angles[6]);
}


//-----------------------------------------------------------------
// setting up interrupts
/*
 * M0:
 *      enca: D40 --> PC13 --> EXTINT[13]
 *      encb: D41 --> PC12 --> EXTINT[12]
 * M1:
 *      enca: D42 --> PC15 --> EXTINT[15]
 *      encb: D43 --> PC14 --> EXTINT[14]
 * M2:
 *      enca: D44 --> PC11 --> EXTINT[11]
 *      encb: D45 --> PC10 --> EXTINT[10]
 * M3:
 *      enca: D46 --> PC06 --> EXTINT[6]
 *      encb: D47 --> PC07 --> EXTINT[9]
 * M4:
 *      enca: D48 --> PC04 --> EXTINT[4]
 *      encb: D49 --> PC05 --> EXTINT[5]
 * M5:
 *      enca: D30 --> PA23 --> EXTINT[7]
 *      encb: D51 --> PD08 --> EXTINT[3]
 * M6:
 *      enca: A3 --> PC00 --> EXTINT[0]
 *      encb: A4 --> PC01 --> EXTINT[1]
 */

// M0
//  *      enca: D40 --> PC13 --> EXTINT[13]
//  *      encb: D41 --> PC12 --> EXTINT[12]
void EIC_13_Handler(void)
{
  EIC_CLR_INTFLAG(13);
  tendons[0].encoder_ISR();
}
void EIC_12_Handler(void)
{
  EIC_CLR_INTFLAG(12);
  tendons[0].encoder_ISR();
}

// M1
//   *      enca: D42 --> PC15 --> EXTINT[15]
//   *      encb: D43 --> PC14 --> EXTINT[14]
void EIC_15_Handler(void)
{
  EIC_CLR_INTFLAG(15);
  tendons[1].encoder_ISR();
}
void EIC_14_Handler(void)
{
  EIC_CLR_INTFLAG(14);
  tendons[1].encoder_ISR();
}

// M2
//  *      enca: D44 --> PC11 --> EXTINT[11]
//  *      encb: D45 --> PC10 --> EXTINT[10]
void EIC_11_Handler(void)
{
  EIC_CLR_INTFLAG(11);
  tendons[2].encoder_ISR();
}
void EIC_10_Handler(void)
{
  EIC_CLR_INTFLAG(10);
  tendons[2].encoder_ISR();
}

// M3
//  *      enca: D46 --> PC06 --> EXTINT[6]
//  *      encb: D47 --> PC07 --> EXTINT[9]
void EIC_6_Handler(void)
{
  EIC_CLR_INTFLAG(6);
  tendons[3].encoder_ISR();
}
void EIC_9_Handler(void)
{
  EIC_CLR_INTFLAG(9);
  tendons[3].encoder_ISR();
}

// M4
//  *      enca: D48 --> PC04 --> EXTINT[4]
//  *      encb: D49 --> PC05 --> EXTINT[5]
void EIC_4_Handler(void)
{
  EIC_CLR_INTFLAG(4);
  tendons[4].encoder_ISR();
}
void EIC_5_Handler(void)
{
  EIC_CLR_INTFLAG(5);
  tendons[4].encoder_ISR();
}

// M5
//  *      enca: D30 --> PA23 --> EXTINT[7]
//  *      encb: D51 --> PD08 --> EXTINT[3]
void EIC_7_Handler(void)
{
  EIC_CLR_INTFLAG(7);
  tendons[5].encoder_ISR();
}
void EIC_3_Handler(void)
{
  EIC_CLR_INTFLAG(3);
  tendons[5].encoder_ISR();
}

// M6
void EIC_1_Handler(void)
{
  EIC_CLR_INTFLAG(1);
  tendons[6].encoder_ISR();
}
void EIC_0_Handler(void)
{
  EIC_CLR_INTFLAG(0);
  tendons[6].encoder_ISR();
}
