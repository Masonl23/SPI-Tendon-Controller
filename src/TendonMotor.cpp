#include <TendonMotor.h>
#include <ml_clocks.h>

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// create tendon controller
// TendonController::TendonController(uint8_t ccChan, ml_pin phasePin, ml_pin pwmPin, ml_pin encA, ml_pin encB, String name)
TendonController::TendonController(String name)
{
    // // set the TCC channel
    m_pwm_channel = TCC0;

    // set PID to default
    m_kp = 1;
    m_kd = 0;
    m_ki = 0;
    m_error_integral = 0;
    m_error_prev = 0;

    // name of this tendon
    m_name = name;
}

void TendonController::Attach_Drive_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc, uint8_t cc_channel)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_drive = {portGroup, pin, portFunc, parity, OUTPUT_PULL_DOWN, DRIVE_ON};
    m_pwm_CC = cc_channel;
}

void TendonController::Attach_Direction_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_phase = {portGroup, pin, portFunc, parity, OUTPUT_PULL_DOWN, DRIVE_ON};
}

void TendonController::Attach_EncA_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_encoder_a = {portGroup, pin, portFunc, parity, INPUT_PULL_UP, DRIVE_OFF};
}

void TendonController::Attach_EncB_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_encoder_b = {portGroup, pin, portFunc, parity, INPUT_PULL_UP, DRIVE_OFF};
}

/**
 * Starts the tcc controller
 */
void TendonController::_init_tcc()
{
    ML_SET_GCLK1_PCHCTRL(TCC0_GCLK_ID);

    TCC_DISABLE(TCC0);
    TCC_SWRST(TCC0);
    TCC_sync(TCC0);

    TCC0->CTRLA.reg =
        (TCC_CTRLA_PRESCALER_DIV2 |
         TCC_CTRLA_PRESCSYNC_PRESC);

    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;

    TCC_set_period(TCC0, m_tcc_freq);

    // default output matrix configuration (pg. 1829)
    TCC0->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(0x00);
    TCC0->CC[m_pwm_CC].reg |= TCC_CC_CC(m_tcc_freq / 2);

    TCC_ENABLE(TCC0);
    TCC_sync(TCC0);
}

void TendonController::encoder_ISR()
{
    static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    uint8_t a_phase = (uint8_t)(!logical_read(&m_encoder_a));
    uint8_t b_phase = (uint8_t)(!logical_read(&m_encoder_b));

    uint16_t current_encoded = (a_phase << 1) | b_phase;
    m_currentTicks += lookup_table[(m_lastTicks << 2) | current_encoded];
    m_lastTicks = current_encoded;
}

void TendonController::Set_EncA_Flag()
{
    m_encA_ticks++;
}

void TendonController::Set_EncB_Flag()
{
    m_encB_ticks++;
}

void TendonController::init_peripheral()
{
    peripheral_port_init(&m_encoder_a);
    peripheral_port_init(&m_encoder_b);
    peripheral_port_init(&m_phase);
    peripheral_port_init(&m_drive);
    logical_set(&m_phase);
}
// set motor duty cycle
void TendonController::Set_Duty_Cyle(uint16_t dutyCycle)
{
    // map pwm to freq range
    dutyCycle > m_tcc_freq ? dutyCycle = 100 : 0;
    uint16_t value = m_tcc_freq * (dutyCycle / 100.0);
    m_cur_pwm = value;
    // m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    TCC_sync(m_pwm_channel);
}

// set motor duty cycle
void TendonController::set_PWM_Freq(uint16_t pwmValue)
{
    // map pwm to freq range
    pwmValue > m_tcc_freq ? pwmValue = m_tcc_freq : pwmValue;
    m_cur_pwm = pwmValue;
    // m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(pwmValue);
    TCC_sync(m_pwm_channel);
}

// set PID parameters
void TendonController::Set_PID_Param(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

// set the direction motor turns
void TendonController::Set_Direction(Tendon_Direction dir)
{
    if (dir == OFF)
    {
        TCC0->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(0x00);
        TCC_sync(m_pwm_channel);
    }
    else if (dir == CW)
    {
        logical_set(&m_phase);
    }
    else
    {
        logical_unset(&m_phase);
    }
    m_direction = dir;
}

void TendonController::Toggle_Direction()
{
    switch (m_direction)
    {
    case OFF:
        return;
    case CW:
        Set_Direction(CCW);
        break;
    default:
        Set_Direction(CW);
        break;
    }
}

/**
 * Calibrates minimum PWM
 */
void TendonController::Calibrate_Min_PWM()
{

    int16_t lastTicks = m_currentTicks;

    // find min value
    uint16_t minPwm = 0;
    uint16_t avgCWPwm = 0;
    uint16_t timesSuccess = 0;
    ulong lastRun = 0;
    Set_Direction(CW);
    for (int i = 0; i < 5; i++)
    {
        timesSuccess++;
        while (lastTicks == m_currentTicks)
        {
            if (millis() - lastRun > 50)
            {
                minPwm += 50;
                if (minPwm >= m_tcc_freq)
                {
                    timesSuccess--;
                    break;
                }
                set_PWM_Freq(minPwm);
                lastRun = millis();
            }
        }
        lastTicks = m_currentTicks;
        avgCWPwm += minPwm;
        minPwm = 0;
        set_PWM_Freq(0);
    }
    avgCWPwm /= timesSuccess;

    set_PWM_Freq(0);
    Set_Direction(CCW);
    lastTicks = m_currentTicks;
    minPwm = 0;
    timesSuccess = 0;
    uint16_t avgCCWPwm = 0;
    for (int i = 0; i < 5; i++)
    {
        timesSuccess++;
        while (lastTicks == m_currentTicks)
        {
            if (millis() - lastRun > 50)
            {
                minPwm += 100;
                if (minPwm >= m_tcc_freq)
                {
                    timesSuccess--;
                    break;
                }
                set_PWM_Freq(minPwm);
                lastRun = millis();
            }
        }
        lastTicks = m_currentTicks;
        avgCCWPwm += minPwm;
        minPwm = 0;
        set_PWM_Freq(0);
    }

    avgCCWPwm /= timesSuccess;
    Set_Direction(OFF);
    Serial.print("Min pwmcw : ");
    Serial.println(avgCWPwm);
    Serial.print("Min pwmccw : ");
    Serial.println(avgCCWPwm);
    m_min_CW_PWM = avgCWPwm;
    m_min_CCW_PWM = avgCCWPwm;
    m_calibrated = true;
}

/**
 * Returns current angle of motor
 */
float TendonController::Get_Angle()
{
    return ((360.0 * m_currentTicks) / (m_cycles_per_rev * m_gear_ratio));
}

/**
 * Set target angle
 */
void TendonController::Set_Angle(float destAngle)
{
    // grab current time
    unsigned long curTime = micros();
    unsigned long deltaTime = curTime - m_prevPIDTime;

    // get number of encoders ticks needed to get to angle
    int32_t target_ticks = (destAngle * m_cycles_per_rev * m_gear_ratio) / 360.0;

    // calculate the error
    int32_t error = target_ticks - m_currentTicks;

    // derivative
    float derivative = (error - m_error_prev) / deltaTime;

    // integral
    m_error_integral += error * deltaTime;

    // calc control signal
    float sig = m_kp * error + m_kd * derivative + m_ki * m_error_integral;

    // set new pwm signal
    m_cur_pwm = (uint16_t)fabs(sig);

    // set direction
    m_direction = CW;
    if (sig < 0)
    {
        m_direction = CCW;
    }

    if (!m_calibrated)
    {
        m_cur_pwm = mapf(m_cur_pwm, 0, 6000, 2500, 6000);
    }
    else
    {
        uint16_t min = sig < 0 ? m_min_CCW_PWM : m_min_CW_PWM;
        m_cur_pwm = mapf(m_cur_pwm, 0, 6000, min, 6000);
    }

    if (m_cur_pwm > m_tcc_freq)
    {
        m_cur_pwm = m_tcc_freq;
    }

    // Set_Duty_Cyle(m_cur_pwm);
    set_PWM_Freq(m_cur_pwm);
    Set_Direction(m_direction);

    // store previous data
    m_prevPIDTime = curTime;
    m_error_prev = error;
}
