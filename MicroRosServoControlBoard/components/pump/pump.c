/**
 * @file pump.c
 * @brief Implementation of the vacuum pump actuation, see pump.h.
 */
#include "pump.h"
#include "servo.h"
#include "servo_config.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "PUMP";

/** Last commanded state, cached purely for the /telemetry array (the GPIO/PWM
 *  output itself is the source of truth for actual hardware state). */
static bool g_pump_on = false;

#if CONFIG_VACUUM_ACTUATION_PWM

/**
 * @brief Map the logical on/off state onto a calibrated pulse width.
 *
 * `on == true` selects vacuum_pwm_max_us and `false` selects
 * vacuum_pwm_min_us, unless vacuum_pwm_invert swaps that assignment -- lets
 * an operator correct the physical mounting orientation of the servo
 * (which endpoint the pump ends up in) via /servo_config without touching
 * the calibrated endpoints themselves.
 */
static int pump_pulse_for(bool on)
{
    bool use_max = on ^ (ServoConfig_Get_VacuumPwmInvert() != 0);
    return use_max ? ServoConfig_Get_VacuumPwmMaxUs() : ServoConfig_Get_VacuumPwmMinUs();
}

#endif // CONFIG_VACUUM_ACTUATION_PWM

void Pump_Init(void)
{
#if !CONFIG_VACUUM_ACTUATION_PWM
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_VACUUM_RELAY_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
#endif

    Pump_Set(false);

#if CONFIG_VACUUM_ACTUATION_PWM
    ESP_LOGI(TAG, "Pump_Init done (mode=PWM, GPIO%d)", CONFIG_VACUUM_RELAY_GPIO);
#else
    ESP_LOGI(TAG, "Pump_Init done (mode=digital, GPIO%d)", CONFIG_VACUUM_RELAY_GPIO);
#endif
}

void Pump_Set(bool on)
{
#if CONFIG_VACUUM_ACTUATION_PWM
    Servo_Set_Pulse_Us(SERVO_CHANNEL_PUMP, pump_pulse_for(on));
#else
    gpio_set_level(CONFIG_VACUUM_RELAY_GPIO, on ? 1 : 0);
#endif
    g_pump_on = on;
}

void Pump_Refresh_Output(void)
{
#if CONFIG_VACUUM_ACTUATION_PWM
    Servo_Set_Pulse_Us(SERVO_CHANNEL_PUMP, pump_pulse_for(g_pump_on));
#endif
    // Digital mode has no calibration to re-apply -- the GPIO already
    // reflects g_pump_on.
}

bool Pump_Get_On(void) { return g_pump_on; }
