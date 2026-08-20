/**
 * @file servo.c
 * @brief Implementation of the hardware-PWM servo driver, see servo.h.
 */
#include "servo.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "SERVO";

/** Standard RC-servo PWM frame: 50Hz -> 20000us period. */
#define SERVO_PWM_FREQ_HZ     50
/** 13-bit duty resolution (8192 steps over 20ms = ~2.44us/step), a widely
 *  used safe value for 50Hz LEDC servo PWM on the ESP32's clock sources. */
#define SERVO_DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define SERVO_DUTY_MAX        ((1 << 13) - 1)
#define SERVO_PERIOD_US       (1000000 / SERVO_PWM_FREQ_HZ)

/** Last pulse width applied per channel, indexed by servo_channel_t. Read
 *  back by the telemetry publisher, see Servo_Get_Pulse_Us(). */
static int g_last_pulse_us[2] = { 0, 0 };

/**
 * @brief Map a servo_channel_t to its LEDC channel and GPIO.
 * @param channel    Logical servo channel.
 * @param out_ledc   Receives the LEDC channel to use.
 * @param out_gpio   Receives the GPIO pin to use.
 */
static void servo_channel_to_ledc(servo_channel_t channel, ledc_channel_t *out_ledc, int *out_gpio)
{
    if (channel == SERVO_CHANNEL_LIFT)
    {
        *out_ledc = LEDC_CHANNEL_0;
        *out_gpio = CONFIG_LIFT_SERVO_GPIO;
    }
    else
    {
        *out_ledc = LEDC_CHANNEL_1;
        *out_gpio = CONFIG_CAMERA_SERVO_GPIO;
    }
}

void Servo_Init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (servo_channel_t ch = SERVO_CHANNEL_LIFT; ch <= SERVO_CHANNEL_CAMERA; ch++)
    {
        ledc_channel_t ledc_ch;
        int gpio;
        servo_channel_to_ledc(ch, &ledc_ch, &gpio);

        ledc_channel_config_t ch_conf = {
            .gpio_num   = gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ledc_ch,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    // Start both servos at their configured neutral/stop pulse -- the lift
    // servo must never power up spinning, and the camera servo has no
    // "neutral" of its own, so the lift stop value is a safe common default
    // until the first real command arrives.
    Servo_Set_Pulse_Us(SERVO_CHANNEL_LIFT, CONFIG_LIFT_PWM_STOP_US);
    Servo_Set_Pulse_Us(SERVO_CHANNEL_CAMERA, CONFIG_LIFT_PWM_STOP_US);

    ESP_LOGI(TAG, "Servo_Init done (lift=GPIO%d, camera=GPIO%d)",
             CONFIG_LIFT_SERVO_GPIO, CONFIG_CAMERA_SERVO_GPIO);
}

void Servo_Set_Pulse_Us(servo_channel_t channel, int pulse_us)
{
    // Hard safety clamp -- applies unconditionally, including to jog
    // commands that intentionally skip calibration/endstop checks.
    if (pulse_us < CONFIG_SERVO_PULSE_ABS_MIN_US) pulse_us = CONFIG_SERVO_PULSE_ABS_MIN_US;
    if (pulse_us > CONFIG_SERVO_PULSE_ABS_MAX_US) pulse_us = CONFIG_SERVO_PULSE_ABS_MAX_US;

    ledc_channel_t ledc_ch;
    int gpio;
    servo_channel_to_ledc(channel, &ledc_ch, &gpio);

    uint32_t duty = (uint32_t)(((int64_t)pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_ch);

    g_last_pulse_us[channel] = pulse_us;
}

int Servo_Get_Pulse_Us(servo_channel_t channel)
{
    return g_last_pulse_us[channel];
}
