/**
 * @file lift.c
 * @brief Implementation of the scissor-table lift state machine, see lift.h.
 */
#include "lift.h"
#include "servo.h"
#include "servo_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "LIFT";

/** Background task cycle time -- also the granularity of jog pulse timing. */
#define LIFT_TASK_PERIOD_MS 20

static volatile lift_state_t g_state = LIFT_STATE_IDLE;
static volatile bool g_endstop_up = false;
static volatile bool g_endstop_down = false;

/** Tick deadline for the current MOVING_UP/MOVING_DOWN move, compared
 *  against xTaskGetTickCount() to detect a jammed/missing endstop. */
static TickType_t g_move_deadline_tick = 0;

/** Set while a /scissor/jog pulse is in progress; the task loop counts this
 *  down and returns the servo to its stop pulse at zero (mirrors the
 *  beep component's Beep_On_Time() auto-off pattern). */
static volatile bool g_jogging = false;
static volatile int g_jog_remaining_ticks = 0;

/**
 * @brief Return the lift servo to its neutral/stop pulse.
 *
 * Used for every "at rest" transition (endstop reached, timeout, jog end,
 * already-there no-op). This deliberately keeps driving a valid pulse --
 * see the "no Servo_Disable()" note in servo.h for why the PWM signal must
 * never be cut entirely on this servo.
 */
static void lift_servo_rest(void)
{
    Servo_Set_Pulse_Us(SERVO_CHANNEL_LIFT, ServoConfig_Get_LiftPwmStopUs());
}

/**
 * @brief Sign of the raw pulse-width offset that physically drives the
 *        lift towards "up", per the runtime-configurable
 *        ServoConfig_Get_LiftDirectionUpIsIncrease() (Kconfig
 *        CONFIG_LIFT_DIRECTION_UP_IS_INCREASE only supplies its first-boot
 *        default -- see servo_config.h).
 */
static int lift_up_sign(void)
{
    return ServoConfig_Get_LiftDirectionUpIsIncrease() ? 1 : -1;
}

/**
 * @brief Start (or resume) driving towards the up endstop: apply the
 *        configured run pulse, (re)arm the timeout deadline and set
 *        LIFT_STATE_MOVING_UP.
 */
static void lift_start_moving_up(int offset_us)
{
    int pulse = ServoConfig_Get_LiftPwmStopUs() + lift_up_sign() * offset_us;
    Servo_Set_Pulse_Us(SERVO_CHANNEL_LIFT, pulse);
    g_move_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(ServoConfig_Get_LiftTimeoutMs());
    g_state = LIFT_STATE_MOVING_UP;
}

/** @brief Mirrors lift_start_moving_up() for the down endstop. */
static void lift_start_moving_down(int offset_us)
{
    int pulse = ServoConfig_Get_LiftPwmStopUs() - lift_up_sign() * offset_us;
    Servo_Set_Pulse_Us(SERVO_CHANNEL_LIFT, pulse);
    g_move_deadline_tick = xTaskGetTickCount() + pdMS_TO_TICKS(ServoConfig_Get_LiftTimeoutMs());
    g_state = LIFT_STATE_MOVING_DOWN;
}

/**
 * @brief Read both endstop GPIOs and update g_endstop_up/g_endstop_down.
 *
 * Always runs, regardless of state, so /telemetry reflects the real switch
 * state even when idle or jogging.
 */
static void lift_read_endstops(void)
{
    int raw_up = gpio_get_level(CONFIG_LIFT_ENDSTOP_UP_GPIO);
    int raw_down = gpio_get_level(CONFIG_LIFT_ENDSTOP_DOWN_GPIO);

    // Polarity is runtime-configurable (see ServoConfig_Get_LiftEndstopActiveLow(),
    // Kconfig only supplies the first-boot default) so it can be corrected
    // via /servo_config without a rebuild/reflash.
    if (ServoConfig_Get_LiftEndstopActiveLow())
    {
        g_endstop_up = (raw_up == 0);
        g_endstop_down = (raw_down == 0);
    }
    else
    {
        g_endstop_up = (raw_up != 0);
        g_endstop_down = (raw_down != 0);
    }
}

/**
 * @brief Background state-machine task: watches endstops/timeout while
 *        MOVING_UP/MOVING_DOWN, and counts down an active jog pulse.
 */
static void lift_task(void *arg)
{
    ESP_LOGI(TAG, "Start lift_task with core:%d", xPortGetCoreID());

    while (1)
    {
        lift_read_endstops();

        if (g_jogging)
        {
            if (g_jog_remaining_ticks > 0)
            {
                g_jog_remaining_ticks--;
            }
            else
            {
                lift_servo_rest();
                g_jogging = false;
                // Re-derive the resting state from the endstops now that
                // manual jogging has ended.
                if (g_endstop_up) g_state = LIFT_STATE_OPEN;
                else if (g_endstop_down) g_state = LIFT_STATE_CLOSED;
                else g_state = LIFT_STATE_IDLE;
            }
        }
        else if (g_state == LIFT_STATE_MOVING_UP)
        {
            if (g_endstop_up)
            {
                lift_servo_rest();
                g_state = LIFT_STATE_OPEN;
            }
            else if (xTaskGetTickCount() >= g_move_deadline_tick)
            {
                lift_servo_rest();
                g_state = LIFT_STATE_ERROR_TIMEOUT;
                ESP_LOGE(TAG, "Timeout while moving up, up endstop not reached");
            }
        }
        else if (g_state == LIFT_STATE_MOVING_DOWN)
        {
            if (g_endstop_down)
            {
                lift_servo_rest();
                g_state = LIFT_STATE_CLOSED;
            }
            else if (xTaskGetTickCount() >= g_move_deadline_tick)
            {
                lift_servo_rest();
                g_state = LIFT_STATE_ERROR_TIMEOUT;
                ESP_LOGE(TAG, "Timeout while moving down, down endstop not reached");
            }
        }
        else if (g_state == LIFT_STATE_OPEN)
        {
            // The spindle's thread is self-locking against normal loads on
            // the table, so this should be rare -- but if something (e.g. a
            // heavy enough load) does turn it while the PWM signal is off,
            // re-engage automatically and drive back to the endstop. Uses
            // its own (typically gentler) offset, not the full run offset,
            // since this is a small correction, not a deliberate auf/zu.
            if (!g_endstop_up) lift_start_moving_up(ServoConfig_Get_LiftPwmReengageOffsetUs());
        }
        else if (g_state == LIFT_STATE_CLOSED)
        {
            if (!g_endstop_down) lift_start_moving_down(ServoConfig_Get_LiftPwmReengageOffsetUs());
        }
        // IDLE / ERROR_TIMEOUT: nothing to do here, servo already sits at
        // its neutral pulse unless a jog is overriding it. ERROR_TIMEOUT
        // specifically does NOT auto-retry -- it means the endstop wasn't
        // reached within lift_timeout_ms, which needs operator attention
        // (jog free, then a fresh /lift/cmd) rather than an automatic loop.

        vTaskDelay(pdMS_TO_TICKS(LIFT_TASK_PERIOD_MS));
    }
}

void Lift_Init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_LIFT_ENDSTOP_UP_GPIO) | (1ULL << CONFIG_LIFT_ENDSTOP_DOWN_GPIO);
    // Both endstops are wired against the ESP32's internal pullup (switch to
    // GND when triggered), per the board's soldering -- see CONFIG_LIFT_ENDSTOP_ACTIVE_LOW
    // for the logical (triggered = low/high) interpretation of that raw level.
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    lift_read_endstops();
    if (g_endstop_up) g_state = LIFT_STATE_OPEN;
    else if (g_endstop_down) g_state = LIFT_STATE_CLOSED;
    else g_state = LIFT_STATE_IDLE;
    // Not moving at boot -- make sure we're explicitly at the neutral pulse
    // (Servo_Init() already set this, but state changed above so reassert).
    lift_servo_rest();

    xTaskCreatePinnedToCore(lift_task, "lift_task", 3 * 1024, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Lift_Init done (up=GPIO%d, down=GPIO%d, initial state=%d)",
             CONFIG_LIFT_ENDSTOP_UP_GPIO, CONFIG_LIFT_ENDSTOP_DOWN_GPIO, (int)g_state);
}

void Lift_Command_Open(void)
{
    g_jogging = false;

    if (g_endstop_up)
    {
        lift_servo_rest();
        g_state = LIFT_STATE_OPEN;
        return;
    }

    lift_start_moving_up(ServoConfig_Get_LiftPwmRunOffsetUs());
}

void Lift_Command_Close(void)
{
    g_jogging = false;

    if (g_endstop_down)
    {
        lift_servo_rest();
        g_state = LIFT_STATE_CLOSED;
        return;
    }

    lift_start_moving_down(ServoConfig_Get_LiftPwmRunOffsetUs());
}

void Lift_Jog(int direction, int amount_ms, int raw_pulse_us)
{
    // A jog explicitly hands manual control to the operator -- cancel any
    // automatic movement so the background task doesn't fight the jog pulse.
    if (g_state == LIFT_STATE_MOVING_UP || g_state == LIFT_STATE_MOVING_DOWN)
    {
        g_state = LIFT_STATE_IDLE;
    }

    if (amount_ms < 0) amount_ms = 0;
    if (amount_ms > CONFIG_SCISSOR_JOG_MAX_PULSE_MS) amount_ms = CONFIG_SCISSOR_JOG_MAX_PULSE_MS;

    if (amount_ms == 0)
    {
        // Explicit "stop now".
        lift_servo_rest();
        g_jogging = false;
        return;
    }

    int pulse;
    if (raw_pulse_us > 0)
    {
        // Explicit raw-value mode: operator-supplied pulse, still subject
        // to Servo_Set_Pulse_Us()'s hard absolute safety clamp. Lets an
        // operator manually probe values directly without first publishing
        // /servo_config.
        pulse = raw_pulse_us;
    }
    else if (direction == 0)
    {
        // Hold mode: apply exactly the configured neutral/stop pulse (no
        // offset) so an operator can tune CONFIG_LIFT_PWM_STOP_US /
        // lift_pwm_stop_us via /servo_config and listen/watch for creep.
        pulse = ServoConfig_Get_LiftPwmStopUs();
    }
    else
    {
        int sign = (direction > 0) ? 1 : -1;
        pulse = ServoConfig_Get_LiftPwmStopUs() + sign * ServoConfig_Get_LiftPwmJogOffsetUs();
    }
    Servo_Set_Pulse_Us(SERVO_CHANNEL_LIFT, pulse);

    g_jog_remaining_ticks = amount_ms / LIFT_TASK_PERIOD_MS;
    if (g_jog_remaining_ticks < 1) g_jog_remaining_ticks = 1;
    g_jogging = true;
}

void Lift_Refresh_Rest_Output(void)
{
    if (!g_jogging && g_state != LIFT_STATE_MOVING_UP && g_state != LIFT_STATE_MOVING_DOWN)
    {
        lift_servo_rest();
    }
}

lift_state_t Lift_Get_State(void) { return g_state; }
bool Lift_Get_Endstop_Up(void) { return g_endstop_up; }
bool Lift_Get_Endstop_Down(void) { return g_endstop_down; }
