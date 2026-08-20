/**
 * @file lift.h
 * @brief Scissor-table lift state machine: endstops + endless-rotation servo.
 *
 * Owns the two endstop GPIOs and the lift servo channel (see servo.h) and
 * runs the auf/zu (open/close) logic entirely on-device: once commanded, it
 * drives the servo and watches the endstops/timeout itself in a background
 * task, with no further ROS interaction required until it reaches OPEN,
 * CLOSED or ERROR_TIMEOUT. The `/scissor/jog` maintenance/emergency path
 * (Lift_Jog()) intentionally bypasses all of this and drives the servo with
 * short, bounded raw pulses instead -- see main.c's scissor_jog_callback().
 */
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Lift state machine states, also used as the `lift_state` field of the
 *  `/telemetry` array (see main.c). */
typedef enum
{
    LIFT_STATE_IDLE          = 0, /**< Between endstops, no movement commanded (or after a jog). */
    LIFT_STATE_MOVING_UP     = 1, /**< Driving towards the "open" (up) endstop. */
    LIFT_STATE_MOVING_DOWN   = 2, /**< Driving towards the "closed" (down) endstop. */
    LIFT_STATE_OPEN          = 3, /**< Up endstop reached, servo held at its neutral pulse. Briefly re-enters MOVING_UP on its own if the endstop is ever lost while resting (see lift_task() in lift.c). */
    LIFT_STATE_CLOSED        = 4, /**< Down endstop reached, servo held at its neutral pulse. Held the same way as LIFT_STATE_OPEN. */
    LIFT_STATE_ERROR_TIMEOUT = 5, /**< Movement did not reach its endstop within CONFIG_LIFT_TIMEOUT_MS. */
} lift_state_t;

/**
 * @brief Configure endstop GPIOs, read their initial state and start the
 *        background task that runs the state machine.
 *
 * Must be called once during startup, after Servo_Init() and ServoConfig_Init().
 */
void Lift_Init(void);

/**
 * @brief Command the scissor table to open ("auf").
 *
 * No-op if the up endstop is already triggered. Otherwise cancels any
 * in-progress jog, starts driving the lift servo and transitions to
 * LIFT_STATE_MOVING_UP; the background task stops the servo automatically
 * once the up endstop triggers (-> LIFT_STATE_OPEN) or the configured
 * timeout elapses (-> LIFT_STATE_ERROR_TIMEOUT).
 */
void Lift_Command_Open(void);

/**
 * @brief Command the scissor table to close ("zu"). Mirrors Lift_Command_Open().
 */
void Lift_Command_Close(void);

/**
 * @brief Cautiously jog the lift servo for a short, bounded pulse, hold it
 *        at its configured neutral/stop pulse, or hold it at an explicit
 *        raw pulse width -- either way ignoring endstops and the normal
 *        open/close state machine.
 *
 * Intended for setup (manually probing for the true PWM_STOP_US where a
 * cheap continuous-rotation servo actually stands still) and for freeing a
 * mechanical jam (see `/scissor/jog` in main.c). Cancels any in-progress
 * MOVING_UP/MOVING_DOWN automatic movement first. The duration is clamped
 * to [0, CONFIG_SCISSOR_JOG_MAX_PULSE_MS], and the resulting servo pulse
 * width is still subject to Servo_Set_Pulse_Us()'s hard absolute safety
 * clamp regardless of mode. Once the duration elapses (or amount_ms is 0),
 * the servo returns to holding its configured neutral/stop pulse -- see
 * servo.h for why the PWM signal is never cut off entirely, even at rest.
 *
 * @param direction    Sign of the pulse-width offset from the configured
 *                      stop point, only used when raw_pulse_us is 0:
 *                      positive increases the pulse width, negative
 *                      decreases it, using the configured
 *                      lift_pwm_jog_offset_us. `0` means "hold exactly at
 *                      the configured stop pulse" instead of jogging away
 *                      from it. This is a raw hardware direction,
 *                      independent of CONFIG_LIFT_DIRECTION_UP_IS_INCREASE,
 *                      so jogging still works even before that mapping is
 *                      known.
 * @param amount_ms     Duration in milliseconds; `0` stops immediately
 *                      regardless of the other parameters.
 * @param raw_pulse_us  If greater than 0, overrides direction/jog_offset
 *                      entirely and holds exactly this pulse width (still
 *                      hard-safety-clamped) for amount_ms -- lets an
 *                      operator manually probe arbitrary values (e.g.
 *                      1480, 1520, ...) directly via `/scissor/jog` without
 *                      touching `/servo_config` first. `0` (or negative)
 *                      means "not set", falls back to the direction-based
 *                      behavior above.
 */
void Lift_Jog(int direction, int amount_ms, int raw_pulse_us);

/**
 * @brief Re-apply the current resting (neutral/stop) pulse, if the lift is
 *        not actively moving or jogging right now.
 *
 * Call this after ServoConfig_Save() changes lift_pwm_stop_us so the new
 * value takes effect on the physical output immediately, instead of only
 * on the next movement/jog -- otherwise /telemetry would keep showing the
 * previously-applied pulse until some unrelated command happens to move
 * the servo. No-op while LIFT_STATE_MOVING_UP/MOVING_DOWN or jogging, so it
 * never disturbs an in-progress move.
 */
void Lift_Refresh_Rest_Output(void);

/** @brief Current lift state machine state. */
lift_state_t Lift_Get_State(void);

/** @brief Current (debounced-by-nothing, raw) logical state of the up endstop. */
bool Lift_Get_Endstop_Up(void);

/** @brief Current (debounced-by-nothing, raw) logical state of the down endstop. */
bool Lift_Get_Endstop_Down(void);

#ifdef __cplusplus
}
#endif
