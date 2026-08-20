/**
 * @file servo.h
 * @brief Low-level hardware-PWM driver for RC-style servos (LEDC peripheral).
 *
 * Drives two independent 50Hz LEDC channels: the endless-rotation scissor-lift
 * servo and the conventional camera-pitch servo. This layer knows nothing
 * about endstops, calibration or ROS -- it only ever converts a pulse width
 * in microseconds into a duty cycle and clamps it to the hardware-safe range.
 * Higher-level logic (direction, calibration, state machines) lives in
 * lift.c / main.c.
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/** Identifies which of the two servo outputs a call applies to. */
typedef enum
{
    SERVO_CHANNEL_LIFT = 0,   /**< Scissor-lift endless-rotation servo (G32). */
    SERVO_CHANNEL_CAMERA = 1, /**< Camera-pitch positional servo (G33). */
} servo_channel_t;

/**
 * @brief Configure the LEDC timer and both servo channels and start them at
 *        their configured "stop"/neutral pulse.
 *
 * Must be called once during startup before any other Servo_*() function.
 */
void Servo_Init(void);

/**
 * @brief Command a servo channel to a given pulse width.
 *
 * The value is always clamped to [CONFIG_SERVO_PULSE_ABS_MIN_US,
 * CONFIG_SERVO_PULSE_ABS_MAX_US] before being applied -- this hard safety
 * limit holds even for jog commands that intentionally bypass endstops or
 * calibration, so a bad ROS message can never command a physically
 * destructive pulse width.
 *
 * There is deliberately no way to stop generating PWM entirely on a
 * channel. An earlier version of this driver had a Servo_Disable() that cut
 * the signal outright; real hardware testing showed the lift's cheap
 * continuous-rotation servo does NOT coast/stop when its PWM signal
 * disappears -- it appears to latch/hold its last driven state instead, so
 * removing the signal did not stop it and once caused it to run away
 * uncontrolled. Always keep a valid periodic pulse on the line; "stop"
 * means driving the neutral/stop pulse width, never silence.
 *
 * @param channel  Which servo output to drive.
 * @param pulse_us Desired pulse width in microseconds (typ. 500-2500).
 */
void Servo_Set_Pulse_Us(servo_channel_t channel, int pulse_us);

/**
 * @brief Read back the last pulse width actually applied to a channel.
 *
 * Used by the telemetry publisher so operators can watch the raw PWM value
 * while jogging/calibrating (see lift.c and main.c's /scissor/jog handling).
 *
 * @param channel Which servo output to query.
 * @return Last applied pulse width in microseconds.
 */
int Servo_Get_Pulse_Us(servo_channel_t channel);

#ifdef __cplusplus
}
#endif
