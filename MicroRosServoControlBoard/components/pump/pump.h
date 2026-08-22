/**
 * @file pump.h
 * @brief Vacuum pump actuation, digital relay or servo-PWM (compile-time choice).
 *
 * The original hardware drove the vacuum pump through a relay on a plain
 * digital GPIO. Real-world testing showed the relay coil's inductive
 * kickback on switch-off could upset the ESP32 into an undefined state even
 * with flyback diodes in place, so the current prototype fix replaces the
 * relay with a servo (driven exactly like the lift/camera channels in
 * servo.c) that mechanically toggles the pump between two calibrated
 * positions. A future revision is expected to use a semi-solid-state relay
 * instead, so both actuation modes are kept selectable via Kconfig (see
 * `CONFIG_VACUUM_ACTUATION_PWM` / `CONFIG_VACUUM_ACTUATION_DIGITAL` in
 * `Kconfig.projbuild`) rather than hard-deleting the digital path. This is
 * a build-time choice, not runtime-switchable, and does not affect the ROS
 * interface: `/vacuum/cmd` and `/telemetry`'s `vacuum_state` stay a plain
 * on/off bool either way -- only what happens on the wire underneath
 * changes.
 */
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure the pump output (digital GPIO or PWM servo channel,
 *        per Kconfig) and drive it to the "off" state.
 *
 * Must be called once during startup, after ServoConfig_Init() and
 * Servo_Init() (both needed when PWM actuation is selected -- the pump's
 * on/off pulse widths and invert flag come from servo_config, and the LEDC
 * channel/timer must already be configured).
 */
void Pump_Init(void);

/**
 * @brief Switch the pump on or off and update the cached telemetry state.
 *
 * In digital mode, drives CONFIG_VACUUM_RELAY_GPIO high/low directly. In
 * PWM mode, maps the bool onto the calibrated
 * vacuum_pwm_min_us/vacuum_pwm_max_us pulse width via servo_config,
 * optionally swapped by vacuum_pwm_invert, and applies it through
 * Servo_Set_Pulse_Us() on SERVO_CHANNEL_PUMP.
 *
 * @param on true = pump on, false = pump off.
 */
void Pump_Set(bool on);

/**
 * @brief Re-apply the current on/off state's mapped output.
 *
 * Call this after ServoConfig_Save() changes the pump's PWM calibration
 * (min/max/invert) so a config-only change (no new /vacuum/cmd) takes
 * effect on the physical output immediately -- mirrors
 * Lift_Refresh_Rest_Output(). A no-op in digital mode (there is no
 * calibration to re-apply).
 */
void Pump_Refresh_Output(void);

/** @brief Last commanded pump state (true = on), for the /telemetry publisher. */
bool Pump_Get_On(void);

#ifdef __cplusplus
}
#endif
