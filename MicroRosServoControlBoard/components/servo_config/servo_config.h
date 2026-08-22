/**
 * @file servo_config.h
 * @brief NVS-persisted PWM calibration for the lift and camera servos.
 *
 * Keeps the raw pulse-width calibration (camera endpoint pulses, lift stop
 * point, lift run/jog offsets, lift timeout) with Kconfig-derived defaults
 * that can be overridden at runtime via the `/servo_config` micro-ROS topic
 * (see servo_config_callback() in main.c) and persisted in NVS, so values
 * found by jogging (see `/scissor/jog`) survive a reboot.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Load servo calibration from NVS, if any was previously saved.
 *
 * Falls back to the Kconfig defaults below if the "servocfg" NVS namespace
 * doesn't exist yet (first boot / never saved). Must be called once during
 * startup, before ServoConfig_Get_*() is relied upon.
 */
void ServoConfig_Init(void);

/** @brief Pulse width in microseconds corresponding to 0 deg camera pitch. */
int ServoConfig_Get_CameraPwmMinUs(void);

/** @brief Pulse width in microseconds corresponding to 90 deg camera pitch. */
int ServoConfig_Get_CameraPwmMaxUs(void);

/** @brief Neutral/stop pulse width in microseconds for the lift servo. */
int ServoConfig_Get_LiftPwmStopUs(void);

/** @brief Pulse-width offset from the stop point used for normal auf/zu movement. */
int ServoConfig_Get_LiftPwmRunOffsetUs(void);

/** @brief Pulse-width offset from the stop point used for cautious /scissor/jog impulses. */
int ServoConfig_Get_LiftPwmJogOffsetUs(void);

/** @brief Pulse-width offset from the stop point used for automatic re-engage in LIFT_STATE_OPEN/CLOSED. */
int ServoConfig_Get_LiftPwmReengageOffsetUs(void);

/** @brief Max. time in ms a normal auf/zu movement may run before ERROR_TIMEOUT. */
int ServoConfig_Get_LiftTimeoutMs(void);

/**
 * @brief Pulse width in microseconds for the pump's "off" endpoint.
 *
 * Only meaningful when CONFIG_VACUUM_ACTUATION_PWM is selected (see
 * pump.h) -- unused in digital-relay mode.
 */
int ServoConfig_Get_VacuumPwmMinUs(void);

/** @brief Pulse width in microseconds for the pump's "on" endpoint. See ServoConfig_Get_VacuumPwmMinUs(). */
int ServoConfig_Get_VacuumPwmMaxUs(void);

/**
 * @brief Whether the pump's on/off pulse-width mapping is inverted.
 * @return Zero: bool true (on) maps to vacuum_pwm_max_us, false (off) to
 *         vacuum_pwm_min_us. Non-zero: swapped -- lets an operator correct
 *         the servo's physical mounting orientation via `/servo_config`
 *         without touching the calibrated endpoints themselves.
 */
int ServoConfig_Get_VacuumPwmInvert(void);

/**
 * @brief Whether the endstop switches are wired active-low.
 * @return Non-zero if active-low (triggered = GPIO reads low), zero if
 *         active-high (triggered = GPIO reads high). Kconfig
 *         CONFIG_LIFT_ENDSTOP_ACTIVE_LOW only supplies the first-boot
 *         default -- from then on this is runtime-configurable via
 *         `/servo_config`, so the electrical polarity can be corrected
 *         without a rebuild/reflash.
 */
int ServoConfig_Get_LiftEndstopActiveLow(void);

/**
 * @brief Which raw PWM direction physically drives the lift towards "up".
 * @return Non-zero if pulse widths above lift_pwm_stop_us drive up, zero if
 *         pulse widths below it drive up. Kconfig
 *         CONFIG_LIFT_DIRECTION_UP_IS_INCREASE only supplies the first-boot
 *         default -- from then on this is runtime-configurable via
 *         `/servo_config`, so the direction can be corrected without a
 *         rebuild/reflash.
 */
int ServoConfig_Get_LiftDirectionUpIsIncrease(void);

/**
 * @brief Persist new servo calibration to NVS and apply it immediately.
 *
 * @param camera_pwm_min_us     Pulse width (us) at 0 deg camera pitch.
 * @param camera_pwm_max_us     Pulse width (us) at 90 deg camera pitch.
 * @param lift_pwm_stop_us      Neutral/stop pulse width (us) for the lift servo.
 * @param lift_pwm_run_offset_us Offset (us) from stop used for normal auf/zu movement.
 * @param lift_pwm_jog_offset_us Offset (us) from stop used for /scissor/jog impulses.
 * @param lift_pwm_reengage_offset_us Offset (us) from stop used for automatic re-engage in OPEN/CLOSED.
 * @param lift_timeout_ms       Max. auf/zu movement time (ms) before ERROR_TIMEOUT.
 * @param lift_endstop_active_low Non-zero = active-low, zero = active-high (see ServoConfig_Get_LiftEndstopActiveLow()).
 * @param lift_direction_up_is_increase Non-zero = pulse > stop drives up, zero = pulse < stop drives up.
 * @param vacuum_pwm_min_us     Pulse width (us) for the pump's "off" endpoint (PWM mode only).
 * @param vacuum_pwm_max_us     Pulse width (us) for the pump's "on" endpoint (PWM mode only).
 * @param vacuum_pwm_invert     Non-zero = swap which endpoint means on/off (see ServoConfig_Get_VacuumPwmInvert()).
 */
void ServoConfig_Save(int camera_pwm_min_us, int camera_pwm_max_us,
                       int lift_pwm_stop_us, int lift_pwm_run_offset_us,
                       int lift_pwm_jog_offset_us, int lift_pwm_reengage_offset_us,
                       int lift_timeout_ms, int lift_endstop_active_low,
                       int lift_direction_up_is_increase,
                       int vacuum_pwm_min_us, int vacuum_pwm_max_us,
                       int vacuum_pwm_invert);

#ifdef __cplusplus
}
#endif
