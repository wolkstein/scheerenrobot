/**
 * @file servo_config.c
 * @brief Implementation of the servo PWM calibration store, see servo_config.h.
 */
#include "servo_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <inttypes.h>

static const char *TAG = "SERVOCFG";
#define NVS_NAMESPACE "servocfg"

// CONFIG_LIFT_ENDSTOP_ACTIVE_LOW / CONFIG_LIFT_DIRECTION_UP_IS_INCREASE are
// Kconfig bools -- only #define'd (as 1) when set to "y"; the macro is
// entirely absent from sdkconfig.h when set to "n", so it must never be
// used as a bare initializer value. Resolve both here once into
// always-defined constants.
#if CONFIG_LIFT_ENDSTOP_ACTIVE_LOW
#define LIFT_ENDSTOP_ACTIVE_LOW_DEFAULT 1
#else
#define LIFT_ENDSTOP_ACTIVE_LOW_DEFAULT 0
#endif

#if CONFIG_LIFT_DIRECTION_UP_IS_INCREASE
#define LIFT_DIRECTION_UP_IS_INCREASE_DEFAULT 1
#else
#define LIFT_DIRECTION_UP_IS_INCREASE_DEFAULT 0
#endif

/** In-memory calibration, initialized from Kconfig defaults, overwritten by
 *  ServoConfig_Init() if NVS has saved values, updated live by ServoConfig_Save().
 *  int32_t (not plain int) because nvs_get_i32()/nvs_set_i32() require that
 *  exact pointer type on this toolchain. */
static int32_t g_camera_pwm_min_us               = CONFIG_CAMERA_PWM_AT_0DEG_US;
static int32_t g_camera_pwm_max_us               = CONFIG_CAMERA_PWM_AT_90DEG_US;
static int32_t g_lift_pwm_stop_us                = CONFIG_LIFT_PWM_STOP_US;
static int32_t g_lift_pwm_run_offset_us          = CONFIG_LIFT_PWM_RUN_OFFSET_US;
static int32_t g_lift_pwm_jog_offset_us          = CONFIG_LIFT_PWM_JOG_OFFSET_US;
static int32_t g_lift_pwm_reengage_offset_us     = CONFIG_LIFT_PWM_REENGAGE_OFFSET_US;
static int32_t g_lift_timeout_ms                 = CONFIG_LIFT_TIMEOUT_MS;
static int32_t g_lift_endstop_active_low         = LIFT_ENDSTOP_ACTIVE_LOW_DEFAULT;
static int32_t g_lift_direction_up_is_increase   = LIFT_DIRECTION_UP_IS_INCREASE_DEFAULT;

void ServoConfig_Init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Partition layout changed or is uninitialized -- wipe and retry once.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        // Namespace doesn't exist yet -- nothing has been saved, keep Kconfig defaults.
        ESP_LOGI(TAG, "No saved servo config, using defaults");
        return;
    }

    nvs_get_i32(handle, "cam_min_us",  &g_camera_pwm_min_us);
    nvs_get_i32(handle, "cam_max_us",  &g_camera_pwm_max_us);
    nvs_get_i32(handle, "lift_stop",   &g_lift_pwm_stop_us);
    nvs_get_i32(handle, "lift_run",    &g_lift_pwm_run_offset_us);
    nvs_get_i32(handle, "lift_jog",    &g_lift_pwm_jog_offset_us);
    nvs_get_i32(handle, "lift_reeng",  &g_lift_pwm_reengage_offset_us);
    nvs_get_i32(handle, "lift_tout",   &g_lift_timeout_ms);
    nvs_get_i32(handle, "lift_alow",   &g_lift_endstop_active_low);
    nvs_get_i32(handle, "lift_updir",  &g_lift_direction_up_is_increase);
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded: cam=%" PRId32 "-%" PRId32 "us stop=%" PRId32 "us run=%" PRId32
             "us jog=%" PRId32 "us reengage=%" PRId32 "us timeout=%" PRId32
             "ms active_low=%" PRId32 " up_is_increase=%" PRId32,
             g_camera_pwm_min_us, g_camera_pwm_max_us, g_lift_pwm_stop_us,
             g_lift_pwm_run_offset_us, g_lift_pwm_jog_offset_us,
             g_lift_pwm_reengage_offset_us, g_lift_timeout_ms,
             g_lift_endstop_active_low, g_lift_direction_up_is_increase);
}

int ServoConfig_Get_CameraPwmMinUs(void)              { return g_camera_pwm_min_us; }
int ServoConfig_Get_CameraPwmMaxUs(void)              { return g_camera_pwm_max_us; }
int ServoConfig_Get_LiftPwmStopUs(void)               { return g_lift_pwm_stop_us; }
int ServoConfig_Get_LiftPwmRunOffsetUs(void)          { return g_lift_pwm_run_offset_us; }
int ServoConfig_Get_LiftPwmJogOffsetUs(void)          { return g_lift_pwm_jog_offset_us; }
int ServoConfig_Get_LiftPwmReengageOffsetUs(void)     { return g_lift_pwm_reengage_offset_us; }
int ServoConfig_Get_LiftTimeoutMs(void)               { return g_lift_timeout_ms; }
int ServoConfig_Get_LiftEndstopActiveLow(void)        { return g_lift_endstop_active_low; }
int ServoConfig_Get_LiftDirectionUpIsIncrease(void)   { return g_lift_direction_up_is_increase; }

void ServoConfig_Save(int camera_pwm_min_us, int camera_pwm_max_us,
                       int lift_pwm_stop_us, int lift_pwm_run_offset_us,
                       int lift_pwm_jog_offset_us, int lift_pwm_reengage_offset_us,
                       int lift_timeout_ms, int lift_endstop_active_low,
                       int lift_direction_up_is_increase)
{
    g_camera_pwm_min_us             = camera_pwm_min_us;
    g_camera_pwm_max_us             = camera_pwm_max_us;
    g_lift_pwm_stop_us              = lift_pwm_stop_us;
    g_lift_pwm_run_offset_us        = lift_pwm_run_offset_us;
    g_lift_pwm_jog_offset_us        = lift_pwm_jog_offset_us;
    g_lift_pwm_reengage_offset_us   = lift_pwm_reengage_offset_us;
    g_lift_timeout_ms               = lift_timeout_ms;
    g_lift_endstop_active_low       = lift_endstop_active_low;
    g_lift_direction_up_is_increase = lift_direction_up_is_increase;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return;
    }

    nvs_set_i32(handle, "cam_min_us", camera_pwm_min_us);
    nvs_set_i32(handle, "cam_max_us", camera_pwm_max_us);
    nvs_set_i32(handle, "lift_stop",  lift_pwm_stop_us);
    nvs_set_i32(handle, "lift_run",   lift_pwm_run_offset_us);
    nvs_set_i32(handle, "lift_jog",   lift_pwm_jog_offset_us);
    nvs_set_i32(handle, "lift_reeng", lift_pwm_reengage_offset_us);
    nvs_set_i32(handle, "lift_tout",  lift_timeout_ms);
    nvs_set_i32(handle, "lift_alow",  lift_endstop_active_low);
    nvs_set_i32(handle, "lift_updir", lift_direction_up_is_increase);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Saved: cam=%d-%dus stop=%dus run=%dus jog=%dus reengage=%dus "
             "timeout=%dms active_low=%d up_is_increase=%d",
             camera_pwm_min_us, camera_pwm_max_us, lift_pwm_stop_us,
             lift_pwm_run_offset_us, lift_pwm_jog_offset_us,
             lift_pwm_reengage_offset_us, lift_timeout_ms, lift_endstop_active_low,
             lift_direction_up_is_increase);
}
