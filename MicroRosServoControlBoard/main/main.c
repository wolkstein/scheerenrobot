#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "servo.h"
#include "servo_config.h"
#include "lift.h"

// Custom UART transport for micro-ROS (identical to ybMecanumWheelMicroRosBot,
// reuses the same ../extra_components micro-ROS build)
#define UART_BUFFER_SIZE 512

static bool transport_serial_open(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    uart_config_t uart_config = {
        .baud_rate  = CONFIG_MICRO_ROS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(*uart_port, &uart_config) != ESP_OK) return false;
    if (uart_set_pin(*uart_port, CONFIG_MICRO_ROS_UART_TXD, CONFIG_MICRO_ROS_UART_RXD,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) return false;
    return true;
}

static bool transport_serial_close(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_driver_delete(*uart_port) == ESP_OK;
}

static size_t transport_serial_write(struct uxrCustomTransport *transport,
                                     const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_write_bytes(*uart_port, (const char *)buf, len);
}

static size_t transport_serial_read(struct uxrCustomTransport *transport,
                                    uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
}


// NOTE: this board's micro-ROS transport shares the physical UART0/USB line
// with the console (see transport_serial_* below + uart_driver_delete(UART_NUM_0)
// in micro_ros_task). printf() bypasses the ESP_LOG level filter and always
// writes straight to that console UART, so it must never be used here once
// the transport is up -- it would corrupt the XRCE-DDS byte stream and drop
// the agent session. Keep these macros silent.
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            vTaskDelete(NULL);       \
        }                            \
    }
#define RCSOFTCHECK(fn)          \
    {                            \
        rcl_ret_t temp_rc = fn;  \
        (void)temp_rc;           \
    }


#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID


static const char *TAG = "MAIN";

// Subscribers (commands in)
rcl_subscription_t lift_cmd_subscriber;
std_msgs__msg__Bool msg_lift_cmd;

rcl_subscription_t camera_tilt_subscriber;
std_msgs__msg__Float32 msg_camera_tilt;

rcl_subscription_t vacuum_cmd_subscriber;
std_msgs__msg__Bool msg_vacuum_cmd;

rcl_subscription_t scissor_jog_subscriber;
std_msgs__msg__Int32MultiArray msg_scissor_jog;

rcl_subscription_t servo_config_subscriber;
std_msgs__msg__Int32MultiArray msg_servo_config;

// Publisher (telemetry out)
rcl_publisher_t publisher_telemetry;
std_msgs__msg__Int32MultiArray msg_telemetry;

rcl_timer_t timer_telemetry;

/** Last commanded vacuum relay state, cached here purely for the /telemetry
 *  array (the GPIO itself is the source of truth for actual hardware state). */
static bool g_vacuum_on = false;

/**
 * @brief Configure the vacuum relay GPIO as output and ensure it starts off.
 */
static void vacuum_gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_VACUUM_RELAY_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_VACUUM_RELAY_GPIO, 0);
    g_vacuum_on = false;
}

/** @brief Switch the vacuum pump relay on/off and update the cached telemetry state. */
static void vacuum_set(bool on)
{
    gpio_set_level(CONFIG_VACUUM_RELAY_GPIO, on ? 1 : 0);
    g_vacuum_on = on;
}

/**
 * @brief Preallocate the fixed-size Int32MultiArray for /scissor/jog.
 *
 * micro-ROS does not reallocate on receive -- the deserializer only ever
 * writes up to msg->data.capacity, so the buffer must exist before the
 * executor starts spinning.
 */
static void scissor_jog_ros_init(void)
{
    msg_scissor_jog.data.data = (int32_t *)malloc(4 * sizeof(int32_t));
    msg_scissor_jog.data.size = 0;
    msg_scissor_jog.data.capacity = 4;
}

/** @brief Preallocate the fixed-size Int32MultiArray for /servo_config. See scissor_jog_ros_init(). */
static void servo_config_ros_init(void)
{
    msg_servo_config.data.data = (int32_t *)malloc(9 * sizeof(int32_t));
    msg_servo_config.data.size = 0;
    msg_servo_config.data.capacity = 9;
}

/** @brief Preallocate the fixed-size Int32MultiArray used to publish /telemetry. */
static void telemetry_ros_init(void)
{
    msg_telemetry.data.data = (int32_t *)malloc(6 * sizeof(int32_t));
    msg_telemetry.data.size = 0;
    msg_telemetry.data.capacity = 6;
}

/**
 * @brief /lift/cmd callback (std_msgs/Bool): true = auf (open), false = zu (close).
 *
 * All movement logic (driving the servo, watching endstops/timeout) happens
 * on-device in the lift component -- this callback only ever triggers it.
 */
void lift_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) Lift_Command_Open();
    else Lift_Command_Close();
}

/**
 * @brief /camera/tilt callback (std_msgs/Float32): target pitch in degrees,
 *        0 = level with the robot frame, 90 = pointing straight up.
 *
 * Linearly maps the clamped angle onto the calibrated pulse-width endpoints
 * from servo_config (defaults from Kconfig, overridable via /servo_config).
 */
void camera_tilt_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float deg = msg->data;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 90.0f) deg = 90.0f;

    int pwm_min = ServoConfig_Get_CameraPwmMinUs();
    int pwm_max = ServoConfig_Get_CameraPwmMaxUs();
    int pulse_us = pwm_min + (int)((pwm_max - pwm_min) * (deg / 90.0f));
    Servo_Set_Pulse_Us(SERVO_CHANNEL_CAMERA, pulse_us);
}

/** @brief /vacuum/cmd callback (std_msgs/Bool): true = on, false = off. */
void vacuum_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    vacuum_set(msg->data);
}

/**
 * @brief /scissor/jog callback (std_msgs/Int32MultiArray, 4 values):
 *        [target, direction, amount, raw_pulse_us]. Maintenance/emergency
 *        path that deliberately ignores endstops and the normal auf/zu
 *        state machine.
 *
 * target: 0 = lift servo, 1 = camera servo.
 * direction: for target=0 (and only if raw_pulse_us is 0/unset), -1/+1 =
 *            raw hardware jog direction, 0 = hold exactly at the configured
 *            neutral/stop pulse instead (see Lift_Jog()). For target=1,
 *            -1/+1 degree step direction for the camera servo (0 is a
 *            no-op). Ignored for target=0 when raw_pulse_us is set.
 * amount: for target=0, pulse/hold duration in ms (clamped to
 *         CONFIG_SCISSOR_JOG_MAX_PULSE_MS, `0` stops immediately); for
 *         target=1, degrees to step the camera servo by.
 * raw_pulse_us: only used for target=0. If >0, overrides direction/the
 *         configured jog offset entirely and holds exactly this pulse width
 *         (still hard-safety-clamped) for `amount` ms -- lets an operator
 *         manually probe values directly, e.g. to find PWM_STOP_US, without
 *         touching /servo_config first. `0` (or negative) means "not set".
 */
void scissor_jog_callback(const void *msgin)
{
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size != 4)
    {
        ESP_LOGE(TAG, "scissor/jog: expected 4 values, got %u", (unsigned)msg->data.size);
        return;
    }

    int32_t target = msg->data.data[0];
    int32_t direction = msg->data.data[1];
    int32_t amount = msg->data.data[2];
    int32_t raw_pulse_us = msg->data.data[3];

    if (target == 0)
    {
        Lift_Jog((int)direction, (int)amount, (int)raw_pulse_us);
    }
    else if (target == 1)
    {
        int pwm_min = ServoConfig_Get_CameraPwmMinUs();
        int pwm_max = ServoConfig_Get_CameraPwmMaxUs();
        float us_per_deg = (pwm_max - pwm_min) / 90.0f;
        int sign = (direction > 0) ? 1 : (direction < 0 ? -1 : 0);
        int delta_us = (int)(sign * amount * us_per_deg);
        Servo_Set_Pulse_Us(SERVO_CHANNEL_CAMERA, Servo_Get_Pulse_Us(SERVO_CHANNEL_CAMERA) + delta_us);
    }
    else
    {
        ESP_LOGE(TAG, "scissor/jog: unknown target %ld", (long)target);
    }
}

/**
 * @brief /servo_config callback (std_msgs/Int32MultiArray, 9 values):
 *        [camera_pwm_min_us, camera_pwm_max_us, lift_pwm_stop_us,
 *         lift_pwm_run_offset_us, lift_pwm_jog_offset_us,
 *         lift_pwm_reengage_offset_us, lift_timeout_ms,
 *         lift_endstop_active_low, lift_direction_up_is_increase].
 *
 * Persists the new calibration to NVS via ServoConfig_Save(). Also
 * immediately re-applies the (possibly new) lift_pwm_stop_us to the
 * physical lift servo output if it's currently at rest (see
 * Lift_Refresh_Rest_Output()) -- otherwise /telemetry would keep showing
 * the previously-applied pulse until an unrelated movement happens to pick
 * up the new value, which is confusing while probing for the stop pulse
 * via /scissor/jog + /telemetry.
 */
void servo_config_callback(const void *msgin)
{
    const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (msg->data.size != 9)
    {
        ESP_LOGE(TAG, "servo_config: expected 9 values, got %u", (unsigned)msg->data.size);
        return;
    }

    ServoConfig_Save(msg->data.data[0], msg->data.data[1], msg->data.data[2],
                      msg->data.data[3], msg->data.data[4], msg->data.data[5],
                      msg->data.data[6], msg->data.data[7], msg->data.data[8]);
    Lift_Refresh_Rest_Output();
}

/**
 * @brief Periodic /telemetry publisher (std_msgs/Int32MultiArray, 6 values):
 *        [lift_state, endstop_up, endstop_down, lift_pwm_us, camera_pwm_us,
 *         vacuum_state]. See lift_state_t in lift.h for the state enum.
 */
void timer_telemetry_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    msg_telemetry.data.data[0] = (int32_t)Lift_Get_State();
    msg_telemetry.data.data[1] = Lift_Get_Endstop_Up() ? 1 : 0;
    msg_telemetry.data.data[2] = Lift_Get_Endstop_Down() ? 1 : 0;
    msg_telemetry.data.data[3] = Servo_Get_Pulse_Us(SERVO_CHANNEL_LIFT);
    msg_telemetry.data.data[4] = Servo_Get_Pulse_Us(SERVO_CHANNEL_CAMERA);
    msg_telemetry.data.data[5] = g_vacuum_on ? 1 : 0;
    msg_telemetry.data.size = 6;

    RCSOFTCHECK(rcl_publish(&publisher_telemetry, &msg_telemetry, NULL));
}

// micro ros processes tasks
void micro_ros_task(void *arg)
{
    // Disable logging and release UART0 so micro-ROS can take it exclusively
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_driver_delete(UART_NUM_0);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    // change ros domain id
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Initialize the rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Register custom serial transport
    static size_t uart_port = CONFIG_MICRO_ROS_UART_NUM;
    RCCHECK(rmw_uros_options_set_custom_transport(
        true, (void *) &uart_port,
        transport_serial_open, transport_serial_close,
        transport_serial_write, transport_serial_read,
        rmw_options));

    // Try to connect to the agent. If the connection succeeds, go to the next step.
    rcl_ret_t state_agent = RCL_RET_ERROR;
    while (1)
    {
        ESP_LOGI(TAG, "Connecting agent over UART transport");
        state_agent = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state_agent == RCL_RET_OK)
        {
            ESP_LOGI(TAG, "Connected agent over UART transport");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // create ROS2 node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ScheerenLift_Node", ROS_NAMESPACE, &support));

    // Subscriber /lift/cmd (std_msgs/Bool: true=auf, false=zu)
    RCCHECK(rclc_subscription_init_default(
        &lift_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "lift/cmd"));

    // Subscriber /camera/tilt (std_msgs/Float32, 0-90 deg)
    RCCHECK(rclc_subscription_init_default(
        &camera_tilt_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "camera/tilt"));

    // Subscriber /vacuum/cmd (std_msgs/Bool)
    RCCHECK(rclc_subscription_init_default(
        &vacuum_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "vacuum/cmd"));

    // Subscriber /scissor/jog (std_msgs/Int32MultiArray, 4 values)
    RCCHECK(rclc_subscription_init_default(
        &scissor_jog_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "scissor/jog"));

    // Subscriber /servo_config (std_msgs/Int32MultiArray, 9 values)
    RCCHECK(rclc_subscription_init_default(
        &servo_config_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "servo_config"));

    // Publisher /telemetry (std_msgs/Int32MultiArray, 6 values)
    RCCHECK(rclc_publisher_init_default(
        &publisher_telemetry,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "telemetry"));

    // create timer. Publishes /telemetry at 5Hz
    const unsigned int telemetry_timer_timeout = 200;
    RCCHECK(rclc_timer_init_default2(
        &timer_telemetry,
        &support,
        RCL_MS_TO_NS(telemetry_timer_timeout),
        timer_telemetry_callback, true));

    // create executor. Handle count must cover 5 subscriptions + 1 timer.
    rclc_executor_t executor;
    int handle_num = 6;
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer_telemetry));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &lift_cmd_subscriber,
        &msg_lift_cmd,
        &lift_cmd_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &camera_tilt_subscriber,
        &msg_camera_tilt,
        &camera_tilt_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &vacuum_cmd_subscriber,
        &msg_vacuum_cmd,
        &vacuum_cmd_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &scissor_jog_subscriber,
        &msg_scissor_jog,
        &scissor_jog_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &servo_config_subscriber,
        &msg_servo_config,
        &servo_config_callback,
        ON_NEW_DATA));

    // Loop the microROS task
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&lift_cmd_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&camera_tilt_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&vacuum_cmd_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&scissor_jog_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&servo_config_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher_telemetry, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    vacuum_gpio_init();
    ServoConfig_Init();
    Servo_Init();
    Lift_Init();

    scissor_jog_ros_init();
    servo_config_ros_init();
    telemetry_ros_init();

    // Start microROS task
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
