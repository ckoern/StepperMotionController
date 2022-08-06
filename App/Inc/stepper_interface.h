#ifndef STEPPER_CTRL_STEPPER_INTERFACE_H
#define STEPPER_CTRL_STEPPER_INTERFACE_H


#include  <stdint.h>
#include "stm32f4xx.h"


#define HOME_SWITCH  ( 1 << 0 )
#define LEFT_SWITCH  ( 1 << 1 )
#define RIGHT_SWITCH ( 1 << 2 )

typedef enum {
    STP_IDLE = 0,
    STP_MOVETO,
    STP_HALT,
    STP_FINISH,
    STP_REFSEARCH_L,
    STP_REFSEARCH_R
} StepperState;


typedef struct {
    uint8_t module_address;
    uint8_t cmd_id;
    uint8_t cmd_type;
    uint8_t cmd_bank;
    uint32_t value;
    uint8_t checksum;

} stepper_command_t;

typedef struct {
    uint8_t reply_address;
    uint8_t module_address;
    uint8_t status_code;
    uint8_t cmd_id;
    uint32_t value;
    uint8_t checksum;
} stepper_reply_t;

typedef struct {

} stepper_global_params_t;

typedef struct{

    int32_t target_position;
    int32_t actual_position;
    int32_t target_speed;
    int32_t actual_speed;
    uint32_t maximum_speed;
    uint32_t acceleration;
    uint8_t target_position_reached;
    uint32_t minimum_speed;
    uint8_t limit_states;
    uint8_t limits_disabled;
    uint8_t limits_switched; // only 0/1
    uint8_t limits_polarity;
    uint8_t microstep_resolution;
    uint32_t end_switch_distance;
    uint8_t reverse_shaft;
    uint8_t refsearch_mode;


} stepper_axis_params_t;


typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    uint32_t base_clk;
} stepper_pulse_timer_t;


typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
} stepper_counter_timer_t;



typedef struct {
    uint32_t nsteps;
    int8_t direction;
    uint32_t nsteps_ramp;
    uint32_t last_tick;
    int32_t start_position;
} stepper_motion_params_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin_left_limit;
    uint16_t pin_right_limit;
    uint16_t pin_direction;
    uint16_t pin_ms0;
    uint16_t pin_ms1;
    uint16_t pin_ms2;
    
} stepper_motor_gpio_t;

typedef struct {
    stepper_axis_params_t ap;
    stepper_motion_params_t mp;
    stepper_pulse_timer_t* pulse_timer;
    stepper_counter_timer_t* count_timer;   
    stepper_motor_gpio_t* io;
    uint32_t update_rate_ms;
    StepperState state;
} stepper_motor_t;


typedef struct {
    uint8_t cmd_buffer[9];
    uint8_t reply_buffer[9];
} stepper_com_buffer_t;


typedef struct {
    stepper_global_params_t gp;
    stepper_motor_t* motor; // this can be an array for multi-axis control
    stepper_com_buffer_t com_buffer;
    UART_HandleTypeDef* huart;
} stepper_board_t;

typedef enum {
    AP_TARGET_POS = 0,
    AP_ACTUAL_POS = 1,
    AP_TARGET_VEL = 2,
    AP_ACTUAL_VEL = 3,
    AP_MAX_VEL = 4,
    AP_MAX_ACC = 5,
    AP_POS_REACHED = 8,
    AP_HOME_SW_STATE = 9,
    AP_RIGHT_SW_STATE = 10,
    AP_LEFT_SW_STATE = 11,
    AP_RIGHT_SW_DISABLE = 12,
    AP_LEFT_SW_DISABLE = 13,
    AP_SWAP_LIMITS = 14,
    AP_START_VEL = 19,
    AP_RIGHT_SW_POLAR = 24,
    AP_LEFT_SW_POLAR = 25,
    AP_MICROSTEP_RESOLUTION = 140,
    AP_REFERENCE_SEARCH_MODE = 193,
    AP_ENDS_DISTANCE = 196,
    AP_REVERSE_SHAFT = 251,
} AxisParamType;


typedef enum {
    CMD_STOP = 3,
    CMD_MOVE = 4,
    CMD_SETAP = 5,
    CMD_GETAP = 6,
    CMD_REFSEARCH = 13,
} StepperCommand;


typedef enum{
    SSC_OK = 100,
    SSC_WRONG_CHECKSUM = 1,
    SSC_INVALID_CMD = 2,
    SSC_INVALID_TYPE = 3,
    SSC_INVALID_VALUE = 4
} StepperStatusCode;


// functions beginning with stepper_ are public
// functions beginning with istepper_ are internal
void stepper_start_movement( stepper_motor_t* motor, int32_t target );
void stepper_stop_movement( stepper_motor_t* motor );
void stepper_update_loop(stepper_motor_t* motor);

void stepper_com_action(stepper_board_t* board);
void istepper_handle_command( stepper_board_t* board, stepper_command_t* cmd, stepper_reply_t* reply );
StepperStatusCode istepper_decode_command( uint8_t* cmd_buffer, stepper_command_t* cmd );
void istepper_encode_reply( uint8_t* reply_buffer, stepper_reply_t* cmd );

void istepper_calculate_motion_params(stepper_motor_t* motor);
void istepper_set_pulse_timer(stepper_motor_t* handle);
void istepper_finish_movement(stepper_motor_t* motor);
uint8_t istepper_check_limit_is_halting(stepper_motor_t* motor);
void istepper_set_microstepping_pins(stepper_motor_t* handle);
void istepper_set_direction_pin(stepper_motor_t* handle);

void istepper_handle_limit_halted(stepper_motor_t* motor);

static inline void stepper_notify_limitstate_changed(stepper_motor_t* motor){
    uint8_t val = 0;
	// TODO Home Switch
	// inputs have pull ups and switches should pull pin to ground,
	// so pin high is switch not triggered
	if (!HAL_GPIO_ReadPin(motor->io->port, motor->io->pin_left_limit)){
		val |= LEFT_SWITCH;
	}
	if (!HAL_GPIO_ReadPin(motor->io->port, motor->io->pin_right_limit)){
		val |= RIGHT_SWITCH;
	}
	motor->ap.limit_states = val;
}

static inline void stepper_notify_target_reached(stepper_motor_t* motor){
    motor->state = STP_FINISH;
}

static inline void istepper_enable_pulse_tim(stepper_motor_t* motor){
    motor->pulse_timer->htim->Instance->CR1 |= TIM_CR1_CEN;
}

static inline void istepper_disable_pulse_tim(stepper_motor_t* motor){
    motor->pulse_timer->htim->Instance->CR1 &= ~TIM_CR1_CEN;
}
uint32_t stepper_get_axis_param(stepper_motor_t* motor, AxisParamType param);
StepperStatusCode stepper_set_axis_param(stepper_motor_t* motor, AxisParamType param, uint32_t value_enc);

#endif
