#ifndef STEPPER_CTRL_STEPPER_INTERFACE_H
#define STEPPER_CTRL_STEPPER_INTERFACE_H


#include  <stdint.h>
#include "stm32f4xx.h"

typedef struct {
    uint8_t address;
    uint8_t cmd_id;
    uint8_t cmd_type;
    uint8_t cmd_bank;
    uint32_t value;
    uint8_t checksum;

} stepper_command_t;

typedef struct {

} stepper_reply_t;

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
    uint8_t direction;
    uint32_t nsteps_ramp;
    uint8_t abort_move;
    uint8_t finish_move;
    uint32_t last_tick;
    uint32_t start_position;
} stepper_motion_params_t;



typedef struct {
    stepper_axis_params_t ap;
    stepper_motion_params_t mp;
    stepper_pulse_timer_t* pulse_timer;
    stepper_counter_timer_t* count_timer;
    uint32_t update_rate_ms;
} stepper_motor_t;


// functions beginning with stepper_ are public
// functions beginning with istepper_ are internal
void stepper_start_movement( stepper_motor_t* motor, int32_t target );
void stepper_stop_movement( stepper_motor_t* motor );
void stepper_update_loop(stepper_motor_t* motor);

void istepper_calculate_motion_params(stepper_motor_t* motor);
void istepper_set_pulse_timer(stepper_motor_t* handle);
void istepper_finish_movement(stepper_motor_t* motor);


enum AxisParamTypes {
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
    AP_RIGHT_SW_POLAR = 24,
    AP_LEFT_SW_POLAR = 25,
    AP_MICROSTEP_RESOLUTION = 140,
    AP_ENDS_DISTANCE = 196,
    AP_REVERSE_SHAFT = 251,
};


enum StepperCommands {
    STOP = 3,
    MOVE = 4,
    SETAP = 5,
    GETAP = 6,
    REFSEARCH = 13,
};


enum StatusCodes{
    OK = 100,
    WRONG_CHECKSUM = 1,
    INVALID_CMD = 2,
    INVALID_TYPE = 3,
    INVALID_VALUE = 4
};



#endif
