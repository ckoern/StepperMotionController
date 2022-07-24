#include "stepper_interface.h"
#include <string.h>

void istepper_calculate_motion_params(stepper_motor_t* motor){

    motor->mp.start_position = motor->ap.actual_position;
    motor->mp.abort_move = 0;
    motor->mp.finish_move = 0;
    motor->mp.direction = motor->ap.target_position > motor->ap.actual_position? 1 : -1;
    if (motor->mp.direction > 0){
        motor->mp.nsteps = motor->ap.target_position - motor->ap.actual_position;
    }
    else{
        motor->mp.nsteps = motor->ap.actual_position - motor->ap.target_position;
    }

    uint32_t dv = motor->ap.maximum_speed - motor->ap.minimum_speed;
	uint32_t nsteps_ramp = dv*dv/motor->ap.acceleration/2 + motor->ap.minimum_speed * dv / motor->ap.acceleration;

	//nsteps_ramp is the number of steps that is needed to complete a (de)acceleration ramp

	// if total length is shorter than combined acceleration/deacceleration
	// accelerate until half of the distance, then deacclerate
	if (2*nsteps_ramp > motor->mp.nsteps){
		nsteps_ramp = motor->mp.nsteps/2;
	}
    motor->mp.nsteps_ramp = nsteps_ramp;
}

void istepper_set_pulse_timer(stepper_motor_t* handle){
    uint32_t period = handle->pulse_timer->base_clk / handle->ap.actual_speed;
    handle->pulse_timer->htim->Instance->ARR = period - 1;
	handle->pulse_timer->htim->Instance->CCR1 = period / 2;
     // reset the counter if the new ARR is lower than the current count
	if (handle->pulse_timer->htim->Instance->CNT >= (period-1)){
		handle->pulse_timer->htim->Instance->CNT=0;
	}
}



void stepper_start_movement( stepper_motor_t* motor, int32_t target ){
	// cannot start move while moving
	if (!motor->ap.target_position_reached){
		return;
	}
	motor->ap.target_position_reached = 0;
    motor->ap.target_position = target;
    motor->mp.last_tick = HAL_GetTick();
    motor->ap.actual_speed = motor->ap.minimum_speed;
    istepper_calculate_motion_params(motor);
    motor->count_timer->htim->Instance->ARR = motor->mp.nsteps - 1;
    motor->count_timer->htim->Instance->CNT = 0;

    // Set direction and MS pins
    istepper_set_microstepping_pins(motor);
    istepper_set_direction_pin(motor);

    istepper_set_pulse_timer(motor);
	//HAL_TIM_PWM_Start(motor->pulse_timer->htim, motor->pulse_timer->channel);
    //motor->pulse_timer->htim->Instance->CR1 |= TIM_CR1_CEN;
    istepper_enable_pulse_tim(motor);
}


void istepper_finish_movement(stepper_motor_t* motor){

    //HAL_TIM_PWM_Stop(motor->pulse_timer->htim, motor->pulse_timer->channel);
    //motor->pulse_timer->htim->Instance->CR1 &= ~TIM_CR1_CEN;
    istepper_disable_pulse_tim(motor);
	motor->ap.actual_position = motor->mp.start_position + motor->mp.direction * motor->mp.nsteps;
    motor->ap.actual_speed = 0;
	motor->ap.target_position_reached = 1;
	motor->mp.finish_move = 0;
}


void stepper_stop_movement( stepper_motor_t* motor ){
    //HAL_TIM_PWM_Stop(motor->pulse_timer->htim, motor->pulse_timer->channel);
    motor->pulse_timer->htim->Instance->CR1 &= ~TIM_CR1_CEN;

	uint32_t current_nsteps = motor->count_timer->htim->Instance->CNT;
    motor->ap.actual_position = motor->mp.start_position + motor->mp.direction * current_nsteps;
    motor->ap.target_position = motor->ap.actual_position;
    motor->ap.actual_speed = 0; 
    motor->ap.target_position_reached = 1;
}



void stepper_update_loop(stepper_motor_t* motor){
    if (motor->ap.target_position_reached){
        return;
    }


    // TODO check endstops
    if (motor->mp.finish_move){
    	istepper_finish_movement(motor);
    }else{
        // Check the limit switch and dis- / enable pulse timer
        if (istepper_limit_halt_move(motor)){
            istepper_disable_pulse_tim(motor);
        }else{
            istepper_enable_pulse_tim(motor);
        }
    }
    uint32_t tick = HAL_GetTick();
    uint32_t last_tick = motor->mp.last_tick;
	if (last_tick > tick){
		// ticker overflow, unlikely to ever occur for 1ms ticks and 32 bit
		last_tick = 0;
	}

    // update motion if update rate was exceeded
	if ((tick - last_tick >= motor->update_rate_ms) ){
    	motor->mp.last_tick = tick;

        //uint32_t arr = motor->pulse_timer->htim->Instance->ARR;
	    uint32_t current_nsteps = motor->count_timer->htim->Instance->CNT;

        motor->ap.actual_position = motor->mp.start_position + motor->mp.direction * current_nsteps;

        uint32_t new_speed;
        if (current_nsteps >= ( motor->mp.nsteps - motor->mp.nsteps_ramp )){
            //decceleration ramp
            float ramp_frac = 1.0f * ( current_nsteps - ( motor->mp.nsteps - motor->mp.nsteps_ramp) ) / motor->mp.nsteps_ramp;
            new_speed = motor->ap.maximum_speed - (uint32_t)( ramp_frac*(motor->ap.maximum_speed - motor->ap.minimum_speed) );
        } else if (current_nsteps < motor->mp.nsteps_ramp){
            // acceleration ramp
            float ramp_frac = 1.0f * current_nsteps  / motor->mp.nsteps_ramp;
            new_speed = motor->ap.minimum_speed +  (uint32_t)( ramp_frac*(motor->ap.maximum_speed - motor->ap.minimum_speed) );

        }else{
            //max speed
            new_speed = motor->ap.maximum_speed ;
        }

        motor->ap.actual_speed = new_speed;
        istepper_set_pulse_timer(motor);
    }
}

uint8_t istepper_limit_halt_move(stepper_motor_t* motor){
    uint8_t switch_to_check;
    if (motor->mp.direction > 0){
        switch_to_check = motor->ap.limits_switched? LEFT_SWITCH : RIGHT_SWITCH;
    }
    else{
        switch_to_check = motor->ap.limits_switched? RIGHT_SWITCH : LEFT_SWITCH;
    }
    if (!( motor->ap.limits_disabled & switch_to_check )){
        return ( motor->ap.limits_polarity^motor->ap.limit_states ) & switch_to_check;
    }
    return 0;
}

void istepper_set_microstepping_pins(stepper_motor_t* handle){
    switch (handle->ap.microstep_resolution){
        case 0:
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms2, GPIO_PIN_RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms2, GPIO_PIN_RESET);
            break;
        
        case 2:
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms2, GPIO_PIN_RESET);
            break;

        case 3:
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms2, GPIO_PIN_RESET);
            break;

        case 4:
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(handle->io->port, handle->io->pin_ms2, GPIO_PIN_SET);
            break;
    }
}

void istepper_set_direction_pin(stepper_motor_t* handle){


    HAL_GPIO_WritePin(
        handle->io->port, 
        handle->io->pin_direction, 
        ((handle->mp.direction > 0) != (handle->ap.reverse_shaft == 0) )? GPIO_PIN_SET : GPIO_PIN_RESET
    );

}


void stepper_handle_command( stepper_motor_t* motor, stepper_command_t* cmd, stepper_reply_t* reply ){
    reply->cmd_id = cmd->cmd_id;
    reply->status_code = SSC_OK;
    switch( cmd->cmd_id ){
        case CMD_STOP:{
            if (!motor->ap.target_position_reached){
                stepper_stop_movement(motor);
            }
            break;
        }
        case CMD_MOVE: {
            if (motor->ap.target_position_reached){
                int32_t target_pos;
                memcpy( &target_pos, &(cmd->value), 4 );
                if (cmd->cmd_type == 1){
                    target_pos += motor->ap.actual_position;
                }
                stepper_start_movement(motor, target_pos);
            }
            else{
                reply->status_code = SSC_INVALID_CMD;
            }
            break;
        }

        case CMD_SETAP: {
            reply->status_code = stepper_set_axis_param(motor, cmd->cmd_type, cmd->value);
            reply->value = stepper_get_axis_param(motor, cmd->cmd_type);
        }

        case CMD_GETAP:{
            reply->value = stepper_get_axis_param(motor, cmd->cmd_type);
            break;
        }

        case CMD_REFSEARCH :{
            break;
        }

        default: {
            reply->status_code = SSC_INVALID_CMD;
        }
    }
}

uint32_t stepper_get_axis_param(stepper_motor_t* motor, AxisParamType param){
    
    uint32_t ret_val;
    switch (param){
        case AP_TARGET_POS:{
            memcpy(&ret_val, &(motor->ap.target_position),4);
            break;
        }
        case AP_ACTUAL_POS:{
            memcpy(&ret_val, &(motor->ap.actual_position),4);
            break;
        }
        case AP_TARGET_VEL:{
            memcpy(&ret_val, &(motor->ap.target_speed),4);
            break;
        }
        case AP_ACTUAL_VEL:{
            memcpy(&ret_val, &(motor->ap.actual_speed),4);
            break;
        }
        case AP_MAX_VEL:{
            memcpy(&ret_val, &(motor->ap.maximum_speed),4);
            break;
        }
        case AP_MAX_ACC:{
            memcpy(&ret_val, &(motor->ap.acceleration),4);
            break;
        }
        case AP_POS_REACHED:{
            ret_val = motor->ap.target_position_reached;
            break;
        }
        case AP_HOME_SW_STATE:{
            ret_val = (motor->ap.limit_states & HOME_SWITCH) > 0;
            break;
        }
        case AP_RIGHT_SW_STATE:{
            ret_val = (motor->ap.limit_states & RIGHT_SWITCH) > 0;
            break;
        }
        case AP_LEFT_SW_STATE:{
            ret_val = (motor->ap.limit_states & LEFT_SWITCH) > 0;
            break;
        }
        case AP_RIGHT_SW_DISABLE:{
            ret_val = (motor->ap.limits_disabled & RIGHT_SWITCH) > 0;
            break;
        }
        case AP_LEFT_SW_DISABLE:{
            ret_val = (motor->ap.limits_disabled & LEFT_SWITCH) > 0;
            break;
        }
        case AP_SWAP_LIMITS:{
            ret_val = motor->ap.limits_switched;
            break;
        }
        case AP_START_VEL: {
            memcpy(&ret_val, &(motor->ap.minimum_speed),4);
            break;
        }
        case AP_RIGHT_SW_POLAR:{
            ret_val = (motor->ap.limits_polarity & RIGHT_SWITCH) > 0;
            break;
        }
        case AP_LEFT_SW_POLAR:{
            ret_val = (motor->ap.limits_polarity & LEFT_SWITCH) > 0;
            break;
        }
        case AP_MICROSTEP_RESOLUTION:{
            ret_val = motor->ap.microstep_resolution;
            break;
        }
        case AP_ENDS_DISTANCE:{
            ret_val = motor->ap.end_switch_distance;
            break;
        }
        case AP_REVERSE_SHAFT:{
            ret_val = motor->ap.reverse_shaft;
            break;
        }
    };
    return ret_val;
}

StepperStatusCode stepper_set_axis_param(stepper_motor_t* motor, AxisParamType param, uint32_t value_enc){
    
    switch (param){
        case AP_TARGET_POS:{
            return SSC_INVALID_TYPE;
        }
        case AP_ACTUAL_POS:{
            return SSC_INVALID_TYPE;
        }
        case AP_TARGET_VEL:{
            memcpy(&(motor->ap.target_speed), &value_enc,4);
            return SSC_OK;
        }
        case AP_ACTUAL_VEL:{
            return SSC_INVALID_TYPE;
        }
        case AP_MAX_VEL:{
            uint32_t buf;
            memcpy(&(buf), &value_enc,4);
            if (buf <= motor->pulse_timer->base_clk/2){
                motor->ap.maximum_speed = buf;
                return SSC_OK;
            }else{
                return SSC_INVALID_VALUE;
            }
        }
        case AP_MAX_ACC:{
            memcpy(&(motor->ap.acceleration), &value_enc,4);
            return SSC_OK;
        }
        case AP_POS_REACHED:{
            return SSC_INVALID_TYPE;
        }
        case AP_HOME_SW_STATE:{
            return SSC_INVALID_TYPE;
        }
        case AP_RIGHT_SW_STATE:{
            return SSC_INVALID_TYPE;
        }
        case AP_LEFT_SW_STATE:{
            return SSC_INVALID_TYPE;
        }
        case AP_RIGHT_SW_DISABLE:{
            if (value_enc > 0){
                motor->ap.limits_disabled |= RIGHT_SWITCH;
            }else{
                motor->ap.limits_disabled &= ~RIGHT_SWITCH;
            }
            return SSC_OK;
        }
        case AP_LEFT_SW_DISABLE:{
            if (value_enc > 0){
                motor->ap.limits_disabled |= LEFT_SWITCH;
            }else{
                motor->ap.limits_disabled &= ~LEFT_SWITCH;
            }
            return SSC_OK;
        }
        case AP_SWAP_LIMITS:{
            motor->ap.limits_switched = value_enc > 0? 1 : 0;
            return SSC_OK;
        }
        case AP_START_VEL:{
            memcpy(&(motor->ap.minimum_speed), &value_enc,4);
            return SSC_OK;
        }
        case AP_RIGHT_SW_POLAR:{
            if (value_enc > 0){
                motor->ap.limits_polarity |= RIGHT_SWITCH;
            }else{
                motor->ap.limits_polarity &= ~RIGHT_SWITCH;
            }
            return SSC_OK;
        }
        case AP_LEFT_SW_POLAR:{
            if (value_enc > 0){
                motor->ap.limits_polarity |= LEFT_SWITCH;
            }else{
                motor->ap.limits_polarity &= ~LEFT_SWITCH;
            }
            return SSC_OK;
        }
        case AP_MICROSTEP_RESOLUTION:{
            if (value_enc <= 4){
                motor->ap.microstep_resolution = value_enc;
                return SSC_OK;
            }
            else{
                return SSC_INVALID_VALUE;
            }
        }
        case AP_ENDS_DISTANCE:{
            return SSC_INVALID_TYPE;
        }
        case AP_REVERSE_SHAFT:{
            motor->ap.reverse_shaft = value_enc > 0? 1 : 0;
            return SSC_OK;
        }
        default: return SSC_INVALID_TYPE;
    };
    return SSC_OK;
}


StepperStatusCode stepper_decode_command( uint8_t* cmd_buffer, stepper_command_t* cmd ){
    cmd->module_address = cmd_buffer[0];
    cmd->cmd_id = cmd_buffer[1];
    cmd->cmd_type = cmd_buffer[2];
    cmd->cmd_bank = cmd_buffer[3];
    cmd->value = (cmd_buffer[4] << 24 )
               + (cmd_buffer[5] << 16 )
               + (cmd_buffer[6] << 8  )
               + cmd_buffer[7];
    cmd->checksum = cmd_buffer[8];
    uint8_t cksm = 0;
    for (uint8_t i = 0; i<8;++i){
        cksm += cmd_buffer[i];
    }
    if (cksm == cmd->checksum){
        return SSC_OK;
    }else{
        return SSC_WRONG_CHECKSUM;
    }
}
void stepper_encode_reply( uint8_t* reply_buffer, stepper_reply_t* reply ){
    reply_buffer[0] = reply->reply_address;
    reply_buffer[1] = reply->module_address;
    reply_buffer[2] = reply->status_code;
    reply_buffer[3] = reply->cmd_id;
    reply_buffer[4] = (reply->value >> 24) & 0xff;
    reply_buffer[5] = (reply->value >> 16) & 0xff;
    reply_buffer[6] = (reply->value >>  8) & 0xff;
    reply_buffer[7] = (reply->value)       & 0xff;
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; ++i){
        checksum += reply_buffer[i];
    }
    reply_buffer[8] = checksum;
}



void stepper_com_action(stepper_motor_t* motor, stepper_com_buffer_t* com_buffer){
    static stepper_command_t cmd;
    static stepper_reply_t reply;

    reply.status_code = stepper_decode_command( com_buffer->cmd_buffer, &cmd );
    if (reply.status_code == SSC_OK){
        stepper_handle_command(motor, &cmd, &reply);
    }
    stepper_encode_reply(com_buffer->reply_buffer, &reply);
}
