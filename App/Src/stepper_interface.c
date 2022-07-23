#include "stepper_interface.h"

void istepper_calculate_motion_params(stepper_motor_t* motor){

    motor->mp.start_position = motor->ap.actual_position;
    motor->mp.abort_move = 0;
    motor->mp.finish_move = 0;
    motor->mp.direction = motor->ap.target_position > motor->ap.actual_position;
    if (motor->mp.direction){
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
    istepper_set_pulse_timer(motor);
	//HAL_TIM_PWM_Start(motor->pulse_timer->htim, motor->pulse_timer->channel);
    motor->pulse_timer->htim->Instance->CR1 |= TIM_CR1_CEN;
}


void istepper_finish_movement(stepper_motor_t* motor){

    //HAL_TIM_PWM_Stop(motor->pulse_timer->htim, motor->pulse_timer->channel);
    motor->pulse_timer->htim->Instance->CR1 &= ~TIM_CR1_CEN;

	motor->ap.actual_position = motor->mp.start_position + motor->mp.nsteps;
    motor->ap.actual_speed = 0;
	motor->ap.target_position_reached = 1;
	motor->mp.finish_move = 0;
}


void stepper_abort_movement( stepper_motor_t* motor ){
    //HAL_TIM_PWM_Stop(motor->pulse_timer->htim, motor->pulse_timer->channel);
    motor->pulse_timer->htim->Instance->CR1 &= ~TIM_CR1_CEN;

	uint32_t current_nsteps = motor->count_timer->htim->Instance->CNT;
    motor->ap.actual_position = motor->mp.start_position + current_nsteps;
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

        motor->ap.actual_position = motor->mp.start_position + current_nsteps;

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
