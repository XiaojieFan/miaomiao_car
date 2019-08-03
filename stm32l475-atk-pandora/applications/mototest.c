#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "motor.h"
#include "wheel.h"
#include "single_pwm_motor.h"
#include "single_phase_encoder.h"
#include "inc_pid_controller.h"
#include "kinematics.h"
#include "chassis.h"
#include <rtdbg.h>

#define THREAD_PRIORITY 5
#define THREAD_STACK_SIZE      1024
#define THREAD_TIMESLICE        5
static rt_thread_t tid1 = RT_NULL;

#define LEFT_ENCODER_PIN  GET_PIN(B, 8)
#define RIGHT_ENCODER_PIN  GET_PIN(B,9)
#define PULSE_PER_REVOL    20
#define WHEEL_RADIUS     0.033
#define GEAR_RATIO       1
#define WHEEL_DIST_X     0.24
#define WHEEL_DIST_Y     0.14
#define SAMPLE_TIME      50
#define PID_SAMPLE_TIME  50
//PD13-  PWM4-2
//PD12-  PWM4-1

//PD14-  PWM4-3
//PD15-  PWM4-4
// MOTOR
#define LEFT_FORWARD_PWM            "pwm4"      //PD13
#define LEFT_FORWARD_PWM_CHANNEL     2
#define LEFT_FORWARD_STRAIGHT_PIN    GET_PIN(C, 4)
#define LEFT_FORWARD_BACK_PIN        GET_PIN(A, 4) 

#define LEFT_BACKWARD_PWM           "pwm4"      //PD15
#define LEFT_BACKWARD_PWM_CHANNEL   4
#define LEFT_BACKWARD_STRAIGHT_PIN    GET_PIN(B, 11)
#define LEFT_BACKWARD_BACK_PIN        GET_PIN(B, 10)

#define RIGHT_FORWARD_PWM           "pwm4"//"pwm2"  //PD12 
#define RIGHT_FORWARD_PWM_CHANNEL    1
#define RIGHT_FORWARD_STRAIGHT_PIN    GET_PIN(B, 13)
#define RIGHT_FORWARD_BACK_PIN        GET_PIN(B, 12)

#define RIGHT_BACKWARD_PWM          "pwm4"//"pwm2"  //PD14
#define RIGHT_BACKWARD_PWM_CHANNEL   3
#define RIGHT_BACKWARD_STRAIGHT_PIN    GET_PIN(B, 15)
#define RIGHT_BACKWARD_BACK_PIN        GET_PIN(B, 14)

float kp = 0.5;
float ki = 0;
float kd = 0;

static void motor_entry(void *parameter)
{
	rt_uint32_t count = 0;
	motor_t letf_moto = RT_NULL;
	motor_t right_moto = RT_NULL;
	chassis_t chas;
	struct velocity target_vel;
	// 1. Initialize two wheels - left and right
wheel_t* c_wheels = (wheel_t*) rt_malloc(sizeof(wheel_t) * 4);
if (c_wheels == RT_NULL)
{
    LOG_D("Failed to malloc memory for wheels");
}

// 1.1 Create two motors
//motor_t left_motor  = motor_create(left_motor_init,  left_motor_enable,  left_motor_disable,  left_motor_set_speed,  DC_MOTOR);
//motor_t right_motor = motor_create(right_motor_init, right_motor_enable, right_motor_disable, right_motor_set_speed, DC_MOTOR);
single_pwm_motor_t left_forward_motor = single_pwm_motor_create(LEFT_FORWARD_PWM,LEFT_FORWARD_PWM_CHANNEL, LEFT_FORWARD_STRAIGHT_PIN, LEFT_FORWARD_BACK_PIN);
single_pwm_motor_t left_backward_motor = single_pwm_motor_create(LEFT_BACKWARD_PWM,LEFT_BACKWARD_PWM_CHANNEL, LEFT_BACKWARD_STRAIGHT_PIN, LEFT_BACKWARD_BACK_PIN);
single_pwm_motor_t right_forward_motor = single_pwm_motor_create(RIGHT_FORWARD_PWM,RIGHT_FORWARD_PWM_CHANNEL, RIGHT_FORWARD_STRAIGHT_PIN, RIGHT_FORWARD_BACK_PIN);
single_pwm_motor_t right_backward_motor = single_pwm_motor_create(RIGHT_BACKWARD_PWM,RIGHT_BACKWARD_PWM_CHANNEL, RIGHT_BACKWARD_STRAIGHT_PIN, RIGHT_BACKWARD_BACK_PIN);


// 1.2 Create two encoders
//encoder_t left_encoder  = encoder_create(LEFT_ENCODER_PIN, PULSE_PER_REVOL);
//encoder_t right_encoder = encoder_create(RIGHT_ENCODER_PIN, PULSE_PER_REVOL);
single_phase_encoder_t left_forward_encoder = single_phase_encoder_create(LEFT_ENCODER_PIN,PULSE_PER_REVOL );
single_phase_encoder_t right_forward_encoder = single_phase_encoder_create(RIGHT_ENCODER_PIN,PULSE_PER_REVOL);

// 1.3 Create two pid contollers
//pid_control_t left_pid  = pid_create();
//pid_control_t right_pid = pid_create();
inc_pid_controller_t left_pid = inc_pid_controller_create(kp,ki,kd);
inc_pid_controller_t right_pid = inc_pid_controller_create(kp, ki,kd);
// 1.4 Add two wheels
//wheel_t wheel_create(motor_t w_motor, encoder_t w_encoder, controller_t w_controller, float radius, rt_uint16_t gear_ratio)

c_wheels[0] = wheel_create((motor_t)left_forward_motor,  (encoder_t)left_forward_encoder,(controller_t)left_pid,   WHEEL_RADIUS, GEAR_RATIO);//left_pid
c_wheels[2] = wheel_create((motor_t)left_backward_motor, (encoder_t)left_forward_encoder,(controller_t)left_pid,  WHEEL_RADIUS, GEAR_RATIO);//left_pid
c_wheels[1] = wheel_create((motor_t)right_forward_motor, (encoder_t)right_forward_encoder,(controller_t)right_pid, WHEEL_RADIUS, GEAR_RATIO);//left_pid
c_wheels[3] = wheel_create((motor_t)right_backward_motor, (encoder_t)right_forward_encoder,(controller_t)right_pid,  WHEEL_RADIUS, GEAR_RATIO);//left_pid


// 2. Iinialize Kinematics - Two Wheel Differential Drive
kinematics_t c_kinematics = kinematics_create(FOUR_WD, WHEEL_DIST_X, WHEEL_DIST_Y, WHEEL_RADIUS);

// 3. Initialize Chassis
chas = chassis_create(c_wheels, c_kinematics);

// 4. Enable Chassis
//chassis_enable(chas);

// Set Sample time
encoder_set_sample_time(chas->c_wheels[0]->w_encoder, SAMPLE_TIME);
encoder_set_sample_time(chas->c_wheels[1]->w_encoder, SAMPLE_TIME);
encoder_set_sample_time(chas->c_wheels[2]->w_encoder, SAMPLE_TIME);
encoder_set_sample_time(chas->c_wheels[3]->w_encoder, SAMPLE_TIME);
//pid_set_sample_time(chas->c_wheels[0]->w_pid, PID_SAMPLE_TIME);
//pid_set_sample_time(chas->c_wheels[1]->w_pid, PID_SAMPLE_TIME);

// 4. Enable Chassis
chassis_enable(chas);
// Turn left
target_vel.linear_x = 0;   // m/s
target_vel.linear_y = 0;    // m/s
target_vel.angular_z = PI / 4; // rad/s

chassis_set_velocity(chas, target_vel);
rt_thread_mdelay(500);

// Turn right
target_vel.linear_x = 0;   // m/s
target_vel.linear_y = 0;    // m/s
target_vel.angular_z = - PI / 4; // rad/s


chassis_set_velocity(chas, target_vel);
rt_thread_mdelay(500);


// Go straight
target_vel.linear_x = 0.06;   // m/s
target_vel.linear_y = 0;    // m/s
target_vel.angular_z = 0;
while(1)
{
 chassis_set_velocity(chas, target_vel);
 rt_thread_mdelay(500);
}
// Stop
target_vel.linear_x = 0.0;   // m/s
target_vel.linear_y = 0;    // m/s
target_vel.angular_z = 0; // rad/s

chassis_set_velocity(chas, target_vel);
rt_thread_mdelay(500);

while(1)
{
    chassis_update(chas);
    rt_thread_mdelay(50);
}

}

void moto_sample(void)
{
	
   tid1 = rt_thread_create("motor_run",motor_entry,RT_NULL,
														THREAD_STACK_SIZE,
														THREAD_PRIORITY,THREAD_TIMESLICE);
	  if (tid1 != RT_NULL)
		{
			rt_thread_startup(tid1);
		}
	
}

MSH_CMD_EXPORT(moto_sample, moto test);