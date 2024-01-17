#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


void forward_pid(float TARGET_L, float TARGET_R) {
	pros::Motor lfb_base(lfb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lft_base(lft_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lbb_base(lbb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lbt_base(lbt_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rfb_base(rfb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rft_base(rft_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rbb_base(rbb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rbt_base(rbt_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Rotation trackingwheel_l(twl_port);
	pros::Rotation trackingwheel_r(twr_port);
	trackingwheel_r.set_reversed(true);
	trackingwheel_l.set_position(0);
	trackingwheel_r.set_position(0);
	// pros::Imu imu_sensor(imu_port);
	// imu_sensor.set_heading(90);

	double powerL = 0;
	double powerR = 0;
	double targPowerL = 0;
	double targPowerR = 0;
	double deltaErrorLeft = 0;
	double deltaErrorRight = 0;
	double encdleft = 0;
	double encdright = 0;
	double errorLeft = 0;
	double errorRight = 0;
	double prevErrorLeft = 0;
	double prevErrorRight = 0;
	double totalErrorLeft = 0;
	double totalErrorRight = 0;

	bool l_move = true;
	bool r_move = true;

	bool ramp_l = true;
	bool ramp_r = true;

	int acceleration = 0;
	uint32_t start_time = pros::millis();
	uint32_t current_time;

	while(l_move || r_move){
		encdleft = trackingwheel_l.get_position() * pi * tw_diameter / 36000;
		encdright = trackingwheel_r.get_position() * pi * tw_diameter / 36000;
		errorLeft = TARGET_L - encdleft;
		errorRight = TARGET_R - encdright;
		totalErrorLeft += errorLeft;
		totalErrorRight += errorRight;
		deltaErrorLeft = errorLeft - prevErrorLeft;
		deltaErrorRight = errorRight - prevErrorRight;
		current_time = pros::millis();

		std::cout << "encdleft: "  << encdleft << "  ||  encdright: " << encdright << std::endl;
		// std::cout << "acceleration: " << acceleration << std::endl;
		// std::cout << "start_time: " << start_time << std::endl;
		// std::cout << "current_time: " << current_time << std::endl;
		// check if left and right reached target
		if ( fabs(errorLeft) <= base_error) {
			powerL = 0;
			l_move = false;
		}
		else {
			powerL = base_kp * errorLeft + base_ki * totalErrorLeft + base_kd * deltaErrorLeft;
		}
		if ( fabs(errorRight) <= base_error) {
			powerR = 0;
			r_move = false;
		}
		else {
			powerR = base_kp * errorRight + base_ki * totalErrorRight + base_kd * deltaErrorRight;
		}

		//set rate of change of rpm
		// if ( ((current_time - start_time) %  50) <= 1) {
		// 	acceleration += 20;
		// }

		// if ( (powerL < targPowerL) && ramp_l) {
		// 	powerL = acceleration;
		// 	std::cout << "ramp up left" << std::endl;
		// }
		// else {
		// 	ramp_l = false;
		// 	powerL = targPowerL;
		// }
		// if ((powerR < targPowerR) && ramp_r) {
		// 	powerR = acceleration;
		// 	std::cout << "ramp up right" << std::endl;
		// }
		// else {
		// 	ramp_r = false;
		// 	powerR = targPowerR;
		// }

		// if (powerL >= base_max_rpm) {
		// 	powerL = base_max_rpm;
		// }
		// if (powerR >= base_max_rpm) {
		// 	powerR = base_max_rpm;
		// }
		lft_base.move_velocity(powerL);
		lfb_base.move_velocity(powerL);
		lbt_base.move_velocity(powerL);
		lbb_base.move_velocity(powerL);
		rft_base.move_velocity(powerR);
		rfb_base.move_velocity(powerR);
		rbt_base.move_velocity(powerR);
		rbb_base.move_velocity(powerR);
		prevErrorLeft = errorLeft;
		prevErrorRight = errorRight;
		pros::delay(2);
	}
	std::cout << "at target" << std::endl;
	lft_base.move_velocity(0);
	lfb_base.move_velocity(0);
	lbt_base.move_velocity(0);
	lbb_base.move_velocity(0);
	rft_base.move_velocity(0);
	rfb_base.move_velocity(0);
	rbt_base.move_velocity(0);
	rbb_base.move_velocity(0);
	acceleration = 0;
}

void turn_pid(double TARGET_ANGLE, bool clockwise) {
	pros::Motor lfb_base(lfb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lft_base(lft_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lbb_base(lbb_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor lbt_base(lbt_port, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rfb_base(rfb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rft_base(rft_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rbb_base(rbb_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rbt_base(rbt_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Rotation trackingwheel_l(twl_port);
	pros::Rotation trackingwheel_r(twr_port);
	trackingwheel_r.set_reversed(true);
	trackingwheel_l.set_position(0);
	trackingwheel_r.set_position(0);
	// pros::Imu imu_sensor(imu_port);
	// imu_sensor.set_heading(90);

	double powerL = 0;
	double powerR = 0;
	// double targPowerL = 0;
	// double targPowerR = 0;
	double deltaErrorLeft = 0;
	double deltaErrorRight = 0;
	double encdleft = 0;
	double encdright = 0;
	double errorLeft = 0;
	double errorRight = 0;
	double prevErrorLeft = 0;
	double prevErrorRight = 0;
	double totalErrorLeft = 0;
	double totalErrorRight = 0;

	bool l_move = true;
	bool r_move = true;

	int direction_l;

	if (clockwise) {
		direction_l = 1;
	}
	else {
		direction_l = -1;
	}

	double TARGET_L = direction_l * TARGET_ANGLE * pi * base_diameter / 360;
	double TARGET_R = (-direction_l) * TARGET_ANGLE * pi * base_diameter / 360;

	// start_timestamp = pros::millis();
	// acceleration = 0;

	while(l_move || r_move){
		encdleft = trackingwheel_l.get_position() * pi * tw_diameter / 36000;
		encdright = trackingwheel_r.get_position() * pi * tw_diameter / 36000;
		errorLeft = TARGET_L - encdleft;
		errorRight = TARGET_R - encdright;
		totalErrorLeft += errorLeft;
		totalErrorRight += errorRight;
		deltaErrorLeft = errorLeft - prevErrorLeft;
		deltaErrorRight = errorRight - prevErrorRight;

		std::cout << "encdleft: "  << encdleft << "  ||  encdright: " << encdright << std::endl;

		
		if ( fabs(errorLeft) <= base_error) {
			powerL = 0;
			l_move = false;
		}
		else {
			powerL = turn_kp * errorLeft + turn_ki * totalErrorLeft + turn_kd * deltaErrorLeft;
		}
		if ( fabs(errorRight) <= base_error) {
			powerR = 0;
			r_move = false;
		}
		else {
			powerR = turn_kp * errorRight + turn_ki * totalErrorRight + turn_kd * deltaErrorRight;
		}

		if (powerL >= turn_max_rpm) {
			powerL = turn_max_rpm;
		}
		if (powerR >= turn_max_rpm) {
			powerR = turn_max_rpm;
		}

		lft_base.move_velocity(powerL);
		lfb_base.move_velocity(powerL);
		lbt_base.move_velocity(powerL);
		lbb_base.move_velocity(powerL);
		rft_base.move_velocity(powerR);
		rfb_base.move_velocity(powerR);
		rbt_base.move_velocity(powerR);
		rbb_base.move_velocity(powerR);
		prevErrorLeft = errorLeft;
		prevErrorRight = errorRight;
		pros::delay(2);
	}
	std::cout << "at target" << std::endl;
	lft_base.move_velocity(0);
	lfb_base.move_velocity(0);
	lbt_base.move_velocity(0);
	lbb_base.move_velocity(0);
	rft_base.move_velocity(0);
	rfb_base.move_velocity(0);
	rbt_base.move_velocity(0);
	rbb_base.move_velocity(0);
}