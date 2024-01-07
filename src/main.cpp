#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "globals.hpp"

double powerL;
double powerR;
double targPowerL;
double targPowerR;
double deltaErrorLeft;
double deltaErrorRight;
double encdleft;
double encdright;
double errorLeft;
double errorRight;
double prevErrorLeft;
double prevErrorRight;
double totalErrorLeft;
double totalErrorRight;
double LEFTTARGET;
double RIGHTTARGET;

bool l_move;
bool r_move;

double imu_heading;

// double start_timestamp;
// double current_time;
// double acceleration;

void forward_pid(double TARGET_L, double TARGET_R) {
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
	pros::Imu imu_sensor(imu_port);
	imu_sensor.set_heading(90);

	powerL = 0;
	powerR = 0;
	targPowerL = 0;
	targPowerR = 0;
	deltaErrorLeft = 0;
	deltaErrorRight = 0;
	encdleft = 0;
	encdright = 0;
	errorLeft = 0;
	errorRight = 0;
	prevErrorLeft = 0;
	prevErrorRight = 0;
	totalErrorLeft = 0;
	totalErrorRight = 0;

	l_move = true;
	r_move = true;

	imu_heading = 0;

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

		imu_heading = imu_sensor.get_heading();

		std::cout << "encdleft: "  << encdleft << "  ||  encdright: " << encdright << std::endl;

		// current_time = pros::millis();
		// acceleration = (current_time - start_timestamp) / 15.0;
		// if (powerL <= targPowerL) {
		// 	powerL += acceleration;
		// }
		// else {
		// 	powerL = targPowerL;
		// }
		// if (powerR <= targPowerR) {
		// 	powerR += acceleration;
		// }
		// else {
		// 	powerR = targPowerR;
		// }
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
		// if (powerL > base_max_rpm) {
		// 	powerL = base_max_rpm;
		// }
		// if (powerR > base_max_rpm) {
		// 	powerR = base_max_rpm;
		// }
		
		// if (imu_heading > 90) {
		// 	powerR -= 20;
		// }
		// else if (imu_heading < 90) {
		// 	powerL -= 20;
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
}

bool l_brake = false;
bool r_brake = false;

// base_brake not to be used with movement functions.
void left_brake() {
	pros::Motor lfb_base(lfb_port);
	pros::Motor lft_base(lft_port);
	pros::Motor lbb_base(lbb_port);
	pros::Motor lbt_base(lbt_port);
	while(true){
		while(l_brake){
			lft_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lfb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lbb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lbt_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lft_base.brake();
			lfb_base.brake();
			lbt_base.brake();
			lbb_base.brake();
			pros::delay(2);
		}
	}
}
void right_brake() {
	pros::Motor rfb_base(rfb_port);
	pros::Motor rft_base(rft_port);
	pros::Motor rbb_base(rbb_port);
	pros::Motor rbt_base(rbt_port);
	while (true) {
		while (r_brake) {
			rft_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rfb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rbb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rbt_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rft_base.brake();
			rfb_base.brake();
			rbt_base.brake();
			rbb_base.brake();
			pros::delay(2);
		}
	}
}

// ratio of turn dist = turn_radius : turn_radius+base
// void turn() {

// }

bool shoot = false;
int cata_state = 0;
double posL;
double posR;
double firing_angle = 150;
double mesh_angle = 359;
double rewind_target;
bool step_l;
bool step_r;

double total_error_l;
double total_error_r;
double errorL;
double errorR;

void cata_control_new() {
	pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);
	catarot_l.set_reversed(true);
	catarot_r.set_reversed(true);

	while (true) {
		posL = catarot_l.get_angle() / 100;
		posR = catarot_r.get_angle() / 100;
		step_l = false;
		step_r = false;

		// state 0 - check angles. if beyond firing angle, proceed to state 1.
		// state 1 - mesh gears. meshing should be around 358 to 5 degrees. after meshing, proceed to state 2.
		// state 2 - wind cata to loading position. loading position is about 145 to 150 degrees.
		// state 3 - ready to fire. if shoot is true, fire and set state to 0.

		switch(cata_state) {
			case 0:
				if (posL > firing_angle && posR > firing_angle) {
					lc.brake();
					rc.brake();
					pros::delay(2);
					cata_state = 1;
					step_l = false;
					step_r = false;
				}
				// else if ( (posL >= mesh_angle || posL <= firing_angle) && (posR >= mesh_angle || posR <= firing_angle)) {
				// 	if (fabs(posL - posR) <= 7) {
				// 		cata_state = 2;
				// 	}
				// }
				else {
					// std::cout << "cata jammed" << std::endl;
					std::cout << "reset cata" << std::endl;
					std::cout << posL << " : " << posR << std::endl;
				}
				break;
			case 1: //meshing
				// std::cout << "mesh" << std::endl;
				if (posL < mesh_angle) {
					lc.move(-40);
					pros::delay(2);
					// std::cout << "posL Mesh " << posL << std::endl;
				}
				if (posL >= mesh_angle || posL <= firing_angle) {
					lc.move(10);
					pros::delay(2);
					step_l = true;
					// std::cout << "posL M "  << posL << std::endl;
				}
				if (posR < mesh_angle) {
					rc.move(-40);
					pros::delay(2);
					// std::cout << "posR Mesh "  << posR << std::endl;
				}
				if (posR >= mesh_angle || posR <= firing_angle) {
					rc.move(10);
					pros::delay(2);
					step_r = true;
					// std::cout << "posR M "  << posR << std::endl;
				}
				if (step_r) {
					cata_state = 2;
					rewind_target = 130;
					step_l = false;
					step_r = false;
					// std::cout << "cata_state" << std::endl;
					total_error_l = 0;
					total_error_r = 0;
				}
				break;
			case 2: //stepping
				std::cout << rewind_target << std::endl;
				std::cout << "posL " << posL << std::endl;
				std::cout << "posR " << posR << std::endl;
				
				if (posL >= 350) {
					posL -= 360;
				}
				if (posR >= 350) {
					posR -= 360;
				}

				std::cout << "posL " << posL << std::endl;
				std::cout << "posR " << posR << std::endl;
				// errorL = rewind_target - posL;
				// errorR = rewind_target - posR;
				// total_error_l += errorL;
				// total_error_r += errorR;
				if (rewind_target <= posL) {
					step_l = true;
					// lc.move(10);
					std::cout << "left at step target" << std::endl;
					// pros::delay(2);
				}
				else {
					step_l = false;
					lc.move_velocity(-100); // - (cata_kp * fabs(errorL)) - (cata_ki * total_error_l));
					std::cout << "left moving" << std::endl;
					// pros::delay(2);
				}
				if (rewind_target <= posR) {
					step_r = true;
					// rc.move(10);
					std::cout << "right at step target3" << std::endl;
					// pros::delay(2);
				}
				else {
					step_r = false;
					rc.move_velocity(-100); // - (cata_kp * fabs(errorR)) -(cata_ki * total_error_r));
					std::cout << "right moving" << std::endl;
					// pros::delay(2);
				}
				if (step_l && step_r) {
					rewind_target += 1; //this is where stepping angle is set 
					std::cout << "done stepping" << std::endl;
					// pros::delay(2);
				}
				if (posL >= firing_angle) {
					lc.move(25);
					// pros::delay(2);
				}
				if (posR >= firing_angle) {
					rc.move(25);
					// pros::delay(2);
				}
				if (rewind_target >= firing_angle) {
					lc.move(25);
					rc.move(25);
					// pros::delay(2);
					cata_state = 3;
				}
				if (posL == firing_angle && posR == firing_angle) {
					lc.move(25);
					rc.move(25);
					pros::delay(10);
					lc.move(0);
					rc.move(0);
					cata_state = 3;
				}
				// std::cout << "powerL " << -100 - (cata_kp * fabs(errorL)) - (cata_ki * total_error_l) << std::endl;
				// std::cout << "powerR " << -127 - (cata_kp * fabs(errorR)) -(cata_ki * total_error_r) << std::endl;

				// std::cout << "total_error_l " << total_error_l << std::endl;
				// std::cout << "total_error_r " << total_error_r << std::endl;
				break;
			case 3:
				// std::cout << "ready" << std::endl;
				if (shoot) {
					lc.move_velocity(-100);
					rc.move_velocity(-100);
					pros::delay(300);
					lc.move(10);
					rc.move(10);
					cata_state = 0;
					shoot = false;
				}
				else {
					lc.move(10);
					rc.move(10);
					pros::delay(2);
				}
				break;
		}
	}
}


bool IntakeTargetPosUp = true;
int RollerPow = 0;
float flipper_error;
float prev_flipper_error;
float flipper_d;
float total_flipper_error;
double flipper_pow;

void flipper_pid() {
    using namespace pros;
    pros::Motor f_arm(flipper_motor);
    pros::Motor f_roller(flipper_roller_motor);
    pros::Rotation flipperrot(flipperrot_port);

    // IntakeTargetPosUp = true; //move to up position    
    // IntakeTargetPosUp = false; //move to down position

    // RollerPow = 127;//roller outtake    
    // RollerPow = -127; //roller intake
    //RollerPow = 0; //roller stop
    while (true) {
        int currentPos = flipperrot.get_position() / 100;
		
        if (IntakeTargetPosUp) {
            flipper_error = flipper_targetUp - currentPos;
		}
        else {
            flipper_error = flipper_targetDown - currentPos;
		}

        flipper_d = flipper_error - prev_flipper_error;
        prev_flipper_error = flipper_error;
        total_flipper_error += flipper_error;
        flipper_pow = flipper_error * flipper_kp + total_flipper_error * flipper_ki + flipper_d * flipper_kd;
        f_arm.move(flipper_pow);
        f_roller.move(RollerPow);
        
		// printf("flipper_rot: %d \n", currentPos);
        // printf("flipper power: %i \n", flipper_pow);
    }
}

void initialize() {
	//controller
    pros::Controller master(CONTROLLER_MASTER);

	//base
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
	trackingwheel_l.set_position(0);
	trackingwheel_r.set_position(0);
	trackingwheel_r.set_reversed(true);

	// pros::Imu imu_sensor(imu_port);
	// imu_sensor.reset();

	//cata
	pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);

	//flipper
    pros::Motor f_arm(flipper_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor f_roller(flipper_roller_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation flipperrot(flipperrot_port);

	//front rollers
    pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

	// pros::Task cata(cata_control_new);
	pros::Task flipper(flipper_pid);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
	forward_pid(600, 600);
	
}


void opcontrol() {
	//controller
    pros::Controller master(CONTROLLER_MASTER);

	//base
    pros::Motor lfb_base(lfb_port);
	pros::Motor lft_base(lft_port);
	pros::Motor lbb_base(lbb_port);
	pros::Motor lbt_base(lbt_port);
	pros::Motor rfb_base(rfb_port);
	pros::Motor rft_base(rft_port);
	pros::Motor rbb_base(rbb_port);
	pros::Motor rbt_base(rbt_port);
	
	//cata motors
    pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);

	//front rollers
    pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

	bool tankdrive = true; //drive mode control
	uint32_t timestamp1;
	uint32_t timestamp2;
	while(true){
        //base control
        double left, right;
        
		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive; //tank toggle

        if(tankdrive) {
            left = master.get_analog(ANALOG_LEFT_Y);
            right = master.get_analog(ANALOG_RIGHT_Y);
        }     
        else {
            double power =  master.get_analog(ANALOG_LEFT_Y);
            double turn = master.get_analog(ANALOG_RIGHT_X);
            left = power + turn;
            right = power - turn;
        }
        lfb_base.move(left);
        lft_base.move(left);
        lbb_base.move(left);
		lbt_base.move(left);
        rfb_base.move(right);
        rft_base.move(right);
        rbb_base.move(right);
		rbt_base.move(right);

		// if(master.get_digital(DIGITAL_R1) && CataIdle){
		// 	CataIdle = false;
		// 	printf("Cata FIring flag 1: %d \n", CataIdle);
        // }

		if(master.get_digital(DIGITAL_R1)){
			shoot = true;
        }

		//flipper control
        //update target speeds for I and update target position for flipper
        if(master.get_digital_new_press(DIGITAL_Y))
            IntakeTargetPosUp = true; //move to up position
        else if(master.get_digital_new_press(DIGITAL_B))
            IntakeTargetPosUp = false; //move to down position
		
		//flipper roller
		if(master.get_digital(DIGITAL_DOWN))
            RollerPow = 127;//roller outtake
        else if(master.get_digital(DIGITAL_RIGHT))
            RollerPow = -127; //roller intake
        else
            RollerPow = 0; //roller stop
		
		if (master.get_digital(DIGITAL_R2)){
			front_roller.set_zero_position(0);
			IntakeTargetPosUp = false;
			pros::delay(400);
			timestamp1 = pros::millis();
			printf("time: %d \n", timestamp1);
			RollerPow = 127;
			pros::delay(500);
			timestamp2 = pros::millis();
			uint32_t total_time = (timestamp2 - timestamp1);
			double position = front_roller.get_position();
			double rpm = position / (total_time / 60000);
			RollerPow = 0;
			pros::delay(10);
			IntakeTargetPosUp = true;
			printf("Total Time: %d \n", total_time);
			printf("Position: %d \n", position);
			printf("RPM: %d \n", rpm);
			pros::delay(10);
    }

		//side rollers control
        front_roller.move(100 * (master.get_digital(DIGITAL_L2) - master.get_digital(DIGITAL_L1)));

		pros::delay(5);
	}
}