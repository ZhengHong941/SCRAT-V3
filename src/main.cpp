#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "globals.hpp"

double prevErrorLeft = 0;
double prevErrorRight = 0;
double encdleft = 0;
double encdright = 0;
double errorLeft = 0;
double errorRight = 0;
double LEFTTARGET = 0;
double RIGHTTARGET = 0;
double deltaErrorLeft = 0;
double deltaErrorRight = 0;
double powerL = 0;
double powerR = 0;
double auton_pos[2] = {0,0};
int i = 0;
bool brake = false;
bool next_movement = true;
int arrlen = 0;

bool IntakeTargetPosUp = true;

// void pidvalues(double* targleft, double* targright){
// 	while (i < arrlen) {
// 		if(next_movement){
// 			auton_pos[0] = auton_pos[0] + targleft[i];
// 			auton_pos[1] = auton_pos[1] + targright[i];
// 			LEFTTARGET = auton_pos[0];
// 			errorLeft = auton_pos[0];
// 			RIGHTTARGET = auton_pos[1];
// 			errorRight = auton_pos[1];
// 			pros::delay(2);
// 			i++;
// 			next_movement = false;
// 			pros::delay(2);
// 		}
// 	}
// }
void pidvalues(double targleft, double targright){
	LEFTTARGET = targleft;
	RIGHTTARGET = targright;
	// errorLeft = targleft;
	// errorRight = targright;
}

void pidmove() {
    // using namespace pros;
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
	while(true){
		// std::cout << "0" << std::endl;
		// pros::delay(20);
		encdleft = trackingwheel_l.get_position() * pi * 28 / 36000;
		encdright = trackingwheel_r.get_position() * pi * 28 / 36000;
		errorLeft = LEFTTARGET - encdleft;
		errorRight = RIGHTTARGET - encdright;
		deltaErrorLeft = errorLeft - prevErrorLeft;
		deltaErrorRight = errorRight - prevErrorRight;
		powerL = base_kp * errorLeft + base_ki * prevErrorLeft + base_kd * deltaErrorLeft; // divide 25 to prevent from going insane - kp must not be too high - when ki=0.1 kd=0,  
		powerR = base_kp * errorRight + base_ki * prevErrorRight + base_kd * deltaErrorRight;
		printf("encdleft: %f \n", encdleft);
		printf("encdright: %f \n", encdright);
		if (fabs(errorLeft) >= base_error ){
			lft_base.move(powerL);
			lfb_base.move(powerL);
			lbt_base.move(powerL);
			lbb_base.move(powerL);
			rft_base.move(powerR);
			rfb_base.move(powerR);
			rbt_base.move(powerR);
			rbb_base.move(powerR);
			printf("powerL: %f \n", powerL);
			printf("powerR: %f \n", powerR);
			pros::delay(2);
		}
		else {
			powerL = 0;
			powerR = 0;
			lft_base.move(powerL);
			lfb_base.move(powerL);
			lbt_base.move(powerL);
			lbb_base.move(powerL);
			rft_base.move(powerR);
			rfb_base.move(powerR);
			rbt_base.move(powerR);
			rbb_base.move(powerR);
			pros::delay(2);
		}
		prevErrorLeft = errorLeft;
		prevErrorRight = errorRight;
		pros::delay(2);
		
	}
}


void base_brake() {
	while(true){
		while(brake){
			pros::Motor lfb_base(lfb_port);
			pros::Motor lft_base(lft_port);
			pros::Motor lbb_base(lbb_port);
			pros::Motor lbt_base(lbt_port);
			pros::Motor rfb_base(rfb_port);
			pros::Motor rft_base(rft_port);
			pros::Motor rbb_base(rbb_port);
			pros::Motor rbt_base(rbt_port);
			lft_base.brake();
			lfb_base.brake();
			lbt_base.brake();
			lbb_base.brake();
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

bool CataIdle = true;
bool mesh = false;

int correctingPow_l;
int correctingPow_r;

bool shoot = false;

void cata_control_new() {
	pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);
	pros::Rotation cata_arm(cata_arm_port);

	int cata_state = 0;
	double posL;
	double posR;
	double firing_angle = 160;
	double mesh_angle = 359;
	double rewind_target;
	bool step_l;
	bool step_r;
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
				}
				else {
					std::cout << "cata jammed" << std::endl;
					std::cout << posL << posR << std::endl;
				}
				break;
			case 1:
				if (posL < mesh_angle) {
					lc.move(-20);
					pros::delay(2);
				}
				else {
					lc.brake();
					pros::delay(2);
				}
				if (posR < mesh_angle) {
					rc.move(-20);
					pros::delay(2);
				}
				else {
					rc.brake();
					pros::delay(2);
				}
				if ( ((posL >= mesh_angle) || (0 < posL < rewind_target)) && ((posR > mesh_angle) || (0 < posR < rewind_target)) ) {
					cata_state = 2;
					rewind_target = 5;
				}
				break;
			case 2:
				if (posL < rewind_target) {
					step_l = false;
					lc.move(-20);
					pros::delay(2);
				}
				else {
					step_l = true;
					lc.brake();
					pros::delay(2);
				}
				if (posR < rewind_target) {
					step_r = false;
					rc.move(-20);
					pros::delay(2);
				}
				else {
					step_r = true;
					rc.brake();
					pros::delay(2);
				}
				if (step_l && step_r) {
					rewind_target += 5;
				}
				if (rewind_target >= cata_target) {
					lc.brake();
					rc.brake();
					pros::delay(2);
					cata_state = 3;
				}
				break;
			case 3:
				if (shoot) {
					lc.move(-50);
					rc.move(-50);
					pros::delay(100);
					lc.brake();
					rc.brake();
					cata_state = 0;
				}
				else {
					lc.brake();
					rc.brake();
					pros::delay(2);
				}
				break;
		}
	}
}



void cata_control() {
	using namespace pros;
    pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);
	pros::Rotation cata_arm(cata_arm_port);

	printf("Cata FIring flag 2: %d \n", CataIdle);

	
	//the cata has a target angle for each slip gear, which it must reach. 
	//If it undershoots, the catapult is too high and balls cannot load, and if it overshoots the catapult will fire.

	while(true)
	{
		delay(1);
		if(!CataIdle) //to fire the catapult
		{
			printf("Cata FIring flag 3: %d \n", CataIdle);
 
			lc.move(-50);
			rc.move(-50);
			delay(100);
			lc.brake();
			rc.brake();
			printf("Cata FIring flag 3: %d \n", CataIdle);
			CataIdle = true; //we are done firing, so we update CataIdle to true
		}
		else //if cata is idle, ie not firing, then we have an opportunity to rewind
		{
			printf("PosL: %i \n", catarot_l.get_angle() / 100);
			printf("PosR: %i \n", catarot_r.get_angle() / 100);

			if(catarot_l.get_angle() / 100 > cata_target || catarot_r.get_angle() / 100 > cata_target) //if we need to rewind
			{
				printf("Cata rewinding flag 74: %d \n", CataIdle);

				bool CataRewinding = true;
				int rewind_target = 355; // position where both gears are almost meshing
				while(CataRewinding)//to rewind the catapult
				{

											printf("Cata rewinding target value: %d \n", rewind_target);
											printf("Cata rewinding left  value: %d \n", catarot_l.get_angle() / 100);

											printf("Cata rewinding right value: %d \n", catarot_r.get_angle() / 100);


					// std::cout << rewind_target << std::endl;
					if (catarot_l.get_angle() / 100 >= rewind_target) {

											printf("Cata rewinding left: %d \n", CataIdle);

						// lc.move(((catarot_l.get_angle() / 100 - rewind_target) * cata_kp));
						// lc.move(-40);
						// pros::delay(2);
						// std::cout << "move_l" << std::endl;
						lc.brake();
					}
					else {
											printf("Cata not rewinding left: %d \n", CataIdle);
						lc.move(-40);
						pros::delay(2);
						// lc.brake();
					}

					if (catarot_r.get_angle() / 100 >= rewind_target) {
											printf("Cata rewinding right: %d \n", CataIdle);

						// rc.move(((catarot_r.get_angle() / 100 - rewind_target) * cata_kp));
						// rc.move(-40);
						// pros::delay(2);
						// std::cout << "move_r" << std::endl;
						rc.brake();
					}
					else {
											printf("Cata not rewinding right: %d \n", CataIdle);
						rc.move(-40);
						pros::delay(2);
						// rc.brake();
					}

					printf("Cata rewinding flag 76: %d \n", CataIdle);

					pros::delay(2);
					if ( (catarot_l.get_angle() / 100) >= rewind_target && (catarot_r.get_angle() / 100) >= rewind_target ){
						mesh = true;
						// std::cout << mesh << std::endl;
						rewind_target = 5;
						while (mesh) {
							std::cout << rewind_target  << std::endl; //+ "   rewind_target"
							if (rewind_target < catarot_l.get_angle() / 100) {
								lc.move(((catarot_l.get_angle() / 100 - 360 - rewind_target) * cata_kp));
								pros::delay(2);
								// std::cout << ((catarot_l.get_angle() / 100 - 360 - rewind_target) * cata_kp) << std::endl;
							}
							else {
								lc.brake();
								pros::delay(2);
							}

							if (rewind_target < catarot_r.get_angle() / 100) {
								rc.move(((catarot_r.get_angle() / 100 - 360 - rewind_target) * cata_kp));
								pros::delay(2);
								// std::cout << ((catarot_r.get_angle() / 100 - 360 - rewind_target) * cata_kp) << std::endl;
							}
							else {
								rc.brake();
								pros::delay(2);
							}
							
							if (rewind_target >= cata_target) {
								lc.brake();
								rc.brake();
								CataRewinding = false;
								CataIdle = true;
								mesh = false;
								pros::delay(2);
							}

							else if ( (catarot_l.get_angle() / 100) >= (rewind_target - 5) && (catarot_r.get_angle() / 100) >= (rewind_target - 5) ) {
								rewind_target += 3;
								pros::delay(2);
							}
						}
					}
					else{
											printf("not fucking meshing: %d \n", CataIdle);

					}
				} 
					//if either motor has not reached its target, move towards the target
					//if either motor is already at the target, then brake immediately to prevent overshooting.
					
					// if(catarot_l.get_angle() / 100 < cata_target)
					// 	lc.move((catarot_l.get_angle() / 100 - cata_target) * cata_kp);
					// else
					// 	lc.brake();

					// if(catarot_r.get_angle() / 100 < cata_target)
					// 	rc.move((catarot_r.get_angle() / 100 - cata_target) * cata_kp);
					// else
					// 	rc.brake();

					// if(catarot_l.get_angle() / 100 >= cata_target && catarot_r.get_angle() / 100 >= cata_target) //if both motors reached the target, we have finished rewinding
					// {
					// 	CataRewinding = false;
					// 	CataIdle = true;
			}
		}
	}
}

// bool IntakeTargetPosUp = true;
int RollerPow = 0;
bool Roller_Intake;
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

	pros::Task cata(cata_control_new);
	pros::Task flipper(flipper_pid);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// pros::Task base_l(base_l_pid);
	// pros::Task base_r(base_r_pid);
	// pros::Task brakes(base_brake);
	//clear controller screen
	// master.clear();
	// pros::delay(50);
	pros::Task base_pid(pidmove);
	pros::delay(2);

	// double targ_l[] = {600, 600};
	// double targ_r[] = {600, 600};
	// arrlen = sizeof(targ_l) / sizeof(targ_l[0]);
	// pros::delay(2);
	// pidvalues(targ_l, targ_r);
	// master.print(2,0,"d",arrlen);

	int a_l = 600;
	int a_r = 600;
	int b_l = 600;
	int b_r = 600;
	pidvalues(a_l, a_r);
	// pidvalues(a_l+b_l, a_r+b_r);
	// pros::delay(400);
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

		if(master.get_digital(DIGITAL_R1) && CataIdle){
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
