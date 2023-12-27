#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

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
	errorLeft = targleft;
	errorRight = targright;
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

bool shoot = false;
bool move_cata = true;
double cata_error_l = 0;
double cata_error_r = 0;
double prev_cata_error_l = 0;
double prev_cata_error_r = 0;
double cata_d_l = 0;
double cata_d_r = 0;
static int correctingPow_l;
static int correctingPow_r;
double currentPos_l;
double currentPos_r;

void cata_pid(){
    using namespace pros;
    pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);
    while(true){
		pros::delay(2);
		if (not shoot) {std::cout << "shoot = false" << std::endl;}
		// if ((loading_pos <= (catarot_l.get_angle() / 100) <= fire_pos) && (loading_pos <= (catarot_r.get_angle() / 100) <= fire_pos)) {
		currentPos_l = catarot_l.get_angle() / 100;
		currentPos_r = catarot_r.get_angle() / 100;
		cata_error_l = currentPos_l - cata_target;
		cata_error_r = currentPos_r - cata_target;
		cata_d_l = cata_error_l - prev_cata_error_l;
		cata_d_r = cata_error_r - prev_cata_error_r;
		correctingPow_l = cata_error_l * cata_kp + cata_d_l * cata_kd;
		correctingPow_r = cata_error_r * cata_kp + cata_d_r * cata_kd;
		prev_cata_error_l = cata_error_l;
		prev_cata_error_r = cata_error_r;
		// printf("CorrectingPow: %i \n", correctingPow);
		// printf("Error: %i \n", cata_error_l);
		// printf("Position_l: %i \n", catarot_l.get_angle()/100);
		printf("Position_r: %i \n", catarot_r.get_angle()/100);
		// printf("CorrectingPow_l: %i \n", correctingPow_l);
		if(shoot){
			// if ((catarot_l.get_angle() / 100) < 40 && (catarot_r.get_angle() / 100) < 40) {
				lc.move(-50);
				rc.move(-50);
				pros::delay(100);
			// }
			// else {
				lc.move(0);
				rc.move(0);
				shoot = false;
				pros::delay(Catadelay);
				// cata_error_l = 0;
				// cata_d_l = 0;
				// prev_cata_error_l = 0;
				// cata_error_r = 0;
				// cata_d_r = 0;
				// prev_cata_error_r = 0;
				pros::delay(2);
			}
		}


			// resetting of cata
		// 	else if (cata_error_l > allowedError && currentPos_l > cata_target) {
		// 		// check if left and right position is the same
		// 		if (fabs(currentPos_l - currentPos_r) < Cata_lr_error) {
		// 			lc.move(correctingPow);
		// 			rc.move(correctingPow);
		// 			// printf("CorrectingPow: %i \n", correctingPow);
		// 		}
		// 		else if (cata_error_l > cata_error_r) {
		// 			lc.move(correctingPow);
		// 			rc.move(0);
		// 			// printf("right_cata_motor is slower \n");
		// 		}
		// 		else {
		// 			lc.move(0);
		// 			rc.move(correctingPow);
		// 			// printf("left_cata_motor is slower \n");
		// 		}
		// 	}
		// 	else {
		// 		lc.move(0);
		// 		rc.move(0);
		// 		// printf("not moving \n");
		// 	}
		// }
		// else {
		// 	printf("catarot_l: %i \n", catarot_l.get_position()/100);
		// 	printf("catarot_r: %i \n", catarot_r.get_position()/100);
		// }
    }
// }

void cata_l() {
	pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	while (true) {
		if ((not shoot) && move_cata){
			if (cata_error_l > allowedError) {
				lc.move(-correctingPow_l);
				printf("correctingPow_l: %i \n", correctingPow_l);
			}
			else {
				lc.move(0);
			}
			pros::delay(2);
		}
		pros::delay(2);
	}
}
void cata_r() {
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
	while (true) {
		if ((not shoot) && move_cata) {
			if (cata_error_r > allowedError) {
				rc.move(-correctingPow_r);
				printf("correctingPow_r: %i \n", correctingPow_r);
			}
			else {
				rc.move(0);
			}
			pros::delay(2);
		}
		pros::delay(2);
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

	//flipper
    pros::Motor f_arm(flipper_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor f_roller(flipper_roller_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation flipperrot(flipperrot_port);

	//front rollers
    pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Task cata(cata_pid);
	pros::Task cataR(cata_r);
	pros::Task cataL(cata_l);
	
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
