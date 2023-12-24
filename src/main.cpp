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
int i = 0;
bool step = false;
bool next = false;
bool brake = false;


void pidvalues(double* targleft, double* targright){
	for (i=0; i<5; i++){
		step = false;
		LEFTTARGET = targleft[i];
		RIGHTTARGET = targright[i];
		errorLeft = targleft[i];
		errorRight = targright[i];
		while(not step){
			pros::delay(2);
		}
	}
	next = true;
	i = 0;
}

void pidmove() {
    using namespace pros;
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
	lft_base.set_zero_position(0);
	rft_base.set_zero_position(0);
    //25 units = 1cm
    //and fabs(errorRight) >= 50
    while (true){
        if (fabs(errorLeft) >= 5 ){
            while (fabs(errorLeft) >= 5 ){
				encdleft = trackingwheel_l.get_position() * pi * 28 / 36000;
				encdright = trackingwheel_r.get_position() * pi * 28 / 36000;
                // encdleft = lft_base.get_position() * pi * 69.85 / 360;
                // encdright = rft_base.get_position() * pi * 69.85 / 360;
				// printf("encdleft: %f \n", encdleft);
				// printf("encdright: %f \n", encdright);
				//double left_tw = trackingwheel_l.get_position()/100;
				//printf("encdleft: %f \n", left_tw);
				//double right_tw = trackingwheel_r.get_position()/100;
				//printf("encdright:%f \n",right_tw);

                errorLeft = LEFTTARGET - encdleft;
                errorRight = RIGHTTARGET - encdright;

                deltaErrorLeft = errorLeft - prevErrorLeft;
                deltaErrorRight = errorRight - prevErrorRight;

                powerL = base_kp * (errorLeft/25) + base_kd * deltaErrorLeft; // divide 25 to prevent from going insane - idk why
                powerR = base_kp * (errorRight/25) + base_kd * deltaErrorRight;

                lft_base.move(powerL);
				lfb_base.move(powerL);
				lbt_base.move(powerL);
				lbb_base.move(powerL);
				rft_base.move(powerR);
				rfb_base.move(powerR);
				rbt_base.move(powerR);
				rbb_base.move(powerR);

                //printf("encdleft: %f encdright:%f errorleft:%f errorright:%f deltaerrleft:%f deltaerrright:%f \n",\
                encdleft, encdright, errorLeft, errorRight, deltaErrorLeft, deltaErrorRight);

                prevErrorLeft = errorLeft;
                prevErrorRight = errorRight;
                pros::delay(2);
            }
            // lf_base.brake();
            // lt_base.brake();
            // lb_base.brake();
            // rf_base.brake();
            // rt_base.brake();
            // rb_base.brake();
            //printf("stopped");
			step = true;
			trackingwheel_l.set_position(0);
			trackingwheel_r.set_position(0);
        }
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
bool cata_move = false;
float cata_error_l;
float cata_error_r;
float prev_cata_error;
float cata_d;
uint32_t timestamp;
int correctingPow;
int currentPos_l;
int currentPos_r;

void cata_pid(){
    using namespace pros;
    pros::Motor lc(lc_motor, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rc(rc_motor, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot_l(catarot_l_port);
	pros::Rotation catarot_r(catarot_r_port);
	catarot_r.set_reversed(true);
    while(true){
		if ((loading_pos <= (catarot_l.get_position() / 100) <= fire_pos) && (loading_pos <= (catarot_r.get_position() / 100) <= fire_pos)) {
			currentPos_l = catarot_l.get_position() / 100;
			currentPos_r = catarot_r.get_position() / 100;
			cata_error_l = currentPos_l - cata_target;
			cata_error_r = currentPos_r - cata_target;
			cata_d = cata_error_l - prev_cata_error;
			prev_cata_error = cata_error_l;
			correctingPow = cata_error_l * cata_kp + cata_d * cata_kd + cata_power;
			// printf("CorrectingPow: %i \n", correctingPow);
			// printf("Error: %i \n", cata_error_l);
			// printf("Position_l: %i \n", catarot_l.get_position()/100);
			// printf("Position_r: %i \n", catarot_r.get_position()/100);
			if(shoot){
				lc.move(40);
				rc.move(40);
				pros::delay(500);
				shoot = false;
				pros::delay(Catadelay);
			}
			// resetting of cata
			else if (cata_error_l > allowedError && currentPos_l > cata_target) {
				// check if left and right position is the same
				if (fabs(currentPos_l - currentPos_r) < Cata_lr_error) {
					lc.move(correctingPow);
					rc.move(correctingPow);
					// printf("CorrectingPow: %i \n", correctingPow);
				}
				else if (cata_error_l > cata_error_r) {
					lc.move(correctingPow);
					rc.move(0);
					// printf("right_cata_motor is slower \n");
				}
				else {
					lc.move(0);
					rc.move(correctingPow);
					// printf("left_cata_motor is slower \n");
				}
			}
			else {
				lc.move(0);
				rc.move(0);
				// printf("not moving \n");
			}
		}
		else {
			printf("catarot_l: %i \n", catarot_l.get_position()/100);
			printf("catarot_r: %i \n", catarot_r.get_position()/100);
		}
    }
}

bool IntakeTargetPosUp = true;
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

	// pros::Task cata(cata_pid);
	pros::Task flipper(flipper_pid);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// pros::Task base_l(base_l_pid);
	// pros::Task base_r(base_r_pid);
	// pros::Task brakes(base_brake);
	pros::Task base_pid(pidmove);
	pros::delay(10);

	double targ_l[] = {600, 600};
	double targ_r[] = {600, 600};
	pidvalues(targ_l, targ_r);
	while (not next){
		pros::delay(2);
	}
	next = false;
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

		// if(master.get_digital(DIGITAL_R1)){
        // }

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
