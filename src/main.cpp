#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "globals.hpp"


bool l_brake = false;
bool r_brake = false;

void left_brake() {
	pros::Motor lfb_base(lfb_port);
	pros::Motor lft_base(lft_port);
	pros::Motor lbb_base(lbb_port);
	pros::Motor lbt_base(lbt_port);
	while(true){
		while(l_brake){
			lft_base.brake();
			lfb_base.brake();
			lbt_base.brake();
			lbb_base.brake();
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
			rft_base.brake();
			rfb_base.brake();
			rbt_base.brake();
			rbb_base.brake();
		}
	}
}

// ratio of turn dist = turn_radius : turn_radius+base
// void turn() {
// }

bool shoot = false;
double cata_currentPos = 0;
int cata_target = 80;
bool arm_up = true;
bool half_draw = false;

void cata_pid(){
    using namespace pros;
    pros::Motor fc(fc_motor);
    pros::Motor bc(bc_motor);
    pros::Rotation catarot(catarot_port);

	double cata_error;
	double prev_cata_error;
	double total_cata_error;
	double cata_d;
	double correctingPow;

    while(true){
		if ( (!arm_up) && half_draw) {
			cata_target = 45;
		}
		else {
			cata_target = 80;
		}

        cata_currentPos = catarot.get_position() / 100;
		if (cata_currentPos > 180) {
			cata_currentPos -= 360;
		}
        cata_error = cata_target - cata_currentPos;
        cata_d = cata_error - prev_cata_error;
		total_cata_error += cata_error;
        correctingPow = cata_error * cata_kp + cata_d * cata_kd + cata_power;
		prev_cata_error = cata_error;
		// std::cout << "correctingPow: " << correctingPow << std::endl;
		// std::cout << "cata_error: " << cata_error << std::endl;
		// std::cout << "currentPos: " << currentPos << std::endl;
		// printf("CorrectingPow: %i \n", correctingPow);
        // printf("Error: %i \n", cata_error);
        // printf("Position: %i \n", catarot.get_position()/100);
        if(shoot){
            fc.move(127);
            bc.move(127);
            pros::delay(200);
			shoot = false;
			// fc.move(127);
			// bc.move(127);
			// pros::delay(2);
			// cata_currentPos = catarot.get_position() / 100;
			// if (cata_currentPos < 70) {
			// 	shoot = false;
			// 	pros::delay(Catadelay);
			// }
			// else {
			// 	shoot = true;
			// 	std::cout << "retry firing" << std::endl;
			// 	pros::delay(2);
			// }
        }
        else if (cata_currentPos <= cata_target) {
            if ( (cata_target - cata_currentPos) >= 1) {
				shoot = false;
				fc.move_velocity(correctingPow);
				bc.move_velocity(correctingPow);
				// std::cout << "rewinding" << std::endl;
				std::cout << "correctingPow: " << correctingPow << std::endl;
				std::cout << "cata_error: " << cata_error << std::endl;
				std::cout << "currentPos: " << cata_currentPos << std::endl;
				std::cout << "actual velocity: " << fc.get_actual_velocity() << std::endl;
				std::cout << "motor efficiency: " << fc.get_efficiency() << std::endl;
				// if ( fabs(fc.get_actual_velocity()) < 3 ) {
				// 	// std::cout << "cata jammed" << std::endl;
				// }
			}
        }
        else {
            fc.move(0);
            bc.move(0);
            std::cout << "done" << std::endl;
        }
		pros::delay(2);
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

	int currentPos;

    // IntakeTargetPosUp = true; //move to up position    
    // IntakeTargetPosUp = false; //move to down position

    // RollerPow = 127;//roller outtake    
    // RollerPow = -127; //roller intake
    //RollerPow = 0; //roller stop
    while (true) {
        currentPos = flipperrot.get_position() / 100;
		if (fabs(currentPos - flipper_targetUp) <= 3) {
			arm_up = true;
		}
		else {
			arm_up = false;
		}
        if (IntakeTargetPosUp) {
            flipper_error = flipper_targetUp - currentPos;
		}
        else {
            flipper_error = flipper_targetDown - currentPos;
		}

        flipper_d = flipper_error - prev_flipper_error;
        total_flipper_error += flipper_error;
        flipper_pow = flipper_error * flipper_kp + total_flipper_error * flipper_ki + flipper_d * flipper_kd;
        f_arm.move(flipper_pow);
        f_roller.move(RollerPow);
        prev_flipper_error = flipper_error;
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
	lft_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lfb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lbb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lbt_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rft_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rfb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rbb_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rbt_base.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::Rotation trackingwheel_l(twl_port);
	pros::Rotation trackingwheel_r(twr_port);
	trackingwheel_r.set_reversed(true);

	// pros::Imu imu_sensor(imu_port);
	// imu_sensor.reset();

	//cata
	pros::Motor fc(fc_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor bc(bc_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation catarot(catarot_port);

	//flipper
    pros::Motor f_arm(flipper_motor, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor f_roller(flipper_roller_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Rotation flipperrot(flipperrot_port);

	//front rollers
    pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Task cata(cata_pid);
	pros::Task flipper(flipper_pid);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	pros::Motor front_roller(front_roller_motor);

	front_roller.move(-127);
	forward_pid(950, 950);
	turn_pid(45, false);


	// turn_pid(20, false);
	// forward_pid(1000, 1000);
	// forward_pid()
	// turn_pid(40, false);
	// forward_pid(1100, 1100);
	
	// front_roller.move(-127);
	// forward_pid(1030, 1030);
	// // pros::delay(100);
	// turn_pid(70, true);
	// // shoot = true;
	// // pros::delay(100);
	// forward_pid(400, 400);
	// pros::delay(500);	
	
	// front_roller.move(0);
	
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

	//front rollers
    pros::Motor front_roller(front_roller_motor, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

	bool tankdrive = true; //drive mode control
	double left, right; //base control
	double power, turn;

	arm_up = true;

	while(true){
		if(master.get_digital_new_press(DIGITAL_A)) tankdrive = !tankdrive; //tank toggle

        if(tankdrive) {
            left = master.get_analog(ANALOG_LEFT_Y);
            right = master.get_analog(ANALOG_RIGHT_Y);
        }     
        else {
            power =  master.get_analog(ANALOG_LEFT_Y);
            turn = master.get_analog(ANALOG_RIGHT_X);
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

		if(master.get_digital_new_press(DIGITAL_R1)){
			shoot = true;
        }
		if(master.get_digital_new_press(DIGITAL_LEFT)) {
			shoot = true;
		}

		//flipper control
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
			// pros::delay(12000);

			half_draw = true;
			uint32_t cycle_start_time = pros::millis();
			for (int i = 1; i <= 2; i++){
				IntakeTargetPosUp = false;
				pros::delay(400); // 400
				shoot = true;
				RollerPow = 127;
				pros::delay(250); // 400
				RollerPow = 0;
				pros::delay(150);
				IntakeTargetPosUp = true;
				pros::delay(400);
				RollerPow = 127;
				pros::delay(40);
				RollerPow = 0;
				pros::delay(500); // 300
				arm_up = true;
				// pros::delay(800);
			}
			pros::delay(400); // 400
			shoot = true;
			pros::delay(200);
			std::cout << "cycle time: " << (pros::millis() - cycle_start_time) << std::endl;
			half_draw = false;


			// turn_pid(25, false);
			// front_roller.move(-127);
			// forward_pid(1000, 1000);
			// // turn_pid();	
			
			// front_roller.move(0);
		}
		
		//side rollers control
        front_roller.move(100 * (master.get_digital(DIGITAL_L2) - master.get_digital(DIGITAL_L1)));

		pros::delay(5);
	}
}