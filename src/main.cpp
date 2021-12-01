#include "../include/main.h"
#include  "../include/autonomous.h"



void leftBtn(){

}
void centerBtn(){

}
void rightBtn(){

}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(leftBtn);
	pros::lcd::register_btn1_cb(centerBtn);
	pros::lcd::register_btn2_cb(rightBtn);
	FBarL.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarR.set_brake_mode(MOTOR_BRAKE_HOLD);
	BRLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	BLLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	piston.set_value(true);
	//set to false if default position is the same

  autonSelector();
	//autonSelector();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
  switch(selected){
    case 0:
		disabledAuton();
		break;
    case 1:
	  Robot_inspection();
		break;
    case 2:
		Drive();
		break;
    case 3:
		AWP1();
		break;
    case 4:
		AWP2();
		break;
    case 5:
		TEST_GO_1();
		break;
    case 6:
		TEST_GO_2();
		break;
    case 7:
		TEST_GO_3();
		break;
	case 8:
		skills();
		break;
	default:
		return;
   }
}

const int NUM_HEIGHTS = 3;
const int height1 = 0;
const int height2 = 700;
const int height3 = 1500;


const int heights[NUM_HEIGHTS] = {height1, height2,height3};
const int heights2[NUM_HEIGHTS] = {0, 700,1800};

int x = 0;

void my_task_fn(void* param) {
	std::string t =std::to_string(std::max(FrontLeft.get_temperature(), FrontRight.get_temperature()), BackLeft.get_temperature()), BackRight.get_temperature()),FBarR.get_temperature()), FBarL.get_temperature()), BRLift.get_temperature()), BLLift.get_temperature()) /8);
	control.print(1, 1, t.c_str());
		delay(200);
		// ...
}




void opcontrol() {
	master.clear();
	control.clear();
	BRLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	BLLift.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarL.set_brake_mode(MOTOR_BRAKE_HOLD);
	FBarR.set_brake_mode(MOTOR_BRAKE_HOLD);
  //piston.set_value(true);
  int goalHeight = 0;
	int bGoalHeight = 0;
	double prevr = 0;
	double prevl = 0;
  while (true){
		Task my_task(my_task_fn);
		double power = -control.get_analog(ANALOG_LEFT_Y);
		double turn = -control.get_analog(ANALOG_LEFT_X);
		driverControl(2*power+turn, 2*power - turn);
		if (control.get_digital(E_CONTROLLER_DIGITAL_X)){
			piston.set_value(false);
		}
		if (control.get_digital(E_CONTROLLER_DIGITAL_B)){
			piston.set_value(true);
		}


    if (RUp.changedToPressed() && goalHeight < NUM_HEIGHTS - 1) {
      goalHeight++;
      liftControl->setTarget(heights[goalHeight]);
    } else if (RDown.changedToPressed() && goalHeight > 0) {
      goalHeight--;
      liftControl->setTarget(heights[goalHeight]);
    }
		if (control.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      fourbarmove(120);

    } else if (control.get_digital(E_CONTROLLER_DIGITAL_R2)) {
      fourbarmove(-120);
    } else {
			fourbarmove(0);
		}
    pros::delay(20);
  }

		if(LUp.changedToPressed() && bGoalHeight < NUM_HEIGHTS-1){
			bGoalHeight++;
			liftControl->setTarget(heights[bGoalHeight]);
		} else if (LDown.changedToPressed() && bGoalHeight > 0){
			bGoalHeight--;
			liftControl->setTarget(heights[bGoalHeight]);
		}
		if (control.get_digital(E_CONTROLLER_DIGITAL_L1)){
			bliftmove(135);

		} else if (control.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			bliftmove(-135);

		} else {
			bliftmove(0);
		}
		pros::delay(20);
}
