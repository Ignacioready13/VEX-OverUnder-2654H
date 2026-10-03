#include "main.h"
#include "okapi/api.hpp"
#include <string>
using namespace okapi;

/////////////////////////////////MOTOR SETUP//////////////////////////////////////////////
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor motor_1(1, MOTOR_GEAR_BLUE, true);
pros::Motor motor_2(2, MOTOR_GEAR_BLUE, false);
pros::Motor motor_3(3, MOTOR_GEAR_BLUE, false);
pros::Motor motor_4(4, MOTOR_GEAR_BLUE, true);
pros::Motor motor_5(5, MOTOR_GEAR_BLUE, true);
pros::Motor motor_6(6, MOTOR_GEAR_BLUE, false);
pros::Motor leftCataMotor(8, MOTOR_GEAR_BLUE, false);
//left when you lookfromfront.
pros::Motor rightCataMotor(9, MOTOR_GEAR_BLUE, true);

pros::ADIDigitalOut leftWingPiston ('A', false);
pros::ADIDigitalOut rightWingPiston ('B', false);

pros::Imu inertialSensor(21);
////////////////globally used variables//////////////////



bool farSide;


std::shared_ptr<ChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors(
	//left motors
	{1, 3, 5},
	{2, 4, 6} //right motors
	)
    .withDimensions({AbstractMotor::gearset::blue, (84/48.0)}, {{4_in, 12.5_in}, imev5BlueTPR})
    .build();



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_RED);
	pros::lcd::set_text_color(LV_COLOR_WHITE);
	pros::lcd::set_text(1, "initialising robotic plasma rays, air-friers...");
	leftWingPiston.set_value(false);
	rightWingPiston.set_value(false);
	//pros::lcd::register_btn1_cb(on_center_button);
	//inertialSensor.reset();
	leftCataMotor.set_zero_position(leftCataMotor.get_position());
	rightCataMotor.set_zero_position(leftCataMotor.get_position());


	motor_1.set_zero_position(motor_1.get_position());
	motor_2.set_zero_position(motor_2.get_position());
	motor_3.set_zero_position(motor_3.get_position());
	motor_4.set_zero_position(motor_4.get_position());
	motor_5.set_zero_position(motor_5.get_position());
	motor_6.set_zero_position(motor_6.get_position());
	inertialSensor.reset(true);
	//rightCataMotor.tare_position();
	master.clear();
	pros::lcd::set_text(1, "eat drywall");
	
	
}

//Not used.
void disabled() {}

void competition_initialize() {
	//Not used: I prefer to have all the modes pre-flashed instead of selected at runtime
}

const double ticksPerMotorRotation = 300;

//12.6 inches rolled per wheel rotation
//motor spins 1 time to make wheel spin 0.57 times
int inchesToTick(const double inchesToRoll)
{
	//double wheelRotations = (12.6*inchesToRoll);
	//double motorRotations = ((12.6*inchesToRoll) / 0.57);
	return (((inchesToRoll/12.6) / 0.57) * ticksPerMotorRotation);
}

void moveAsymmetrically(double leftSpeedPercent, double rightSpeedPercent, double duration) { 
	//Move the left and right wheel sets at different speeds: used to move in curve shapes.
	//very basic: no PID.
	//intentionally swapped left and right; more intuitive because when I type a high number for right side
	//its intuitive to want the robot to curve more to the right.
	motor_2.move(127 * leftSpeedPercent); //right side
	motor_4.move(127 * leftSpeedPercent);
	motor_6.move(127 * leftSpeedPercent);

	motor_1.move(127 * rightSpeedPercent); //left side
	motor_3.move(127 * rightSpeedPercent);
	motor_5.move(127 * rightSpeedPercent);

	pros::delay(duration * 1000);
	
	motor_2.move(0); //right side
	motor_4.move(0);
	motor_6.move(0);

	motor_1.move(0); //left side
	motor_3.move(0);
	motor_5.move(0);

}



void move(double setInches) { //Moves the robot a set number of inches, using the PID controller.
	int setTick = motor_1.get_position() + inchesToTick(setInches); //accounts for start

	int direction;
	if (setInches < 0) {
		direction = -1;
	} else {
		direction = 1;
	}

	int currentDistance = motor_1.get_position();
	double previousError = 0;
	double error = 0;
	double integral = 0;
	double power = 0;
	double derivative = 0;
	bool pass = true;
	//pid vars
	double kP = 0.21;
	double kI = 0.00;
	double kD = 1.6;
	
	int loopCount = 0; 
	while (pass){
		loopCount++;
		if (power < 40 && loopCount > 133) {
			return;
		}
		currentDistance = motor_1.get_position(); //tick now assuming we started at 0
		error = setTick - currentDistance; //ticks left, both numbers account for start so its good.
		//integral: keeps adding itself up. previous integral + new riemann summ(error*time)
		//max allowable power is 127.
		//call max tick for a normal move 3,000.
		//3000 / ? = 127
		integral = integral + (error); //dont need ki here cause the 20s factors out?
		derivative = error - previousError; //gonna be negative.
		power = (error * kP) + (integral * kI) + (derivative*kD);


		previousError = error;
		pros::lcd::set_text(3, std::to_string(error));
		motor_1.move(power);
		motor_3.move(power);
		motor_5.move(power);

		motor_2.move(power*1.18);
		motor_4.move(power*1.18);
		motor_6.move(power*1.18);
		pros::delay(15);
		if (direction == -1) {
			pass = (currentDistance - 60) > setTick;
		} else {
			pass = (currentDistance + 60) < setTick;
		}

	}
	motor_1.move(0);
	motor_3.move(0);
	motor_5.move(0);
	motor_2.move(0);
	motor_4.move(0);
	motor_6.move(0);
}

void turn(double originalDegreesToAdd) { //rotate robot in place, using PID, until gyroscope says we made it
	inertialSensor.tare_rotation();
	int turnDirection;
	if (originalDegreesToAdd < 0) {
		turnDirection = -1;
	} else {
		turnDirection = 1;
	}
	double setDegreesToAdd = abs(originalDegreesToAdd);

	double startRot = inertialSensor.get_rotation();
	double setRotation = startRot + setDegreesToAdd; //accounts for start
	//int startingTick = motor_1.get_position();
	double currentRotation = startRot;
	double previousError = 0;
	double error = 0;
	double integral = 0;
	double power = 0;
	double derivative = 0;
	//pid vars
	double kP = 1.7;
	double kI = 0.0028;
	double kD = 9;

	//double kP = 1.2;
	//double kI = 0;
	//double kD = 2;
	//bool keepLooping = true;

	int loopCount = 0; //over 133 is over 2 seconds
	while (((currentRotation+(2)) < setRotation) && loopCount < 133) {
		loopCount++;
		currentRotation = abs(inertialSensor.get_rotation()); //tick now assuming we started at 0
		error = setRotation - currentRotation; //ticks left, both numbers account for start so its good. setrot should be positive too.

		//integral: keeps adding itself up. previous integral + new riemann summ(error*time)
		//max allowable power is 127.
		//call max tick for a normal move 3,000.
		//3000 / ? = 127
		integral = integral + (error); //dont need ki here cause the 20s factors out?
		derivative = error - previousError; //gonna be negative.
		power = ((error * kP) + (integral * kI) + (derivative*kD));

		
	
		//if setDeg is positive, power is positive.

		previousError = error;
		pros::lcd::set_text(4, std::to_string(error));
		motor_1.move(power*(1)*turnDirection); //left side
		motor_3.move(power*(1)*turnDirection);
		motor_5.move(power*(1)*turnDirection);

		motor_2.move(power*(-1)*turnDirection); //right side
		motor_4.move(power*(-1)*turnDirection);
		motor_6.move(power*(-1)*turnDirection);

		
		pros::delay(15);

	}
	//loop end, rest of function:
	motor_1.move(0);
	motor_3.move(0);
	motor_5.move(0);
	motor_2.move(0);
	motor_4.move(0);
	motor_6.move(0);
}

//The three arrays that record robot movements. One for the left wheels, one for the right wheels, and one for buttons
int leftVoltages[750];
int rightVoltages[750];
int buttons[750]; //Three options: nothing, intake (1), or catapult-and-outtake (2)



int i = 0;
void recordMovements() {
	if (i <749) {
		i++; //start writing at index 1
		leftVoltages[i] = master.get_analog(ANALOG_LEFT_Y);
		rightVoltages[i] = master.get_analog(ANALOG_RIGHT_Y);
		if (master.get_digital(DIGITAL_L1)) {
      		//Fire catapult + outtake
			buttons[i] = 2;
		}  
		else if (master.get_digital(DIGITAL_R1)){
			//intake
			buttons[i] = 1;
		}
		else {
			//idle
			buttons[i] = 0;	
		}


		
	}
	
}

int b = 0;
void playbackMovements() {
	master.print(0,0,"playing back");
	while (b <749) {
		b++; //start reading at index 1
		

		motor_2.move(rightVoltages[b]); //first write: rightVoltages[1]
		motor_4.move(rightVoltages[b]); //last write: rightVoltages[749] (b=748, then increment)
		motor_6.move(rightVoltages[b]);

		motor_1.move(leftVoltages[b]);
		motor_3.move(leftVoltages[b]);
		motor_5.move(leftVoltages[b]);
		
		if (buttons[b] == 2) {
			std::cout<<buttons[b]<<std::endl;
			leftCataMotor.move(127);
			rightCataMotor.move(127);
		}
		else if (buttons[b] == 1) {
			std::cout<<buttons[b]<<std::endl;
			leftCataMotor.move(-127);
			rightCataMotor.move(-127);
		}
		else {
			std::cout<<buttons[b]<<std::endl;
			leftCataMotor.move(0);
			rightCataMotor.move(0);
		}
		

		pros::delay(20);
	}

}

/////////////////////////KEY/////////////////////////
/////////farside - FILENNUM = 1
//1.txt - close side, left side
//2.txt - close side, right side
//3.txt - close side, intake outtake

void saveStuff(int fileNum) {
	//based on SD card reading and writing documentation found at https://pros.cs.purdue.edu/v5/tutorials/topical/filesystem.html
	const char* prefix = "/usd/";
	const char* suffix = ".txt";
	std::string stringRepresentingArray = "";

	//left side
	char LeftName[16]; //big enough to store file name
	sprintf(LeftName, "%s%d%s", prefix, fileNum, suffix);
	FILE* usd_file_write = fopen(LeftName, "w");	
	for (int i = 0; i < 750; i++) {
		stringRepresentingArray = stringRepresentingArray + (std::to_string(leftVoltages[i])) + ",";
	}
	fputs(stringRepresentingArray.c_str(), usd_file_write);
	fclose(usd_file_write);
	stringRepresentingArray = "";

	//right side
	char RightName[16];
	sprintf(RightName, "%s%d%s", prefix, (fileNum + 1), suffix);
	FILE* usd_file_write2 = fopen(RightName, "w");	
	for (int i = 0; i < 750; i++) {
		stringRepresentingArray = stringRepresentingArray + (std::to_string(rightVoltages[i])) + ",";
	}
	fputs(stringRepresentingArray.c_str(), usd_file_write2);
	fclose(usd_file_write2);
	stringRepresentingArray = "";

	//intake outtake
	char CataName[16];
	sprintf(CataName, "%s%d%s", prefix, (fileNum + 2), suffix);
	FILE* usd_file_write3 = fopen(CataName, "w");	
	for (int i = 0; i < 750; i++) {
		std::cout<<std::to_string(buttons[i])<<std::endl;
		stringRepresentingArray = stringRepresentingArray + (std::to_string(buttons[i])) + ",";
	}
	stringRepresentingArray = stringRepresentingArray + "8";
	fputs(stringRepresentingArray.c_str(), usd_file_write3);
	fclose(usd_file_write3);
	stringRepresentingArray = "";


	
}

void readStuff(int fileNum) {
	std::string TempStringRepresentingAnInt = "";
	const char* prefix = "/usd/";
	const char* suffix = ".txt";

	char LeftName[16];
	sprintf(LeftName, "%s%d%s", prefix, fileNum, suffix);
	FILE* usd_file_read = fopen(LeftName, "r");
	char buf[3760] = {0};
	fread(buf, 1, 3750, usd_file_read);
	int positionInArrayBeingWrittenTo = 0;
	//go thru and convert txt file to left array
	for (int v = 0; buf[v] != '\0'; ++v) {
		if (buf[v] == ',') {
			leftVoltages[positionInArrayBeingWrittenTo] = std::stoi(TempStringRepresentingAnInt);
			positionInArrayBeingWrittenTo = positionInArrayBeingWrittenTo + 1;
			TempStringRepresentingAnInt = "";
		}
		else {
			TempStringRepresentingAnInt.push_back(buf[v]);
		}
		
	}
	fclose(usd_file_read);
	positionInArrayBeingWrittenTo = 0;
	TempStringRepresentingAnInt = "";


	//right side
	char RightName[16];
	sprintf(RightName, "%s%d%s", prefix, (fileNum + 1), suffix);
	FILE* usd_file_read2 = fopen(RightName, "r");
	char buf2[3760] = {0}; //fill with zeroes
	fread(buf2, 1, 3750, usd_file_read2);
	//go thru and convert txt file to left array
	for (int v = 0; buf2[v] != '\0'; ++v) {
		if (buf2[v] == ',') {
			rightVoltages[positionInArrayBeingWrittenTo] = std::stoi(TempStringRepresentingAnInt);
			positionInArrayBeingWrittenTo = positionInArrayBeingWrittenTo + 1;
			TempStringRepresentingAnInt = "";
		}
		else {
			TempStringRepresentingAnInt.push_back(buf2[v]);
		}
		
	}
	fclose(usd_file_read2);
	positionInArrayBeingWrittenTo = 0;
	TempStringRepresentingAnInt = "";



	//rcat
	char catName[16];
	sprintf(catName, "%s%d%s", prefix, (fileNum + 2), suffix);
	FILE* usd_file_read3 = fopen(catName, "r");
	//std::cout<<positionInArrayBeingWrittenTo<<std::endl;
	char buf3[1510] = {0}; //some extra space
	fread(buf3, 1, 1501, usd_file_read3);
	int r = 0;
	//go thru and convert txt file to left array
	for (int v = 0; buf3[v] != '8'; ++v) { //I used the number 8 as a stop marker

	
		if (buf3[v] == ',') {
		
			buttons[positionInArrayBeingWrittenTo] = (buf3[v-1] - '0');
		
			positionInArrayBeingWrittenTo = positionInArrayBeingWrittenTo + 1;
			
		}
	
	}
	//std::cout<<positionInArrayBeingWrittenTo<<std::endl;

	fclose(usd_file_read3);
    
}




/**
runs autonomous code, either when Field Controller triggers it, or when I call it manually.
 */
int readSlot = 1;
void autonomous() {
	readStuff(1);
	
	playbackMovements(); //As written, code is in playback mode. To run the manual auton, I comment out these two lines
	pros::delay(5252342424); //I upload each mode to a different save slot on robot. So playback mode never needs the manual auton.


//___________________________________________
/////////////////////////////////>
	bool skills = false; ///////////////>
/////////////////////////////////>
//____________________________________________

	if (skills) { ////////////////////////////////////////////////////SKILLS////////////////////////////////////////////	
		move(28);
		turn(84);
	 	moveAsymmetrically(-0.5, -0.5, 1.5);
		//rightWingPiston.set_value(true);
		leftCataMotor.move(127);
		rightCataMotor.move(127);
		pros::delay(30000);
		leftCataMotor.move(0);
		rightCataMotor.move(0);
		
		turn(10);
		move(9);
		turn(-125);
	 	moveAsymmetrically(-0.55, -0.6, 0.5);
		
	
		leftCataMotor.move(67);
		rightCataMotor.move(67);
	 	moveAsymmetrically(-0.53, -0.75, 0.2);
		moveAsymmetrically(-1, -0.73, 0.1);
		


	 	moveAsymmetrically(-0.6, -0.6, 1);
		leftCataMotor.move(0);
		rightCataMotor.move(0);
	 	moveAsymmetrically(-0.5, -0.88, 0.5);
	 	moveAsymmetrically(-0.4, -0.85, 0.3);
	 	moveAsymmetrically(-0.52, -0.63, 1);
		pros::delay(500);
		move(5);
		turn(93);
		
		turn((270 - inertialSensor.get_heading()) + 7);
	 	moveAsymmetrically(0.6, 0.6, 0.8);
		pros::delay(250);
		turn(-70);
		leftWingPiston.set_value(true);
		rightWingPiston.set_value(true);
	 	moveAsymmetrically(-1, -0.58, 1.5);
		leftWingPiston.set_value(false);
		rightWingPiston.set_value(false);
		move(30);
		turn(-93);
		move(-32);
		turn(93);
		leftWingPiston.set_value(true);
		rightWingPiston.set_value(true);
	 	moveAsymmetrically(-1, -1, 1);



		pros::delay(243242);
	}

//___________________________________________
/////////////////////////////////>
	farSide = false;  ///////////////>
/////////////////////////////////>
//____________________________________________

	if (farSide) {   //////////////////////////////////////////////FAR SIDE///////////////////////////////////////

		moveAsymmetrically(-0.54, -1, 0.45);
		
		leftWingPiston.set_value(true);
		rightWingPiston.set_value(true);
	 	moveAsymmetrically(-0.95, -0.6, 0.45);
	 	moveAsymmetrically(-0.6, -0.6, 0.28);

	 	moveAsymmetrically(1, 1, 0.1);//stop

		
		turn(abs(inertialSensor.get_heading()));

		leftWingPiston.set_value(false);
		rightWingPiston.set_value(false);
		pros::delay(500);
		
		move(5);
		turn(-93);

		leftCataMotor.move(127);
		rightCataMotor.move(127);
	 	moveAsymmetrically(1, 1, 0.9);
		leftCataMotor.move(0);
		rightCataMotor.move(0);
		pros::delay(500);
		
		move(-12);
		turn(93);
		move(20.5);
		turn(93);
		leftCataMotor.move(-127);
		rightCataMotor.move(-127);
	 	moveAsymmetrically(0.5, 0.5, 1);
	 	moveAsymmetrically(-0.9, -0.2, 0.5);
		pros::delay(1000);
	 	moveAsymmetrically(0.4, 1, 0.9);



		pros::delay(2432432);




		turn(90);
		move(-23);
		
		turn(20);
		rightWingPiston.set_value(true);





	}
	else { ///////////////////////////////////////////////////////////CLOSE SIDE//////////////////////////////////////////

		chassis->setMaxVelocity(100);
		pros::lcd::set_text(2, "EZ side");
		//chassis->turnAngle(360_deg);
		//EZ SIDE CODE


		leftCataMotor.move(-127);
		rightCataMotor.move(-127);
	 	moveAsymmetrically(0.5, 0.7, 0.6);
	 	moveAsymmetrically(0.75, 0.6, 0.3);
	 	moveAsymmetrically(0.5, 0.1, 0.3);
	 	moveAsymmetrically(-1, -1, 0.1);
	 	moveAsymmetrically(0.3, 0.3, 0.5);

		pros::delay(200);
		// intake

		//turn(20);
		move(-10);
		turn(60);
		move(-44);


		//leftCataMotor.move(0);
		//rightCataMotor.move(0);
		turn(-89);
		//move(3);
		
		leftWingPiston.set_value(true);
		//left wing out
		leftCataMotor.move(0);
		rightCataMotor.move(0);
		pros::delay(550);
		leftCataMotor.move(-127);
		rightCataMotor.move(-127);
	 	moveAsymmetrically(-0.5, -0.5, 0.35);
		leftWingPiston.set_value(false);
	 	moveAsymmetrically(-0.5, -0.5, 0.2);
		turn(-73);
		move(-35);
		rightWingPiston.set_value(true);
		pros::delay(500);
	 	moveAsymmetrically(-0.5, 0.5, 1);
		

		
	}
	
	
}



void reloadCatapult() {
	//300 tics per motor rotation
	//4 motor rots per cata cycle
	//300*4=900 tics per cata cycle
	pros::lcd::set_text(2, "re-load");
	int ticsPerCata = 1500;
	int unsettledPos = ((int)leftCataMotor.get_position()) % ticsPerCata;

	int missingTics = ticsPerCata - unsettledPos;
	
	leftCataMotor.move_absolute((leftCataMotor.get_position() + missingTics), 600);
	rightCataMotor.move_absolute((rightCataMotor.get_position() + missingTics), 600);
	//while ((leftCataMotor.get_position() > (leftCataMotor.get_position() + missingTics) - 5) && (leftCataMotor.get_position() < (leftCataMotor.get_position() + missingTics) + 5)) {
	//	pros::delay(200);
	//}
	pros::delay(1500);
	pros::lcd::set_text(4, std::to_string((int)ticsPerCata - unsettledPos));
}




//bool previouslyCataWorking = false;
void checkCatapultFire() {
	if (master.get_digital(DIGITAL_L1)) {
      	//Fire catapult. 
		//outtake too
		//previouslyCataWorking = true;
		leftCataMotor.move(127);
		rightCataMotor.move(127);
	}  
	else if (master.get_digital(DIGITAL_R1)){
		//previouslyCataWorking = true;
		//intake.
		leftCataMotor.move(-127);
		rightCataMotor.move(-127);
	}
	else {
		//pros::lcd::set_text(3, "no button");
		
		leftCataMotor.move(0);
		rightCataMotor.move(0);
		//previouslyCataWorking = false;
		
	}

}


bool previousLWButtonState = false;
bool leftWingState = false;
void checkLeftWingExtend() {

	bool currentLWButtonState = master.get_digital(DIGITAL_L2);
	if (currentLWButtonState == true && previousLWButtonState == false) {

		if (leftWingState == false) {
			leftWingState = true;
			leftWingPiston.set_value(true);
		} 
		else
		{
			leftWingState = false;
			leftWingPiston.set_value(false);
		}
	}
	previousLWButtonState = currentLWButtonState;
}


bool previousRWButtonState = false;
bool rightWingState = false;
void checkRightWingExtend() {
	bool currentRWButtonState = master.get_digital(DIGITAL_R2);
	if (currentRWButtonState == true && previousRWButtonState == false) {
		if (rightWingState == false) {
			rightWingState = true;
			rightWingPiston.set_value(true);
		} 
		else
		{
			rightWingState = false;
			rightWingPiston.set_value(false);
		}
	}
	previousRWButtonState = currentRWButtonState;
}


void driveTrain() {
	motor_2.move(master.get_analog(ANALOG_RIGHT_Y));
	motor_4.move(master.get_analog(ANALOG_RIGHT_Y));
	motor_6.move(master.get_analog(ANALOG_RIGHT_Y));

	motor_1.move(master.get_analog(ANALOG_LEFT_Y));
	motor_3.move(master.get_analog(ANALOG_LEFT_Y));
	motor_5.move(master.get_analog(ANALOG_LEFT_Y));

}

//description from PROS:
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
bool autonRecordingMode = true;

int tempCount = 0;
bool saveWasHandled = false;

//closeside:1
//farside: 3
//////////////////
int saveSlot = 1;
/////////////////

void opcontrol() {

	while (true) {
		if (autonRecordingMode) {
			//pros::delay(5000);
			//master.print(0,0,"starting...");
			if (tempCount < 750) {
				tempCount = tempCount + 1;
				recordMovements();
				driveTrain();
				if ((tempCount*20)%1000 == 0) {
					master.print(0,0,"recording: %i", (int)(tempCount));
				}
			} 
			else {
				//master.print(0,0,"re-playing---------");
				if (saveWasHandled == false) {
					saveWasHandled = true;
					pros::delay(500);
					master.print(0,0,"recorded. hold A to save.");
					pros::delay(3000);
					if (master.get_digital(DIGITAL_A)) {
						saveStuff(saveSlot);
						master.clear();
						pros::delay(250);
						master.print(0,0,"SAVED!");
					}
					//readStuff(13);
					//saveStuff(14);
					//master.print(0,0,"re-playing---------");
				}
				//playbackMovements();
			}
		
		}
		else {
			driveTrain();
		}
		
	
		checkCatapultFire();
		checkLeftWingExtend();
		checkRightWingExtend();
		

		
		
		pros::delay(20); //20
	}
}