#include <Wire.h>
#include <LiquidCrystal.h>
#include "Arduino.h"
#include <math.h>
#include "NewPing.h"
#include "SharpIR.h"
#include "QTRSensors.h"




//Line sensor defines
#define Kp_line 1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd_line 8 
#define Ki_line 0// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot
#define rightBaseSpeed 100 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 110  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   3     // emitter is controlled by digital pin 2

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 4
#define motorPower 8



//Encoder pins
#define encRtA T4 //on PH7
#define encLtA T5 //on PL2
#define PWM_lt 4
#define PWM_rt 5
#define EN_lt  7
#define EN_rt  6

//LCD pins
#define LCD_RS 24
#define LCD_EN 25
#define LCD_D0 18
#define LCD_D1 19
#define LCD_D2 22
#define LCD_D3 23

//GLOBAL VARIABLES
#define DEBUG false
#define LOOPTIME 100
#define WHEEL_SIZE 2.36



#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//IR
#define IR_Left A9
#define IR_Right A6
SharpIR sharp_l(IR_Left, 25, 93, 1080);
SharpIR sharp_r(IR_Right, 25, 93, 1080);
int Sensor[4];

//Sonar variables
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

int last_proximity = 0;
const int base_speed_lt = 120;
const int base_speed_rt = 110;
const int set_point = 15;

double P_adjust = 0;
double I_adjust = 0;
double D_adjust = 0;
int PIorD = 0;


//Sensor 0 = front
//Sensor 1 = Right
//Sensor 2 = left

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
	NewPing(A13, A12, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
	NewPing(A11, A10, MAX_DISTANCE),
	NewPing(13, 12, MAX_DISTANCE),

};


boolean Maze = false;

//Line following stuff
unsigned char QTIPins[] = { A1, A2, A3, A4, A5 };
QTRSensorsRC qtrrc(QTIPins, 3, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
int lastError = 0;
int I = 0;
int position = -1;
boolean LineFollow = false;


//LCD Setup
LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D0, LCD_D1, LCD_D2, LCD_D3);

//Button input
// Right  = 1
// Up     = 2
// Down   = 3
// Left   = 4
// Select = 5
volatile int button = 0;

enum Function {
	 DRIVE,
	 LINE,
	 TURN,
	 STOP,
	 CALIBRATE,
	 COURSE,
	 MAZE
} FUNCTION;
/*
enum Direction {
	LEFT,
	RIGHT,
	FWD,
	REV,
	BRAKE,
	HARD_LEFT
}direction_t;
*/
Function STATE;
int StateIncrementer = 0;
enum Direction {
	LEFT,
	RIGHT
} DIRECTION;

int CourseCounter = 0;

volatile boolean RobotStopped = true;



//Motor/Encoder variables
int target_rt = 119;
int target_lt = 122;
int RPM_rt = 0;
int RPM_lt = 0;
static int last_error_rt = 0;
static int last_error_lt = 0;

unsigned long lastMilli = 0;
volatile long speed_count_rt = 0;
volatile long speed_count_lt = 0;

volatile long distance_count_rt = 0;
volatile long distance_count_lt = 0;
volatile long desired_distance = 0;
volatile long count = 0;


volatile long lastCount_rt = 0;
volatile long lastCount_lt = 0;
volatile long count_correction = 1000; // take into account that the wheels don't stop when PWM's go to  0

int PWM_val_rt = 0;
int PWM_val_lt = 0;

float Kp = .45;          //setting Kp  
float Ki = 0; // .55 //.98
float Kd = 1.8; // 1.5;       //.58 //.98    //setting Kd

//End motor variables

void pinInit() {
	pinMode(PWM_rt, OUTPUT);
	pinMode(PWM_lt, OUTPUT);
	pinMode(EN_lt, OUTPUT);
	pinMode(EN_rt, OUTPUT);

	analogWrite(PWM_rt, 0);
	analogWrite(PWM_lt, 0);
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 1);
	pinMode(A11, OUTPUT);
	pinMode(A10, INPUT);

}

void LCDInit() {
	LCD.begin(16, 2);
	LCD.setCursor(0, 0);
	LCD.print("Mode: ");
	LCD.print(FUNCTION);
}

void TCInit() {

	TIMSK4 &= ~(1 << TOIE4);  //disable Timer4 interrupt
	TIMSK5 &= ~(1 << TOIE5);  //disable Timer5 interrupt

	TCCR4A = 0;        // set entire TCCR4A register to 0
	TCCR4B = 0;

	TCCR5A = 0;        // set entire TCCR5A register to 0
	TCCR5B = 0;
							  //Set timer4 mode 0  
	TCCR4A &= ~(1 << WGM40);
	TCCR4A &= ~(1 << WGM41);
	TCCR4B &= ~(1 << WGM43);
	TCCR4B &= ~(1 << WGM42);

	//Set timer4 external clock, triggered on rising edge (T1 pin = Arduino pin 5)  
	TCCR4B |= (1 << CS42);
	TCCR4B |= (1 << CS41);
	TCCR4B &= ~(1 << CS40);

	//Set timer5 mode 0  
	TCCR5A &= ~(1 << WGM50);
	TCCR5A &= ~(1 << WGM51);
	TCCR5B &= ~(1 << WGM53);
	TCCR5B &= ~(1 << WGM52);

	//Set timer5 external clock, triggered on rising edge (T1 pin = Arduino pin 5)  
	TCCR5B |= (1 << CS52);
	TCCR5B |= (1 << CS51);
	TCCR5B |= (1 << CS50);

	//TIMSK4 |= (1 << TOIE4);  //enable Timer4 interrupt
	//TIMSK5 |= (1 << TOIE5);  //enable Timer5 interrupt
}

void ReadButtons() {
	int x;
	x = analogRead(0);
	LCD.setCursor(7, 0);
	if (x < 60) {
		button = 1;
		LCD.print("Right ");
		if (STATE == MAZE && PIorD != 0) {
			if (STATE == MAZE && PIorD == 1) P_adjust = P_adjust + .1;
			if (STATE == MAZE && PIorD == 2) I_adjust = I_adjust + .1;
			if (STATE == MAZE && PIorD == 3) D_adjust = D_adjust + .1;
		}
		else 		StateIncrementer++;
	}
	else if (x < 200) {
		button = 2;
		if (STATE == MAZE) {
			PIorD++;
			if (PIorD > 3) PIorD = 0;
		}
	//	STATE = DRIVE;
	}
	else if (x < 400) {
		button = 3;
		if (STATE == MAZE) {
			PIorD--;
			if (PIorD < 0) PIorD = 3;
		}
	//	LCD.print("Line");
	//	STATE = LINE;
	}
	else if (x < 600) {
		button = 4;
	//	LCD.print("Left  ");
		if (STATE == MAZE && PIorD != 0) {
			if (STATE == MAZE && PIorD == 1) P_adjust = P_adjust - .1;
			if (STATE == MAZE && PIorD == 2) I_adjust = I_adjust - .1;
			if (STATE == MAZE && PIorD == 3) D_adjust = D_adjust - .1;
		}
		else 		{
		if (StateIncrementer != 0) StateIncrementer--;
		else StateIncrementer = 5;
		}

	}
	else if (x < 800) {
		button = 5;
		FUNCTION = STATE;
		distance_count_lt = 0;
		distance_count_rt = 0;
		//LCD.print("Select");
	}
}

void ChangeAngle(int angle) {
	int steps = CountsPerDistance(  angle * (28.26 / 360) );
	desired_distance = steps;
	OCR4A = steps;
	OCR5A = steps;
	TIMSK4 |=  (1 << TOIE4);  //disable Timer4 interrupt
	TIMSK5 |=  (1 << TOIE5);  //disable Timer5 interrupt
	FUNCTION = TURN;
}

void setup() {
	Serial.begin(9600);
	FUNCTION = STOP;
	DIRECTION = LEFT;
	
	pinInit();
	LCDInit();
	TCInit();
	pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
	for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
		pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;


}

void Brake() {
	//This is applying "brakes"
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_rt, 20);
	analogWrite(PWM_lt, 30);
	RobotStopped = true;
}

/*
void rbt_move(direction_t new_dir, uint8_t speed)
{
	if (speed)
	{
		switch (new_dir) {
		case LEFT:
			digitalWrite(EN_lt, LOW);
			digitalWrite(EN_rt, HIGH);
			analogWrite(PWM_lt, speed);
			analogWrite(PWM_rt, speed - speed / 2);
			break;
		case RIGHT:
			digitalWrite(EN_lt, LOW);
			digitalWrite(EN_rt, HIGH);
			analogWrite(PWM_rt, speed);
			analogWrite(PWM_lt, speed - speed / 2);
			break;
		case FWD:
			digitalWrite(EN_lt, LOW);
			digitalWrite(EN_rt, HIGH);
			analogWrite(PWM_rt, speed);
			analogWrite(PWM_lt, speed);
			break;
		case REV:
			digitalWrite(EN_lt, HIGH);
			digitalWrite(EN_rt, LOW);
			analogWrite(PWM_rt, speed);
			analogWrite(PWM_lt, speed);
			break;
		case HARD_LEFT:
			digitalWrite(EN_lt, LOW);
			digitalWrite(EN_rt, HIGH);
			analogWrite(PWM_rt, speed);
			analogWrite(PWM_lt, 0);
			break;
		default:
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
			break;
		}
	}
	else
	{
		analogWrite(PWM_lt, 0);
		analogWrite(PWM_rt, 0);
	}
}
*/
void LinePID() {
	unsigned int sensors[NUM_SENSORS];

	position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
	int error = position - 1500;



	int P = error * Kp_line;
	I = (I + error) * Ki_line;
	int D = (error - lastError) * Kd_line;
	int motorSpeed = P + I + D;
	lastError = error;
	
	int rightMotorSpeed = rightBaseSpeed + motorSpeed;
	int leftMotorSpeed = leftBaseSpeed - motorSpeed;
	
	if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
	if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
	if (rightMotorSpeed < 70) rightMotorSpeed = 0; // keep the motor speed positive
	if (leftMotorSpeed < 80) leftMotorSpeed = 0; // keep the motor speed positive

	analogWrite(rightMotorPWM, rightMotorSpeed);
	analogWrite(leftMotorPWM, leftMotorSpeed);
}

void BackUp() {
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_lt, 100);
	analogWrite(PWM_rt, 100);
	delay(750);
	analogWrite(PWM_lt, 0);
	analogWrite(PWM_rt, 0);
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 1);
	delay(50);
}

void UpdateSensors() {
	Sensor[0] = sonar[2].ping_cm();
	delay(33);

	//No idea why sharp_r.distance() was having issues, just using raw voltage 
	//to calculate cm now
	int raw = analogRead(IR_Right);
	float voltFromRaw = map(raw, 0, 1023, 0, 5000);
	Sensor[3] = 27.728*pow(voltFromRaw / 1000, -1.2045);
	//Sensor[3] = sharp_r.distance();
	Sensor[4] = sonar[1].ping_cm();
	delay(33);

	Sensor[1] = sharp_l.distance();
	Sensor[2] = sonar[0].ping_cm();
	delay(33);
	


	for (int i = 0; i < 5; i++) {
		Serial.print("Sensor["); Serial.print(i); Serial.print("]: "); Serial.print(Sensor[i]); Serial.print(" ");
	}
	Serial.println();
}

void MazePID() {
	UpdateSensors();
	int front_proximity = Sensor[2];
	int proximity = Sensor[1];

	
	if (front_proximity < 30 && front_proximity != 0) {
		BackUp();
		delay(500);
		if (Sensor[0] > 50) TurnLeft90();
		else	TurnRight90();
		delay(500);
	}


	int error = proximity - set_point ;



	int P = error * (Kp_line + P_adjust);
	I = (I + error) * (Ki_line + I_adjust);
	int D = (error - lastError) * (Kd_line + D_adjust );
	int motorSpeed = P + I + D;
	lastError = error;

	int rightMotorSpeed = rightBaseSpeed + motorSpeed;
	int leftMotorSpeed = leftBaseSpeed - motorSpeed;

	if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
	if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
	if (rightMotorSpeed < 70) rightMotorSpeed = 0; // keep the motor speed positive
	if (leftMotorSpeed < 80) leftMotorSpeed = 0; // keep the motor speed positive

	analogWrite(rightMotorPWM, rightMotorSpeed);
	analogWrite(leftMotorPWM, leftMotorSpeed);
	
	/*
	int P = proximity - set_point;
	int D = proximity - last_proximity;

	int Pd = P * .8 + D * 10;
	int left_motor = base_speed_lt - Pd;
	int right_motor = base_speed_rt + Pd;
	if (left_motor < 80) left_motor = 80;
	if (right_motor < 80) right_motor = 80;
	analogWrite(PWM_lt, left_motor);
	analogWrite(PWM_rt, right_motor);
	*/
	last_proximity = proximity;
	
	Serial.print("Left PWM: "); Serial.print(leftMotorSpeed); Serial.print(" Right PWM: "); Serial.print(rightMotorSpeed);
	Serial.print(" Left sensor: "); Serial.print(proximity);  Serial.print(" Front sensor: "); Serial.println(front_proximity);

}

void ModeLoop() {
	switch (FUNCTION)
	{
	case DRIVE:
		RobotStopped = false;
		if ( (distance_count_lt >= desired_distance) || (distance_count_rt >= desired_distance)) {
		/*	
		//Moved to STOP function		
		if (!RobotStopped) {
				digitalWrite(EN_lt, 1);
				digitalWrite(EN_rt, 0);
				analogWrite(PWM_rt, 30);
				analogWrite(PWM_lt, 50);
				//delay(1);
			}*/
			FUNCTION = STOP;
			distance_count_lt = 0;
			distance_count_rt = 0;
			PWM_val_lt = 0;
			PWM_val_rt = 0;
		}
		else {
			SetSpeed_L();
			SetSpeed_R();
			analogWrite(PWM_lt, PWM_val_lt);
			analogWrite(PWM_rt, PWM_val_rt);


		}
		break;

	case COURSE:
		CourseCounter = 1;
		FUNCTION = DRIVE;
		break;

	case MAZE:
		Maze = true;
		MazePID();
		break;
	case TURN:
		RobotStopped = false;
		if (DIRECTION == LEFT) {
			digitalWrite(EN_lt, 1);
			digitalWrite(EN_rt, 1);
			FUNCTION = DRIVE;
		}
		else if (DIRECTION == RIGHT) {
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 0);
			FUNCTION = DRIVE;
		}

		break;

	case LINE:

		LineFollow = true;
		break;

	case CALIBRATE:
		for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line
			qtrrc.calibrate();
		FUNCTION = STOP;
		break;

	case STOP:
		
		if (!RobotStopped) { 
			//This is applying "brakes"
		digitalWrite(EN_lt, 1);
		digitalWrite(EN_rt, 0);
		analogWrite(PWM_rt, 40);
		analogWrite(PWM_lt, 50);
		}
		else {

			analogWrite(PWM_rt, 0);
			analogWrite(PWM_lt, 0);
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
		//	count_correction = distance_count_rt;
		}
		RobotStopped = true;

		//target_lt = 100;
		//target_rt = 100;
		break;
	}
}


ISR(TIMER5_COMPA_vect) {

	TIMSK4 &= ~(1 << TOIE4);  //disable Timer4 interrupt
	TIMSK5 &= ~(1 << TOIE5);  //disable Timer5 interrupt
	FUNCTION = STOP;
	distance_count_lt = 0;
	distance_count_rt = 0;
	PWM_val_lt = 0;
	PWM_val_rt = 0;
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_rt, 20);
	analogWrite(PWM_lt, 30);
	RobotStopped = true;
}

ISR(TIMER4_COMPA_vect) {

	TIMSK4 &= ~(1 << TOIE4);  //disable Timer4 interrupt
	TIMSK5 &= ~(1 << TOIE5);  //disable Timer5 interrupt
	FUNCTION = STOP;
	distance_count_lt = 0;
	distance_count_rt = 0;
	PWM_val_lt = 0;
	PWM_val_rt = 0;
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_rt, 20);
	analogWrite(PWM_lt, 30);
	RobotStopped = true;
}

void LCDLoop() {
	ReadButtons();
	LCD.setCursor(0, 0);

	switch (StateIncrementer) {
	case 0: 
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Stopped   ");
		STATE = STOP;
		break;

	case 1:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Drive    ");
		STATE = DRIVE;
		desired_distance = CountsPerDistance(2);
		break;

	case 2:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Line     ");
		STATE = LINE;
		break;

	case 3:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Calibrate     ");
		STATE = CALIBRATE;
		unsigned int sensors[3];
		position = qtrrc.readLine(sensors);

		Serial.println(position);
		break;
	case 4:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Course     ");
		STATE = COURSE;
		break;
	case 5:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Maze        ");
		STATE = MAZE;

		break;
	default:
		StateIncrementer = 0;
		break;
	}
	/*
	if (button == 2) {
		button = 0;
		desired_distance = CountsPerDistance(100);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (button == 4) {
		button = 0;
		TurnLeft90();
		
		//distance_count_lt = 0;
		//distance_count_rt = 0;
		//ChangeAngle(90);
		
	}

	else if (button == 1) {
		button = 0;
		TurnRight90();
	}
	else if (button == 3) {
		button = 0;
		CourseCounter = 1;

	}
	else if (button == 5) {
		FUNCTION = STATE;
	}
	*/
	LCD.setCursor(0, 1);
	if (STATE == LINE || STATE == CALIBRATE) {
		LCD.setCursor(0, 1);
		if (position > 1900 && position < 2100) {
			LCD.print("     X0X  "); Serial.print(position);
		}
		else if (position < 999) {
			LCD.print("     0XX  "); LCD.print(position); LCD.print("    ");
		}
		else if (position > 1000) { LCD.print("     XX0  "); LCD.print(position); LCD.print("    ");
		}
		else { LCD.print("     XXX      "); LCD.print("         "); LCD.print("    ");
		}

	}
	else if (STATE == MAZE) {
		LCD.print(""); LCD.print(P_adjust); LCD.print(" "); LCD.print(I_adjust); LCD.print(" "); LCD.print(D_adjust);
	}
	else {
		LCD.print(" L: ");	LCD.print(RPM_lt);
		LCD.print(" R: ");    LCD.print(RPM_rt); LCD.print("     ");


	}
}

void TurnRight90() {
	int temp = TCNT4;
	int temp2 = TCNT5;
	//RobotStopped = false;
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_lt, 115);
	analogWrite(PWM_rt, 105);
	delay(190);
	analogWrite(PWM_lt, 0);
	analogWrite(PWM_rt, 0);
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 1);
	delay(50);

	TCNT4 = temp;
	TCNT5 = temp2;

}

void TurnLeft90() {
	int temp = TCNT4;
	int temp2 = TCNT5;
	//RobotStopped = false;
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 1);
	analogWrite(PWM_lt, 115);
	analogWrite(PWM_rt, 105);
	delay(205);
	analogWrite(PWM_lt, 0);
	analogWrite(PWM_rt, 0);
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 1);
	delay(50);
	TCNT4 = temp;
	TCNT5 = temp2;
}

void CourseControl() {
	if (CourseCounter == 1 && RobotStopped) {
		delay(250);
		CourseCounter++;
		FUNCTION = DRIVE;
		desired_distance = CountsPerDistance(20);
		distance_count_lt = 0;
		distance_count_rt = 0;		
	}
	else if (CourseCounter == 2 && RobotStopped ){
		delay(250);
		CourseCounter++;
		TurnRight90();
		delay(250);
		desired_distance = CountsPerDistance(5.5);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (CourseCounter == 3 && RobotStopped) {
		delay(250);
		CourseCounter++;
		TurnRight90();
		delay(250);
		desired_distance = CountsPerDistance(20);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (CourseCounter == 4 && RobotStopped) {
		delay(250);
		CourseCounter++;
		TurnRight90();
		delay(250);
		desired_distance = CountsPerDistance(2);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
		CourseCounter = 0;
	}
	else if (CourseCounter == 5 && RobotStopped) {
		delay(500);
		TurnRight90();
		delay(200);
		TurnRight90();
		delay(200);
		CourseCounter++;

	}
	else if (CourseCounter == 6 && RobotStopped) {
		delay(500);
		CourseCounter++;
		desired_distance = CountsPerDistance(60);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (CourseCounter == 7 && RobotStopped) {
		delay(500);
		CourseCounter++;
		TurnLeft90();
		delay(200);
		desired_distance = CountsPerDistance(36);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (CourseCounter == 8 && RobotStopped) {
		delay(500);
		CourseCounter++;
		TurnLeft90();
		delay(200);
		desired_distance = CountsPerDistance(96);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
	}
	else if (CourseCounter == 9 && RobotStopped) {
		delay(500);
		CourseCounter++;
		TurnLeft90();
		delay(200);
		desired_distance = CountsPerDistance(24);
		distance_count_lt = 0;
		distance_count_rt = 0;
		FUNCTION = DRIVE;
		CourseCounter = 0;
	}


}

void loop() {
	if (!LineFollow) {
	if ((millis() - lastMilli) >= LOOPTIME)
	{
		lastMilli = millis();
		getCount();
		if (DEBUG) printCount();
		if (CourseCounter != 0) CourseControl();
		ModeLoop();
		LCDLoop();
	}
	}
	else if (LineFollow) LinePID();

}

//Given a distance, returns the amount of counts on the output shaft to get there
int CountsPerDistance(double distance) {
//	int temp = count_correction;
//	if (FUNCTION == TURN) temp = 0;
	return (round((distance / (WHEEL_SIZE * PI)) * 2249)); // -temp; //1100 is roughly how many extra encoder ticks happen after shutting the motors off
}

void SetSpeed_R() {
	PWM_val_rt = updatePid_rt(PWM_val_rt, target_rt, RPM_rt);

}

void SetSpeed_L() {
	PWM_val_lt = updatePid_lt(PWM_val_lt, target_lt, RPM_lt);
}

int updatePid_rt(int command, int targetValue, int currentValue) {      // compute PWM value
	float pidTerm = 0;                                                           // PID correction
	int error = 0;
	int dErr = 0;
	int errSum = 0;

	error = abs(targetValue) - abs(currentValue);
	dErr = round(((error - last_error_rt) / LOOPTIME / 1000));
	errSum = error * (LOOPTIME / 1000);


	pidTerm = ((Kp + .12) * error) + ( (Kd + .2) * (dErr)) + ( (Ki + .2  ) * errSum);
	last_error_rt = error;
	return constrain(command + int(pidTerm), 76, 255);
}

int updatePid_lt(int command, int targetValue, int currentValue) {      // compute PWM value
	float pidTerm = 0;                                                           // PID correction
	int error = 0;
	int dErr = 0;
	int errSum = 0;

	error = abs(targetValue) - abs(currentValue);
	dErr = (error - last_error_lt) / (LOOPTIME/1000);
	errSum = error * (LOOPTIME / 1000);

	//.33
	pidTerm = ((Kp + .15 ) * error) + ( (Kd + .2) * (dErr)) + ((Ki + .2) * errSum );
	last_error_lt = error;
	return constrain(command + int(pidTerm), 87, 255);
}

void getCount() {
	distance_count_lt += TCNT4;
	distance_count_rt += TCNT5;

	lastCount_rt = speed_count_rt;
	lastCount_lt = speed_count_lt;
	speed_count_rt = TCNT5;
	speed_count_lt = TCNT4;


	RPM_lt = (speed_count_lt) * 60 * ((1000 / LOOPTIME)) / 2249;
	RPM_rt = (speed_count_rt) * 60 * ((1000 / LOOPTIME)) / 2249;

	TCNT5 = 0;
	TCNT4 = 0;
}

void printCount() {

	Serial.print("Count_rt:  ");  Serial.print(speed_count_rt);
	Serial.print("  Last Count_rt:  "); Serial.print(lastCount_rt);
	Serial.print("  PWM_val_rt:  "); Serial.print(PWM_val_rt);
	Serial.print("  RPM_rt:  "); Serial.print(RPM_rt);
	Serial.print("  Distance count right:  ");  Serial.print(distance_count_rt);
	Serial.print("  Desired distance: "); Serial.println(desired_distance);

	Serial.print("Count_lt:  ");  Serial.print(speed_count_lt);
	Serial.print("  Last Count_lt:  "); Serial.print(lastCount_lt);
	Serial.print("  PWM_val_lt:  "); Serial.print(PWM_val_lt);
	Serial.print("  RPM_lt:  "); Serial.println(RPM_lt);
}