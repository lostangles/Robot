#include <Wire.h>
#include <LiquidCrystal.h>
#include "Arduino.h"
#include <math.h>
#include "SharpIR.h"
#include "QTRSensors.h"





//Line sensor defines
#define Kp_line .13 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd_line 1 
#define Ki_line 0// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
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
#define WHEEL_SIZE 1.18
#define WHEEL_DISTANCE 6.69


#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define IR_LeftLeft A15
#define IR_Front A14
#define IR_RightRight A7
#define IR_Left A9
#define IR_Right A6
SharpIR sharp_l(IR_Left, 25, 93, 1080);
SharpIR sharp_r(IR_Right, 25, 93, 1080);
int Sensor[4];

int last_proximity = 0;
int base_speed_lt = 150;
int base_speed_rt = 150;
const int set_point = 30;

double P_adjust = 0;
double I_adjust = 0;
double D_adjust = 0;
int PIorD = 0;


////////////PROJECT VARIABLES///////////////
float x, y, Beta, theta, Vr, Vl, w, R;
int DirL = 0;
int DirR = 1;
char cmd[15];
int target_x, target_y, target_theta;
bool NewCommand = false;
bool Waypoint = false;
int errSum = 0;
int DistanceToGoal = 0;
int InitialDistanceToGoal = 0;
float Goal[3];
float PreGoal[3];
int errSum_lt = 0;
int errSum_rt = 0;




//Sensor 0 = front
//Sensor 1 = Right
//Sensor 2 = left


boolean Maze = false;

//Line following stuff
unsigned char QTIPins[] = { A1, A2, A3, A4, A5 };
QTRSensorsRC qtrrc(QTIPins, 3, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
int lastError = 0;
double I = 0;
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
	 MAZE,
	LEADER,
	FOLLOWER,
	COMMAND,
	NAVIGATE
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
volatile boolean RaceStarted = false;



//Motor/Encoder variables
volatile int target_rt = 150;
volatile int target_lt = 150;
volatile int RPM_rt = 0;
volatile int RPM_lt = 0;
static int last_error_rt = 0;
static int last_error_lt = 0;
unsigned long currentMilli = 0;
unsigned long lastMilli = 0;
volatile unsigned long speed_count_rt = 0;
volatile unsigned long speed_count_lt = 0;

volatile unsigned long distance_count_rt = 0;
volatile unsigned long distance_count_lt = 0;
volatile unsigned long desired_distance = 0;
volatile unsigned long count = 0;


volatile long lastCount_rt = 0;
volatile long lastCount_lt = 0;
volatile long count_correction = 1000; // take into account that the wheels don't stop when PWM's go to  0

int PWM_val_rt = 0;
int PWM_val_lt = 0;

float Kp = 1.5;//.45;          //setting Kp  
float Ki = 1;// .5;// 1.8; // .55 //.98
float Kd = .5;// .5;// 5.5;// 1.8; // 1.5;       //.58 //.98    //setting Kd

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
		else StateIncrementer = 8;
		}

	}
	else if (x < 800) {
		if (FUNCTION == STOP) {
		button = 5;
		FUNCTION = STATE;
		distance_count_lt = 0;
		distance_count_rt = 0;
		//LCD.print("Select");
		}
		else {
			FUNCTION = STOP;
			PWM_val_lt = 0;
			PWM_val_rt = 0;
		}
	}
	delay(200);
}


void ChangeAngle(int angle) {
	int steps = CountsPerDistance(  angle * (28.26 / 360) );
	desired_distance = steps;
	OCR4A = steps;
	OCR5A = steps;
	TIMSK4 |=  (1 << TOIE4); 
	TIMSK5 |=  (1 << TOIE5); 
	FUNCTION = TURN;
}

void setup() {
	Serial.begin(115200);
	FUNCTION = STOP;
	DIRECTION = LEFT;
	
	pinInit();
	LCDInit();
	TCInit();


	TCNT5 = 0;
	TCNT4 = 0;
	x = 0;
	y = 0;
	theta = PI / 2;
	Vr = 0;
	Vl = 0;
	DirL = 0;
	DirR = 1;
	while (Serial.available() > 0) char flushBuffer = Serial.read();
}

void Brake() {
	//This is applying "brakes"
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_rt, 20);
	analogWrite(PWM_lt, 30);
	RobotStopped = true;
}

void LinePID() {
	unsigned int sensors[NUM_SENSORS];
	
	if (!RaceStarted) UpdateSensors();
	position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
	int error = position - 1500;

	if (Sensor[2] < 20 && !RaceStarted ) {
	
		analogWrite(rightMotorPWM, 0);
		analogWrite(leftMotorPWM, 0);
	}

	else {
		RaceStarted = true;

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
}

void BackUp() {
	digitalWrite(EN_lt, 1);
	digitalWrite(EN_rt, 0);
	analogWrite(PWM_lt, 110);
	analogWrite(PWM_rt, 100);
	delay(300);
	analogWrite(PWM_lt, 0);
	analogWrite(PWM_rt, 0);
	digitalWrite(EN_lt, 0);
	digitalWrite(EN_rt, 1);
	delay(50);
}

void UpdateSensors() {

	int raw = analogRead(IR_Right);
	float voltFromRaw = map(raw, 0, 1023, 0, 5000);
	Sensor[1] = 27.728*pow(voltFromRaw / 1000, -1.2045);

	raw = analogRead(IR_LeftLeft);
	voltFromRaw = map(raw, 0, 1023, 0, 5000);
	Sensor[4] = 27.728*pow(voltFromRaw / 1000, -1.2045);

	raw = analogRead(IR_Front);
	voltFromRaw = map(raw, 0, 1023, 0, 5000);
	Sensor[0] = 27.728*pow(voltFromRaw / 1000, -1.2045);

	raw = analogRead(IR_RightRight);
	voltFromRaw = map(raw, 0, 1023, 0, 5000);
	Sensor[2] = 27.728*pow(voltFromRaw / 1000, -1.2045);

	Sensor[3] = sharp_l.distance();

	if (Sensor[0] == 50 || Sensor[0] == 51) Sensor[0] = 0;
	if (Sensor[4] == 50 || Sensor[4] == 51) Sensor[4] = 0;


	/*
	for (int i = 0; i < 5; i++) {
		Serial.print("Sensor["); Serial.print(i); Serial.print("]: "); Serial.print(Sensor[i]); Serial.print(" ");
	}
	Serial.print(" Target_rt: "); Serial.print(target_rt); Serial.print(" Target_lt: "); Serial.print(target_lt);
	Serial.print(" RPM_rt: "); Serial.print(RPM_rt); Serial.print(" RPM_lt: "); Serial.print(RPM_lt); 
	Serial.print(" PWM_val_lt: "); Serial.print(PWM_val_lt); Serial.print(" PWM_val_rt: "); Serial.print(PWM_val_rt);
	Serial.println();*/
}

void MazePID() {
	UpdateSensors();
	int front_proximity = Sensor[2];
	int proximity = Sensor[1];

	
	if (front_proximity < 30 && front_proximity != 0) {
		BackUp();
		delay(500);
			TurnLeft90();
		delay(500);
	}


	double error = proximity - set_point ;



	double P = error * ( P_adjust);
	I = (I + error) * (I_adjust);
	double D = (error - lastError) * ( D_adjust );
	double motorSpeed =  (P + I + D);
	lastError = error;

	int rightMotorSpeed = rightBaseSpeed + motorSpeed;
	int leftMotorSpeed = leftBaseSpeed - motorSpeed;

	if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
	if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
	if (rightMotorSpeed < 70) rightMotorSpeed = 80; // keep the motor speed positive
	if (leftMotorSpeed < 80) leftMotorSpeed = 80; // keep the motor speed positive

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

void Leader() {

//	unsigned int sensors[NUM_SENSORS];
	UpdateSensors();
//	int cliff = qtrrc.readLine(sensors);
//	Serial.println("Cliff: "); Serial.print(cliff);
//	position = cliff;
	while ( (Sensor[0] > 7 || Sensor[2] < 20) && Sensor[0] != 0 && Sensor[2] != 0) {
		if (Sensor[0] > 15) FUNCTION = STOP;
		delay(1000);
		analogWrite(PWM_lt, 0);
		analogWrite(PWM_rt, 0);
		UpdateSensors();
		
		//	FUNCTION = STOP;
	}
	
		if (Sensor[1] > 15)
		{
			//PWM_val_lt += 15;
			//PWM_val_rt -= 10;
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 0);
			analogWrite(PWM_lt, 120);
			analogWrite(PWM_rt, 100);
			delay(15);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
		}
		else if (Sensor[3] > 15) {
			//PWM_val_lt -= 10;
			//PWM_val_rt += 15;

			digitalWrite(EN_lt, 1);
			digitalWrite(EN_rt, 1);
			analogWrite(PWM_lt, 110);
			analogWrite(PWM_rt, 100);
			delay(15);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
		}
		else {
			
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
			analogWrite(PWM_lt, 120);
			analogWrite(PWM_rt, 100);
			delay(30);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
		}
		//SetSpeed_L();
		//SetSpeed_R();
		/*
		if (PWM_val_lt > 200) PWM_val_lt = 200;
		if (PWM_val_rt > 200) PWM_val_rt = 200;
		if (PWM_val_lt < 70) PWM_val_lt = 70;
		if (PWM_val_rt < 75) PWM_val_rt = 75;
		*/
		//delay(100);

	}

void Follower() {

	//	unsigned int sensors[NUM_SENSORS];
	UpdateSensors();
	float error = Sensor[2] - 30;
	float Kp = .5;
	
	int PWM_value = round( constrain((error)* Kp + 120, 90, 255));

	if ((error) > 0)
	{

		if (Sensor[1] > 15)
		{
			//PWM_val_lt += 15;
			//PWM_val_rt -= 10;
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 0);
			analogWrite(PWM_lt, 120);
			analogWrite(PWM_rt, 100);
			delay(10);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
		}
		else if (Sensor[3] > 15) {
			//PWM_val_lt -= 10;
			//PWM_val_rt += 15;

			digitalWrite(EN_lt, 1);
			digitalWrite(EN_rt, 1);
			analogWrite(PWM_lt, 110);
			analogWrite(PWM_rt, 100);
			delay(10);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
		}
		else {

			digitalWrite(EN_lt, 0);
			digitalWrite(EN_rt, 1);
			analogWrite(PWM_lt, PWM_value + 5);
			analogWrite(PWM_rt, PWM_value);
			delay(30);
			analogWrite(PWM_lt, 0);
			analogWrite(PWM_rt, 0);
		}
		//SetSpeed_L();
		//SetSpeed_R();
		/*
		if (PWM_val_lt > 200) PWM_val_lt = 200;
		if (PWM_val_rt > 200) PWM_val_rt = 200;
		if (PWM_val_lt < 70) PWM_val_lt = 70;
		if (PWM_val_rt < 75) PWM_val_rt = 75;
		*/
		//delay(25);
	}
	
		//analogWrite(PWM_lt, 0);
		//analogWrite(PWM_rt, 0);

	
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
			printCount();
			SetSpeed_L();
			SetSpeed_R();
			analogWrite(PWM_lt, PWM_val_lt);
			analogWrite(PWM_rt, PWM_val_rt);


		}
		break;

	case LEADER: 
		RobotStopped = false;
		//Leader();
		break;
	case FOLLOWER:
		RobotStopped = false;
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
	case COMMAND:
		//printCount();
		if (NewCommand) executeCmd();

		break;
	case NAVIGATE:
		getCmd();
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

	//TIMSK4 &= ~(1 << TOIE4);  //disable Timer4 interrupt
	//TIMSK5 &= ~(1 << TOIE5);  //disable Timer5 interrupt

}

ISR(TIMER4_COMPA_vect) {

	//TIMSK4 &= ~(1 << TOIE4);  //disable Timer4 interrupt
	//TIMSK5 &= ~(1 << TOIE5);  //disable Timer5 interrupt
	
}

void LCDLoop() {
	if (FUNCTION == STOP)
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
		desired_distance = CountsPerDistance(60);
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
		UpdateSensors();
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
	case 6:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Leader        ");
		STATE = LEADER;

		break;
	case 7:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Follower        ");
		STATE = FOLLOWER;

		break;
	case 8:
		LCD.setCursor(0, 0);
		LCD.print("Mode:  Navigate        ");
		STATE = NAVIGATE;

		break;
	default:
		StateIncrementer = 0;
		break;
	}
	LCD.setCursor(0, 1);
	if (STATE == LINE || STATE == CALIBRATE) {
		LCD.setCursor(0, 1);
		LCD.print(Sensor[2]);
		if (position > 1400 && position < 1600) {
			LCD.print("  X0X  "); Serial.print(position);
		}
		else if (position < 1400) {
			LCD.print("  XX0  "); LCD.print(position); LCD.print("    ");
		}
		else if (position > 1600) { LCD.print("  0XX  "); LCD.print(position); LCD.print("    ");
		}
		else { LCD.print("  XXX      "); LCD.print("         "); LCD.print("    ");
		}

	}
	else if (STATE == MAZE) {
		LCD.print(""); LCD.print(P_adjust); LCD.print(" "); LCD.print(I_adjust); LCD.print(" "); LCD.print(D_adjust);
	}
	else if (STATE == LEADER || STATE == FOLLOWER) {
		LCD.print(" L: ");	LCD.print(Sensor[1]);
		LCD.print(" R: ");    LCD.print(Sensor[3]); LCD.print(" "); LCD.print(Sensor[0]); LCD.print("     ");
	}
	else if (STATE == NAVIGATE) {
		LCD.print("X: ");	LCD.print(x);
		LCD.print(" Y: ");    LCD.print(y);  LCD.print("     ");
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
	if (!LineFollow && !(FUNCTION == LEADER) && !(FUNCTION == FOLLOWER)) {
	if ((millis() - lastMilli) >= LOOPTIME)
	{
		currentMilli = lastMilli;
		lastMilli = millis();
		getCount();
		if (DEBUG) printCount();
		if (CourseCounter != 0) CourseControl();
		LCDLoop();
		ModeLoop();
		//printCount();
		if (STATE == NAVIGATE) getCmd();

	}
	}
	else if (LineFollow) LinePID();
	else if (FUNCTION == LEADER) {
		Leader(); LCDLoop();
	}
	else if (FUNCTION == FOLLOWER) {
		Follower();
		LCDLoop();
	}
//	UpdateSensors();

}

//Given a distance, returns the amount of counts on the output shaft to get there
int CountsPerDistance(double distance) {
//	int temp = count_correction;
//	if (FUNCTION == TURN) temp = 0;
	return (round((distance / (WHEEL_SIZE * PI * 2 * 2)) * (2249/3.0))); // -temp; //1100 is roughly how many extra encoder ticks happen after shutting the motors off
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

	error = abs(targetValue) - abs(currentValue);
	dErr = round(((error - last_error_rt) / (LOOPTIME / 1000)));
	errSum_rt += error * (LOOPTIME / 1000);


	pidTerm = ((Kp - 0) * error) + ( (Kd + 0) * (dErr)) + ( (Ki   ) * errSum);
	last_error_rt = error;
	return constrain(command + int(pidTerm), 76, 255);
}

int updatePid_lt(int command, int targetValue, int currentValue) {      // compute PWM value
	float pidTerm = 0;                                                           // PID correction
	int error = 0;
	int dErr = 0;

	error = abs(targetValue) - abs(currentValue);
	dErr = (error - last_error_lt) / (LOOPTIME/1000);
	errSum_lt += error * (LOOPTIME / 1000);

	//.33
	pidTerm = ((Kp + 0 ) * error) + ( (Kd + 0) * (dErr)) + ((Ki + 0 ) * errSum );
	last_error_lt = error;
	return constrain(command + int(pidTerm), 93, 255);
}

void getCount() {
	cli();
	speed_count_rt = TCNT5;
	speed_count_lt = TCNT4;
	distance_count_lt += speed_count_lt;
	distance_count_rt += speed_count_rt;
	TCNT5 = 0;
	TCNT4 = 0;
	sei();
	lastCount_rt = speed_count_rt;
	lastCount_lt = speed_count_lt;



	RPM_lt = ((float)speed_count_lt) * 60 * ((1000.0 / (float)LOOPTIME)) / (2249.0/3.0);
	RPM_rt = ((float)speed_count_rt) * 60 * ((1000.0 / (float)LOOPTIME)) / (2249.0/3.0);

	Vl = RPM_lt * (WHEEL_SIZE ) / 60;
	Vr = RPM_rt * (WHEEL_SIZE ) / 60;

	x += Vr * cos(theta);
	y += Vl * sin(theta);
	theta += (Vr - Vl) / WHEEL_DISTANCE;

	if (Vr - Vl != 0)
		R = (WHEEL_DISTANCE / 2) * (Vr + Vl) / (Vr - Vl);
	else
		R = 0;

	w = (Vr - Vl) / WHEEL_DISTANCE;

	if (theta > 2 * PI )
		theta -= 2 * PI;
	else if (theta < -2 * PI )
		theta += 2 * PI;

	Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.print(theta); Serial.print(","); Serial.print(Vl); Serial.print(","); Serial.print(Vr); Serial.print(","); Serial.print(w); 
	Serial.print(","); Serial.println(R);
}

void printCount() {
	String message = "";
	message += "PWM_val_rt:  ";
	message += PWM_val_rt;
	message += "  RPM_rt:  ";
	message += RPM_rt;
	message += "\nPWM_val_lt:  ";
	message += PWM_val_lt;
	message += "  RPM_lt:  ";
	message += RPM_lt;
	message += " Vr: ";
	message += Vr;
	message += " Vl: ";
	message += Vl;
	message += " x: ";
	message += x;
	message += " y: ";
	message += y;
	message += " theta: ";
	message += theta * RAD_TO_DEG;
	message += " target x: ";
	message += target_x;
	message += " target_y: ";
	message += target_y;
	message += " target_theta: ";
	message += target_theta;
	message += " Time: ";
	
	Serial.print(message);
	Serial.println(lastMilli - currentMilli);
//	Serial.println(distance_count_lt);


	/*
	Serial.print("Count_rt:  ");  Serial.print(speed_count_rt);
	Serial.print("  Last Count_rt:  "); Serial.print(lastCount_rt);
	Serial.print("  PWM_val_rt:  "); Serial.print(PWM_val_rt);
	Serial.print("  RPM_rt:  "); Serial.print(RPM_rt);
	Serial.print("  Distance count right:  ");  Serial.print(distance_count_rt);
	Serial.print("  Desired distance: "); Serial.print(desired_distance);
	Serial.print("  Target_RPM_rt: "); Serial.println(target_rt);

	Serial.print("Count_lt:  ");  Serial.print(speed_count_lt);
	Serial.print("  Last Count_lt:  "); Serial.print(lastCount_lt);
	Serial.print("  PWM_val_lt:  "); Serial.print(PWM_val_lt);
	Serial.print("  RPM_lt:  "); Serial.println(RPM_lt);
	//Serial.print("  Target_RPM_lt: "); Serial.println(target_lt);
	*/
}

void getCmd() {
	int i = 0;
	if (Serial.available() > 0) {
		while (Serial.available() > 0) {
			char command = Serial.read();
			cmd[i] = command;
			i++;
		}
		if (cmd[0] != NULL) parseCommand();
	}
	 
}

void parseCommand() 
{
	int j = 0;
	int commaCnt = 0;
	char temp[15];
	//Serial.println(cmd);

	for (int i = 0; i < 10; i++) {

		if (cmd[i] == ',') 
		{
			temp[j + 1] = NULL;

			if (commaCnt == 0) 
			{
				target_x = atoi(temp);
				Goal[0] = target_x;
				j = 0;
			}
			else if (commaCnt == 1)
			{
				target_y = atoi(temp);
				Goal[1] = target_y;
				j = 0;
			}
			else if (commaCnt == 2)
			{
				target_theta = atoi(temp) * DEG_TO_RAD;
				j = 0;
				Goal[2] = target_theta;
				InitialDistanceToGoal = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));

				PreGoal[0] = target_x +  (InitialDistanceToGoal/2 * cos( target_theta + 180*DEG_TO_RAD  ));
				PreGoal[1] = target_y + (InitialDistanceToGoal/2 * sin( target_theta + 180*DEG_TO_RAD ));
				PreGoal[2] = (target_theta + 180*DEG_TO_RAD);
			}
			commaCnt++;
			temp[1] = temp[2] = temp[3] = temp[4] = NULL;
		}
		else 
		{
			temp[j] = cmd[i];
			j++;
		}
		NewCommand = true;
		FUNCTION = COMMAND;

	}

}

void executeCmd()
{

	SpeedTrack();
	HeadingTrack();
	if (FUNCTION == COMMAND && DistanceToGoal < 5 )
	{
		Brake();
		FUNCTION = STOP;
		NewCommand = false;
		Waypoint = false;
	}

}

void SpeedTrack() 
{
	//Serial.println( (Beta - theta) * RAD_TO_DEG);
	DistanceToGoal = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));


	int error = 0;
	int dErr = 0;

	error = (DistanceToGoal);
	dErr = ((error - last_error_lt) / (LOOPTIME / 1000));
	errSum += (error) * (LOOPTIME / 1000);

	last_error_lt = error;

	int PID = (8)*error + (3)*dErr + (4)*errSum;
    base_speed_rt = constrain(PID, 100, 200);
	base_speed_lt = constrain(PID, 100, 200);

	//Serial.println(error);
	//Serial.println(base_speed_lt);


}

void HeadingTrack() 
{
	//Serial.println( (Beta - theta) * RAD_TO_DEG);

	Beta = (atan2(target_y - y, target_x - x)) - (1.75 * DEG_TO_RAD);// -(45 * DEG_TO_RAD);


	//Serial.println((Beta)* RAD_TO_DEG);

	int error = 0;
	int dErr = 0;

	error = (Beta - theta) * RAD_TO_DEG;


	//Serial.println(error);
	dErr = ((error - last_error_rt) / (LOOPTIME / 1000));
	errSum += (error) * (LOOPTIME / 1000);

	last_error_rt = error;

	int motorSpeed = (8)*error + (3)*dErr + (4)*errSum;
	int rightMotorSpeed = base_speed_rt + motorSpeed;
	int leftMotorSpeed = base_speed_lt - motorSpeed;


	PWM_val_lt = constrain(leftMotorSpeed, 80, 255);
	PWM_val_rt = constrain(rightMotorSpeed, 80, 255);
	analogWrite(rightMotorPWM, PWM_val_rt);
	analogWrite(leftMotorPWM, PWM_val_lt);

}