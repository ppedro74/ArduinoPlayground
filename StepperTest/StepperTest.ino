#include <Arduino.h>
#include <SerialCommands.h>

//*** Stepper Driver Selection ***
//#define USE_L298N
//#define USE_DRV8825_LIB
#define USE_DRV8825

const int DEBUG_INTERVAL = 1000;

const int8_t TOP_SWITCH_PIN = A0;
const int8_t BOTTOM_SWITCH_PIN = A1;
const int8_t BUTTON_PIN = 2;

enum STEPPER_STATE
{
	STEPPER_STATE_IDLE = 0,
	STEPPER_STATE_MOVING_UP,
	STEPPER_STATE_MOVING_DOWN
};



#if defined(USE_L298N)
#include <Stepper.h>
const int8_t L298N_ENA_PIN = 5;
const int8_t L298N_IN1_PIN = 6;
const int8_t L298N_IN2_PIN = 7;
const int8_t L298N_IN3_PIN = 8;
const int8_t L298N_IN4_PIN = 9;
const int8_t L298N_ENB_PIN = 10;
Stepper stepper_(MOTOR_STEPS_PER_REVOLUTION, L298N_IN1_PIN, L298N_IN2_PIN, L298N_IN3_PIN, L298N_IN4_PIN);
#elif defined(USE_DRV8825)
const int DRV8825_SLEEP_PIN = 11;
const int DRV8825_STEP_PIN = 3;
const int DRV8825_DIR_PIN = 4;

//microseconds
volatile int delay_between_steps_ = 800;

#elif defined(USE_DRV8825_LIB)
const int DRV8825_SLEEP_PIN = 11;
const int DRV8825_STEP_PIN = 3;
const int DRV8825_DIR_PIN = 4;

#include "DRV8825.h"
//Motor: Number Steps per revolution
const int MOTOR_STEPS_PER_REVOLUTION = 200;
const int SPEED_RPM = 100;
//const long TOTAL_STEPS = MOTOR_STEPS_PER_REVOLUTION * 5;
DRV8825 stepper_(MOTOR_STEPS_PER_REVOLUTION, DRV8825_DIR_PIN, DRV8825_STEP_PIN, DRV8825_SLEEP_PIN);
#endif


volatile STEPPER_STATE stepper_state_ = STEPPER_STATE_IDLE;
volatile STEPPER_STATE stepper_prev_state_ = STEPPER_STATE_IDLE;
volatile STEPPER_STATE stepper_last_direction_ = STEPPER_STATE_MOVING_UP;
int position_ = 0;
uint32_t check_stepper_next_millis_ = millis();
uint32_t debug_next_millis_ = millis();

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

void cmd_up(SerialCommands* sender)
{
	sender->GetSerial()->println("=>up");
	stepper_prev_state_ = STEPPER_STATE_IDLE;
	stepper_last_direction_ = stepper_state_ = STEPPER_STATE_MOVING_UP;
}

void cmd_down(SerialCommands* sender)
{
	sender->GetSerial()->println("=>down");
	stepper_prev_state_ = STEPPER_STATE_IDLE;
	stepper_last_direction_ = stepper_state_ = STEPPER_STATE_MOVING_DOWN;
}

void cmd_stop(SerialCommands* sender)
{
	sender->GetSerial()->println("=>stop");
	stepper_state_ = STEPPER_STATE_IDLE;
}

void cmd_set_speed(SerialCommands* sender)
{
	char* str = sender->Next();
	if (str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_ARG");
		return;
	}
	int value = atoi(str);

	sender->GetSerial()->print("=>set rpm/delay to ");
	sender->GetSerial()->println(value);
#if defined(USE_DRV8825_LIB)	
	stepper_.begin(value, 1);
#else
	delay_between_steps_ = value;
#endif
}

void cmd_move(SerialCommands* sender)
{
	char* str = sender->Next();
	if (str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_DIR");
		return;
	}
	int dir = atoi(str);
	
	str = sender->Next();
	if (str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_STEPS");
		return;
	}
	int steps = atoi(str);

	str = sender->Next();
	if (str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_DELAY");
		return;
	}
	int delay_between_steps = atoi(str);

	sender->GetSerial()->print("move dir=");
	sender->GetSerial()->print(dir == 0 ? "DN" : "UP");
	sender->GetSerial()->print(" #steps=");
	sender->GetSerial()->print(steps);
	sender->GetSerial()->print(" delay=");
	sender->GetSerial()->print(delay_between_steps);
	sender->GetSerial()->println();

	digitalWrite(DRV8825_DIR_PIN, dir == 0 ? LOW : HIGH);
	digitalWrite(DRV8825_STEP_PIN, LOW);
	digitalWrite(DRV8825_SLEEP_PIN, HIGH);
	//wake time 
	delay(4);

	for (auto sx = 0; sx < steps; sx++)
	{
		delayMicroseconds(delay_between_steps);
		digitalWrite(DRV8825_STEP_PIN, HIGH);
		delayMicroseconds(delay_between_steps);
		digitalWrite(DRV8825_STEP_PIN, LOW);
	}

	sender->GetSerial()->println("move finished");

	digitalWrite(DRV8825_STEP_PIN, LOW);
	digitalWrite(DRV8825_SLEEP_PIN, LOW);
}



SerialCommand cmd_up_("up", cmd_up);
SerialCommand cmd_down_("dn", cmd_down);
SerialCommand cmd_stop_("stop", cmd_stop);
SerialCommand cmd_set_speed_("speed", cmd_set_speed);
SerialCommand cmd_move_("move", cmd_move);

void button_click()
{
	if (digitalRead(TOP_SWITCH_PIN) == LOW)
	{
		stepper_last_direction_ = stepper_state_ = STEPPER_STATE_MOVING_DOWN;
	}
	else if (digitalRead(BOTTOM_SWITCH_PIN) == LOW)
	{
		stepper_last_direction_ = stepper_state_ = STEPPER_STATE_MOVING_UP;
	}
	else
	{
		stepper_state_ = stepper_last_direction_;
	}
}

void setup()
{
	Serial.begin(115200);

#if defined(USE_L298N)
	pinMode(L298N_ENA_PIN, OUTPUT);
	pinMode(L298N_ENB_PIN, OUTPUT);
	stepper_.setSpeed(SPEED_RPM);
#elif defined(USE_DRV8825)	
	pinMode(DRV8825_SLEEP_PIN, OUTPUT);
	digitalWrite(DRV8825_SLEEP_PIN, LOW);

	pinMode(DRV8825_STEP_PIN, OUTPUT);
	digitalWrite(DRV8825_STEP_PIN, LOW);

	pinMode(DRV8825_DIR_PIN, OUTPUT);
	digitalWrite(DRV8825_DIR_PIN, LOW);
#elif defined(USE_DRV8825_LIB)	
	//1=full step, 2=half step etc.
	stepper_.begin(SPEED_RPM, 1);
	stepper_.disable();
#endif
	pinMode(TOP_SWITCH_PIN, INPUT_PULLUP);
	pinMode(BOTTOM_SWITCH_PIN, INPUT_PULLUP);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_click, FALLING);

	Serial.println("Ready!");

	if (digitalRead(TOP_SWITCH_PIN) == LOW)
	{
		Serial.println("Stepper is in the TOP position!");
	}
	else if (digitalRead(BOTTOM_SWITCH_PIN) == LOW)
	{
		Serial.println("Stepper is in the BOTTOM position!");
	}
	else
	{
		Serial.println("Stepper is somewhere!");
	}

	//Serial.println("Moving Positive!");
	//stepper_.move(MOTOR_STEPS_PER_REVOLUTION * 2);
	//delay(2000);
	//Serial.println("Moving Negative!");
	//stepper_.move(-MOTOR_STEPS_PER_REVOLUTION * 3);
	//stepper_last_direction_ = stepper_state_ = STEPPER_STATE_MOVING_UP;

	serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmd_up_);
	serial_commands_.AddCommand(&cmd_down_);
	serial_commands_.AddCommand(&cmd_stop_);
	serial_commands_.AddCommand(&cmd_set_speed_);
	serial_commands_.AddCommand(&cmd_move_);

	Serial.println("Ready!");
}

void loop()
{
	if (stepper_state_ != STEPPER_STATE_IDLE)
	{
		if (stepper_prev_state_ == STEPPER_STATE_IDLE)
		{
			Serial.print("Start moving ");
			Serial.print(stepper_state_ == STEPPER_STATE_MOVING_UP ? "Up" : "Down");
			Serial.println();

#if defined(USE_L298N)
			//Enable H-Bridge M1 & M2
			digitalWrite(L298N_ENA_PIN, HIGH);
			digitalWrite(L298N_ENB_PIN, HIGH);
#elif defined(USE_DRV8825_LIB)
			stepper_.enable();
#elif defined(USE_DRV8825)
			digitalWrite(DRV8825_DIR_PIN, stepper_state_ == STEPPER_STATE_MOVING_UP ? HIGH : LOW);
			digitalWrite(DRV8825_STEP_PIN, LOW);
			digitalWrite(DRV8825_SLEEP_PIN, HIGH);
			//wake time 
			delay(2);
#endif
		}

		//Moving
#if defined(USE_L298N)
		stepper_.step(stepper_state_ == STEPPER_STATE_MOVING_UP ? 1 : -1);
#elif defined(USE_DRV8825_LIB)
		stepper_.move(stepper_state_ == STEPPER_STATE_MOVING_UP ? 2 : -2);
#elif defined(USE_DRV8825)

		delayMicroseconds(delay_between_steps_);
		digitalWrite(DRV8825_STEP_PIN, HIGH);
		delayMicroseconds(delay_between_steps_);
		digitalWrite(DRV8825_STEP_PIN, LOW);

#endif
		position_ += stepper_state_ == STEPPER_STATE_MOVING_UP ? 1 : -1;

		if (stepper_state_ == STEPPER_STATE_MOVING_UP)
		{
			if (digitalRead(TOP_SWITCH_PIN) == LOW)
			{
				Serial.println("Reached the Top!");
				stepper_state_ = STEPPER_STATE_IDLE;

				Serial.print("Position=");
				Serial.print(position_);
				Serial.println();
			}
		}
		else if (stepper_state_ == STEPPER_STATE_MOVING_DOWN)
		{
			if (digitalRead(BOTTOM_SWITCH_PIN) == LOW)
			{
				Serial.println("Reached the Bottom!");
				stepper_state_ = STEPPER_STATE_IDLE;

				Serial.print("Position=");
				Serial.print(position_);
				Serial.println();

				position_ = 0;
			}
		}

	}

	if (stepper_state_ == STEPPER_STATE_IDLE)
	{
		if (stepper_prev_state_ != STEPPER_STATE_IDLE)
		{
			//stopping
			Serial.println("Stopped!");

#if defined(USE_L298N)
			//Disable H-Bridge M1 & M2
			digitalWrite(L298N_ENA_PIN, LOW);
			digitalWrite(L298N_ENB_PIN, LOW);
#elif defined(USE_DRV8825_LIB)
			stepper_.disable();
#elif defined(USE_DRV8825) || defined(USE_DRV8825_LIB)
			digitalWrite(DRV8825_STEP_PIN, LOW);
			digitalWrite(DRV8825_SLEEP_PIN, LOW);
#endif
		}
	}

	stepper_prev_state_ = stepper_state_;

	if (millis() >= debug_next_millis_)
	{
		debug_next_millis_ = millis() + DEBUG_INTERVAL;

		Serial.print("Position=");
		Serial.print(position_);
		Serial.print(" State=");
		Serial.print(stepper_state_);
		Serial.print(" PrevState=");
		Serial.print(stepper_prev_state_);
		Serial.print(" LastDirection=");
		Serial.print(stepper_last_direction_);
		Serial.println();
	}

	serial_commands_.ReadSerial();
}
