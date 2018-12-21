#include <Arduino.h>

//*** Stepper Driver Selection ***
//#define USE_L298N
#define USE_DRV8825

const int DEBUG_INTERVAL = 1000;

const int8_t TOP_SWITCH_PIN = A0;
const int8_t BOTTOM_SWITCH_PIN = A1;
const int8_t BUTTON_PIN = 2;

enum STEPPER_STATE
{
	STEPPER_STATE_IDLE,
	STEPPER_STATE_MOVING_UP,
	STEPPER_STATE_MOVING_DOWN
};

//Motor: Number Steps per revolution
const int MOTOR_STEPS_PER_REVOLUTION = 200;

const int SPEED_RPM = 100;


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
#include "DRV8825.h"
const int DRV8825_ENABLE_PIN = 11;
const int DRV8825_STEP_PIN = 3;
const int DRV8825_DIR_PIN = 4;
DRV8825 stepper_(MOTOR_STEPS_PER_REVOLUTION, DRV8825_DIR_PIN, DRV8825_STEP_PIN, DRV8825_ENABLE_PIN);
#endif


volatile STEPPER_STATE stepper_state_ = STEPPER_STATE_IDLE;
volatile STEPPER_STATE stepper_prev_state_ = STEPPER_STATE_IDLE;
volatile STEPPER_STATE stepper_last_direction_ = STEPPER_STATE_MOVING_UP;
int position_ = 0;
uint32_t check_stepper_next_millis_ = millis();
uint32_t debug_next_millis_ = millis();


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

	pinMode(DRV8825_ENABLE_PIN, OUTPUT);
	pinMode(DRV8825_STEP_PIN, OUTPUT);
	pinMode(DRV8825_DIR_PIN, OUTPUT);

	//1=full step, 2=half step etc.
	stepper_.begin(SPEED_RPM, 1);
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

}

void loop()
{
	if (stepper_state_ != STEPPER_STATE_IDLE)
	{
		if (stepper_prev_state_ == STEPPER_STATE_IDLE)
		{
			Serial.print("Start moving ");
			Serial.print(STEPPER_STATE_MOVING_UP ? "Up" : "Down");
			Serial.println();

#if defined(USE_L298N)
			//Enable H-Bridge M1 & M2
			digitalWrite(L298N_ENA_PIN, HIGH);
			digitalWrite(L298N_ENB_PIN, HIGH);
#elif defined(USE_DRV8825)
			stepper_.enable();
#endif
		}

#if defined(USE_L298N)
		stepper_.step(stepper_state_ == STEPPER_STATE_MOVING_UP ? 1 : -1);
#elif defined(USE_DRV8825)
		stepper_.move(stepper_state_ == STEPPER_STATE_MOVING_UP ? 1 : -1);
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

		if (stepper_state_ == STEPPER_STATE_IDLE)
		{
			Serial.println("Stopped!");

#if defined(USE_L298N)
			//Disable H-Bridge M1 & M2
			digitalWrite(L298N_ENA_PIN, LOW);
			digitalWrite(L298N_ENB_PIN, LOW);
#elif defined(USE_DRV8825)
			stepper_.disable();
#endif
		}

		stepper_prev_state_ = stepper_state_;
	}

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
}
