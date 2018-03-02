#include <Arduino.h>
#include <Stepper.h>

const int DEBUG_INTERVAL = 1000;

//L298N H-Bridge
const int8_t ENA_PIN = 5;
const int8_t IN1_PIN = 6;
const int8_t IN2_PIN = 7;
const int8_t IN3_PIN = 8;
const int8_t IN4_PIN = 9;
const int8_t ENB_PIN = 10;

const int8_t TOP_SWITCH_PIN = A0;
const int8_t BOTTOM_SWITCH_PIN = A1;
const int8_t BUTTON_PIN = 2;

enum STEPPER_STATE
{
    STEPPER_STATE_IDLE,
    STEPPER_STATE_MOVING_UP,
    STEPPER_STATE_MOVING_DOWN
};

//20 = the number of steps in one revolution
Stepper stepper_(20, IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN);
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

    //enable coil A
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, HIGH);

    //enable coil B
    pinMode(ENB_PIN, OUTPUT);
    digitalWrite(ENB_PIN, HIGH);

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

    //600 revolutions per minute
    stepper_.setSpeed(600);
}

void loop()
{
    if (stepper_state_ != STEPPER_STATE_IDLE)
    {
        if (stepper_prev_state_ != stepper_state_)
        {
            Serial.print("Start moving ");
            Serial.print(STEPPER_STATE_MOVING_UP ? "Up" : "Down");
            Serial.println();
        }

        stepper_.step(stepper_state_ == STEPPER_STATE_MOVING_UP ? 1 : -1);
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

        if (millis() >= debug_next_millis_)
        {
            debug_next_millis_ = millis() + DEBUG_INTERVAL;

            Serial.print("Position=");
            Serial.print(position_);
            Serial.println();
        }
    }

    stepper_prev_state_ = stepper_state_;
}
