/*--------------------------------------------------------------------
Author		: Pedro Pereira
License		: BSD

SerialCommands - Playing with Arguments

--------------------------------------------------------------------*/
#include <Arduino.h>
#include "SerialCommands.h"

//Arduino UNO PWM pins
const int kRedLedPin = 3;
const int kGreenLedPin = 5;
const int kBlueLedPin = 9;

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

const int kMaxPushSubjects = 2;
const int kPushPingIndex = 0;
const int kPushAnalogIndex = 1;
//0 = disabled
int push_subjects_intervals_[kMaxPushSubjects] = { 0, 0};
unsigned long push_subjects_next_millis_[kMaxPushSubjects];

//some ping info
unsigned long ping_sequence_ = 0;
unsigned long loop_ticks_ = 0;

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

//expects one single parameter
void cmd_read_analog(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter

	char* port_str = sender->Next();
	if (port_str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_PORT");
		return;
	}

	int port = atoi(port_str);
	int value = -1;
	switch(port)
	{
		case 0:
			value = analogRead(A0);
			break;
		case 1:
			value = analogRead(A1);
			break;
		case 2:
			value = analogRead(A2);
			break;
	}
	
	sender->GetSerial()->print("ANALOG ");
	sender->GetSerial()->print(port);
	sender->GetSerial()->print(" ");
	sender->GetSerial()->print(value);
	sender->GetSerial()->println();
}

//helper function pass led string (lower case) and pwm value
bool set_led(char* led, int pwm)
{
	int pin = -1;

	//note: compare is case sensitive
	if (strcmp(led, "red") == 0)
	{
		pin = kRedLedPin;
	}
	else if (strcmp(led, "green") == 0)
	{
		pin = kGreenLedPin;
	}
	else if (strcmp(led, "blue") == 0)
	{
		pin = kBlueLedPin;
	}

	if (pin<0)
	{
		//invalid pin
		return false;
	}

	if (pwm <= 0)
	{
		digitalWrite(pin, LOW);
	}
	else if (pwm >= 255)
	{
		digitalWrite(pin, HIGH);
	}
	else
	{
		analogWrite(pin, pwm);
	}
	
}


//First parameter pwm value is required
//Optional parameters: led colors
//e.g. LED 128 red
//     LED 128 red blue
//     LED 0 red blue green
void cmd_set_led(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter

	char* pwm_str = sender->Next();
	if (pwm_str == NULL)
	{
		sender->GetSerial()->println("ERROR NO_PWM");
		return;
	}
	int pwm = atoi(pwm_str);

	char* led_str;
	while ((led_str = sender->Next()) != NULL)
	{
		if (set_led(led_str, pwm))
		{
      //let's send some feedback
			sender->GetSerial()->print("ACK_SET_LED ");
			sender->GetSerial()->print(led_str);
			sender->GetSerial()->print(" ");
			sender->GetSerial()->println(pwm);
		}
		else
		{
			sender->GetSerial()->print("ERROR ");
			sender->GetSerial()->println(led_str);
		}
	}
}

//First parameter is Push Subject's Index
//Second parameter is Push Subject's Interval (ms)
void cmd_start_push(SerialCommands* sender)
{
  char* index_str = sender->Next();
  if (index_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO_INDEX");
    return;
  }
  
  int index = atoi(index_str);
  if (index<0 || index>=kMaxPushSubjects)
  {
    sender->GetSerial()->println("ERROR INVALID_INDEX");
    return;
  }

  char* interval_str = sender->Next();
  if (interval_str == NULL)
  {
    sender->GetSerial()->println("ERROR INTERVAL");
    return;
  }

  int interval = atoi(interval_str);
  if (interval<100)
  {
    sender->GetSerial()->println("ERROR INVALID_INTERVAL");
    return;
  }

  push_subjects_intervals_[index] = interval;
  push_subjects_next_millis_[index] = millis() + interval;

  //let's send some feedback
  sender->GetSerial()->print("ACK_START_PUSH ");
  sender->GetSerial()->print(index);
  sender->GetSerial()->print(" ");
  sender->GetSerial()->println(interval);
}

//First parameter is Push Subject's Index
void cmd_stop_push(SerialCommands* sender)
{
  char* index_str = sender->Next();
  if (index_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO_INDEX");
    return;
  }
  
  int index = atoi(index_str);
  if (index<0 || index>=kMaxPushSubjects)
  {
    sender->GetSerial()->println("ERROR INVALID_INDEX");
    return;
  }

  push_subjects_intervals_[index] = 0;

  //let's send some feedback
  sender->GetSerial()->print("ACK_STOP_PUSH ");
  sender->GetSerial()->println(index);
}

void push_data(int subject_index)
{
  switch(subject_index)
  {
    case kPushPingIndex:
    {
      //Sending
      //PING SEQ_NUMBER LOOP_TICKS_PER_PING_INTERVAL
      Serial.print("PING ");
      Serial.print(ping_sequence_++);
      Serial.print(" ");
      Serial.print(loop_ticks_);
      Serial.println();
      loop_ticks_ = 0;
      break;
    }
    case kPushAnalogIndex:
    {
      //let's send 3 analog results (A0-A2);
      //ANALOG VALUE;VALUE;VALUE
      Serial.print("ANALOG ");
      Serial.print(analogRead(A0));
      Serial.print(";");
      Serial.print(analogRead(A1));
      Serial.print(";");
      Serial.print(analogRead(A2));
      Serial.println();
      break;
    }
  }
}

SerialCommand cmd_read_analog_("READ_ANALOG", cmd_read_analog);
SerialCommand cmd_set_led_("SET_LED", cmd_set_led);
SerialCommand cmd_start_push_("START_PUSH", cmd_start_push);
SerialCommand cmd_stop_push_("STOP_PUSH", cmd_stop_push);

void setup() 
{
	Serial.begin(57600);

	pinMode(kRedLedPin, OUTPUT);
	pinMode(kGreenLedPin, OUTPUT);
	pinMode(kBlueLedPin, OUTPUT);

	set_led("red", 0);
	set_led("green", 0);
	set_led("blue", 0);

	serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmd_read_analog_);
	serial_commands_.AddCommand(&cmd_set_led_);
  serial_commands_.AddCommand(&cmd_start_push_);
  serial_commands_.AddCommand(&cmd_stop_push_);

	Serial.println("Ready!");
}

void loop() 
{
	serial_commands_.ReadSerial();

  for (int ix=0; ix<kMaxPushSubjects; ix++)
  {
    if (push_subjects_intervals_[ix]!=0 && millis()>=push_subjects_next_millis_[ix])
    {
      push_subjects_next_millis_[ix] += push_subjects_intervals_[ix];
      push_data(ix);
    } 
  }

  loop_ticks_++;
}



