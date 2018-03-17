/*--------------------------------------------------------------------
Author		: Pedro Pereira
License		: BSD
Hardware  : Arduino UNO
--------------------------------------------------------------------*/
#include <Arduino.h>
#include "SerialCommands.h"

//Using AltSoftSerial
/*
//https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
//Uno: 9 - TX, 8 - RX
#include <AltSoftSerial.h>
AltSoftSerial software_serial_;
*/

#include <SoftwareSerial.h>
SoftwareSerial software_serial_(8, 9); // RX, TX


char serial_command_buffer_[32];
SerialCommands serial_commands_(&software_serial_, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  //Print to the Uno's Serial port 
  Serial.print("Unrecognized command [");
  Serial.print(cmd);
  Serial.println("]");

  //Print to the Sender's Serial port 
	sender->GetSerial()->print("Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

void cmd_test_arguments(SerialCommands* sender)
{
  //Print to the Uno's Serial port 
  Serial.println("Received: Test");
  
  //Print to the Sender's Serial port 
  sender->GetSerial()->println("Received: Test");

  //Print the arguments
  char *arg;
  int ax = 0;
  do
  {
    arg = sender->Next();
    if (arg!=NULL)
    {
      //Print to the Uno's Serial port 
      Serial.print("...Arg: ");
      Serial.print(ax);
      Serial.print(" [");
      Serial.print(arg);
      Serial.println("]");

      //Print to the Sender's Serial port 
      sender->GetSerial()->print("...Arg: ");
      sender->GetSerial()->print(ax);
      sender->GetSerial()->print(" [");
      sender->GetSerial()->print(arg);
      sender->GetSerial()->println("]");

      ax++;
    }
  }
  while (arg != NULL);
}


SerialCommand cmd_test_arguments_("test", cmd_test_arguments);

void setup() 
{
	Serial.begin(57600);
  Serial.println("Ready!");

  software_serial_.begin(57600);
  software_serial_.println("Ready!");

	serial_commands_.SetDefaultHandler(cmd_unrecognized);
	serial_commands_.AddCommand(&cmd_test_arguments_);
}

void loop() 
{
	serial_commands_.ReadSerial();
}
