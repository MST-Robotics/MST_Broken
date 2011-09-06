#ifndef motorController_h_
#define motorController_h_

#include "serialDriver.h"
#include <iostream>

using namespace std;

class motorController {
	public:
//variables
		static const char Default_mode = '1';
		static unsigned int numOfMotors;//number of motor controllers that has been created
		bool ms; //the state the motor is currently in(0 off 1 on)	
		char mode;
		serialDriver sp; //serial port to use
		unsigned int id; //stores the id of the motor controller
//methods
		motorController(char p[]);
		~motorController();
		int setMode(char mo);
		int setMode(int mo);
		int setMode(string mo);
		int setVelocity(double v);
		int setTorque(float a);
		int toggleMotor(bool state);
		int setPosition(long int pos);
		int getPosition();
		int setEncoder(int count);
		unsigned int getID();
		bool operator==(motorController m);
		int stopMotor();
		int getSerialData();
}; //end of class


#endif
