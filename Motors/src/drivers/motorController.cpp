/*******************************************************************************************
* This class handles using most of the simpleIQ commands to send and get data from the
*   elmo controllers. It will create a serial port instance to use for communicating
*
*
*   Author: Daniel Peterson (dpeterson309@yahoo.com) * Yahoo sucks *
*   Version: V1.0
*
********************************************************************************************/
#include "serialDriver.h"
#include "motorController.h"
#include <stdio.h>
#include <sstream>
#include <string.h>
#include <iostream>

#include <ros/ros.h>

using namespace std;

///////////////////////////////////////////convience methods//////////////////////////////////////////////
/*********************************************************************************************
* Function Name: int to string
* Purpose: convience method to convert an integer to a string
* Arguments: int num - the number to convert
* Returns: result - the number as a string
*********************************************************************************************/
string intToString (int num) {
    ostringstream t;
    t << num;
    return t.str();
}//======================================================
/*********************************************************************************************
* Function Name: doubleToChar
* Purpose: convience method to convert a float to a char array
* Arguments: float x - the number to convert
* Returns: result - the number as a char array
*********************************************************************************************/
void doubleToChar (double x, char *result, int size){
    ostringstream t; // string stream to use to convert
    t.precision(4);
    t << x; //conversion
    strncpy(result, t.str().c_str(), size); //get the resulting array
}//======================================================
/*********************************************************************************************
* Function Name: charToInt
* Purpose: convience method to convert a float to a char array
* Arguments: float x - the number to convert
* Returns: result - the number as a char array
*********************************************************************************************/
long int charToInt (char data[]){
    istringstream t(data); // string stream to use to convert
    //St.precision(4);
    int x = 0;
    t >> x; //conversion
    return x;
}//======================================================
////////////////////////end convience methods. Begin class code//////////////////////////////////////////

unsigned int motorController::numOfMotors;
/********************************************************************************************
* Function Name: motorController
* Purpose: constructor for this class
* Arguments: port - the port we will use to control the motor
* Returns: nothing
********************************************************************************************/
motorController::motorController(char p[]){
    ROS_INFO("Creating motor controller %s", p);
    numOfMotors++;// increment the number of motor controllers
    id=numOfMotors; //get the ID of this instance
    mode=Default_mode; //say we are in default mode
    sp.initializeSerialPort(p); //initialize the serial port
    char t[6] = "\rUM\r\r";
    short int err; //holds any errors that might come back from the serial port
    err = sp.writeSerial(t, strlen(t)-1);
    memset(t, '\r', 6);
    sp.writeSerial(t, strlen(t));
    char *buffer = new char[10];
    err = sp.readSerial(buffer); //try to read the serial port for the answer
    if (err < 0) ROS_ERROR("Error reading port!");
    if ((sizeof(buffer)/sizeof(char)) == 0){ROS_ERROR("Could not read serial port"); return;}
    ROS_INFO("%s", buffer);
    if (buffer[0] != '\0' && buffer[0] != Default_mode) { //if the harmonica is not in the default mode, make it the default mode
        t[3] = '='; //add an equal sign to um
        t[4] = Default_mode; //add the default mode
        t[5]='\r';
        err = sp.writeSerial(t, strlen(t)); //write the new mode
    }
    delete[] buffer;
}//======================================================
/********************************************************************************************
* Function Name: ~motorController
* Purpose: destructor for this class
* Arguments: none
* Returns: nothing
********************************************************************************************/
motorController::~motorController(){
    toggleMotor(0); //turn the motor off before destroying the port. This gaurentees that the motors will not be left to overheat and catch fire
    sp.~serialDriver(); //destroy the serial port as well
}//======================================================
/********************************************************************************************
* Function Name: setMode
* Purpose: used to set the mode for motor controller
* Arguments: short int mode - the number of the mode to set
* Returns: 1 on success
********************************************************************************************/
int motorController::setMode(char mo){
    if (mo > '5' || mo < '1') return -1; //return an error if we are out of range
    char t[5] = "UM=";
    t[3] = mo;
    t[4]='\r';
    mode = mo;
    short int err = sp.writeSerial(t, strlen(t));
    if (err != 1) return -1; //if there was an error tell the caller so
    return 1;
}//======================================================
/********************************************************************************************
* Function Name: setMode
* Purpose: used to set the mode for motor controller
* Arguments: string mode - a sting saying which mode to set
* Returns: 1 on success
********************************************************************************************/
int motorController::setMode(string mo){
try{
    char m[5] = "UM="; //will hold the mode to place
    if (mo == "torque" ) m[3] = '1';
    else if (mo == "speed") m[3] = '2';
    else if (mo == "stepper") m[3] = '3'; //only for use with AC motors
    else if (mo == "dual pos") m[3] = '4';
    else if (mo == "single pos") m[3] = '5';
    else return -1; //if none of the modes are correct, return an error
    mode = m[3];
    m[4]='\r';
    int err = sp.writeSerial(m, strlen(m));
    if (err != 1) return -1; //if there was an error tell the caller so
    return 1;
}
catch(...) {return -2;}
}//======================================================
/********************************************************************************************
* Function Name: setMode
* Purpose: used to set the mode for motor controller
* Arguments: int mode - a sting saying which mode to set
* Returns: 1 on success
********************************************************************************************/
int motorController::setMode(int mo){
try{
    char m[5] = "UM="; //will hold the mode to place
    switch (mo){
        case 1 : m[3] = '1'; break;
        case 2 : m[3] = '2'; break;
        case 3 : m[3] = '3'; break; //AC motors only
        case 4 : m[3] = '4'; break;
        case 5 : m[3] = '5'; break;
        default : return -1;
    }
    mode = m[3];
    m[4]='\r';
    int err = sp.writeSerial(m, strlen(m));
    if (err != 1) return -1; //if there was an error tell the caller so
    return 1;
}
catch(...) {return -2;}
}//======================================================
/*********************************************************************************************
* Function Name: setVelocity
* Purpose: sets the velocity of the controller
* Arguments: int v - the velocity
* Returns: 1 on success
* Precondition : the controller must be set to a mode that accepts velocity commands
*********************************************************************************************/
int motorController::setVelocity(double v){
    char command[16] = "JV=";
    if (ms == true && mode != 1){
        char temp[11];
        doubleToChar(v, temp, 11);
        strcat(command, temp);
        strcat(command, "\r");
        sp.writeSerial(command, strlen(command));
        strcpy(command,"BG\r");
        sp.writeSerial(command, strlen(command));
    }
    else if (ms == false) return -1;
    else if (mode == '1') return -2;
    return 1;
}//=====================================================
/*********************************************************************************************
* Function Name: setTorque
* Purpose: sets the spped of the controller using amperes
* Arguments: int a  - the amperage to set the motor to
* Returns: 1 on success
* Precondition : the controller must be set to a mode that accepts torque commandsi
*********************************************************************************************/
int motorController::setTorque(float a){
    if (ms == true && mode == '1'){
        char command[16] = "tc=";
        char temp[11];
        doubleToChar(a, temp, 11);
        strcat(command, temp);
        strcat(command, "\r");
        sp.writeSerial(command, strlen(command));
    }
    else if(ms == false) return -1;
    else if (mode != '1') return -2;
    return 1;
}//=====================================================
/*********************************************************************************************
* Function Name: toggleMotor
* Purpose: changes the state of the motor
* Arguments: state - the state to turn the motor to
* Returns: 1 on success
*********************************************************************************************/
int motorController::toggleMotor(bool state){
try{
    ms = state;
    char command[6] = "\rmo="; //array to store command
    if (state == true) command[4] = '1'; // select the state
    else if (state == false) command[4] = '0';
    command[5]='\r';
    short int err = sp.writeSerial(command, strlen(command)); //write the command
    if (err < 0) return -1; //if there is an error return it
    return 1;
}
catch(...){return -1;}
}//=====================================================
/*********************************************************************************************
* Function Name: setPosition
* Purpose: sets the position of the motor if the motor is in position mode
* Arguments: pos - the position to set the motor to
* Returns: 1 on success
*********************************************************************************************/
int motorController::setPosition(long int pos){
try{
    if (ms == 1 && mode == '3' || mode == '4' || mode == '5'){
        char command[16] = "pa="; //array to store command
        char temp[11];
        doubleToChar(pos, temp, 11);
        strcat(command, temp);
        strcat(command, "\r");
        short int err = sp.writeSerial(command, strlen(command)); //write the command
        if (err < 0) return -1; //if there is an error return it
        return 1;
    }
    else if(ms == 0) return -1;
    else return -2;
}
catch(...){return -1;}
}//===========================================
/*********************************************************************************************
* Function Name: getPosition
* Purpose: gets the position of the encoder
* Arguments: none
* Returns: the position of the encoder (null on error)
*********************************************************************************************/
int motorController::getPosition(){
    char temp[11]="\rpx\r";
    short int err = sp.writeSerial(temp, strlen(temp)); //write PX so the harmonica will return the encoder position
    if (err < 0) return NULL; //return NULL on error
    memset(temp, '\0', 10);
    return getSerialData();
}//==========================================
/*********************************************************************************************
* Function Name: setEncoder
* Purpose: tells the encoder to set its current posititon to the specified count
* Arguments: int count
* Returns: -1 on error
*********************************************************************************************/
int motorController::setEncoder(int count){
    char command[9]="\rpx=";
    char temp[4];
    doubleToChar(count, temp, 4);
    strcat(command, temp);
    strcat(command, "\r");
    short int err = sp.writeSerial(command, strlen(command));
    if (err < 0) return -1;
    return 1;
}//==========================================
/*********************************************************************************************
* Function Name: getID
* Purpose: returns the ID of the motorController to the caller
* Arguments: none
* Returns: the id of the motorController object
*********************************************************************************************/
unsigned int motorController::getID(){
    return id;
}//====================================================
/*********************************************************************************************
* Function Name: operator==
* Purpose: compares two motor controllers to see if they have the same ID
* Arguments: motorController m - the motor to compare to
* Returns: true if the controllers are equal
*********************************************************************************************/
bool motorController::operator==(motorController m){
    return (id == m.getID());
}//====================================================
/*********************************************************************************************
* Function Name: stopMotor
* Purpose: stops the motor by setting torque/velocity to 0
* Arguments: none
* Returns: 1 on success
*********************************************************************************************/
int motorController::stopMotor(){
    int err;
    if (mode == '1' && ms == true) {err = setTorque(0.0); return err;}
    else if ((mode > '1' || mode < '5') && ms == true) {err = setVelocity(0.0); return err;}
    else return -2;
}//====================================================
/*********************************************************************************************
* Function Name: getSerialData
* Purpose: used to get valid Elmo data from the serial port
* Arguments: none
* Returns: int num - the response from the Harmonica
*********************************************************************************************/
int motorController::getSerialData(){
    char temp[20];
    sp.readSerial(temp, 20);
    if (temp[0] >= '0' && temp[0] <= '9'){
        for (short unsigned int i = 1; i < 20; i++){
            if (temp[i] == ';') temp[i] = NULL;
        }
    }
    return charToInt(temp);
}//======================================================
