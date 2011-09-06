/******************************************************************************************
* The purpose of this class is to handle interactions with the serial ports. It has nothing
*   special in it. Is is simply a serail port driver. It allows other code to read and
*   write to the serial port
*
*   Author: Daniel Peterson
*   Version 1.0 11/23/09
********************************************************************************************/
#include "serialDriver.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include <ros/ros.h>

using namespace std;

/********************************************************************************************
* Function Name: serialDriver
* Purpose: constructor for this class
* Arguments: none
* Returns: nothing
*********************************************************************************************/
serialDriver::serialDriver(void){

}//==============================================
/********************************************************************************************
* Function Name: ~serialDriver
* Purpose: destructor for this class
* Arguments: none
* Returns: nothing
*********************************************************************************************/
serialDriver::~serialDriver(void){}

/********************************************************************************************
* Function Name: initialize
* Purpose: does the actual initialization of the port
* Arguments: port - the port to initialize
* Returns: 1 on success
*********************************************************************************************/
int serialDriver::initializeSerialPort(char port[]){
    struct termios tty;
    struct termios oldtty;

try{
    const char* p = port;
    fd_serial = open(p, O_RDWR | O_NOCTTY | O_NONBLOCK); //try to open the serial port

    if (fd_serial < 0) { //if opening the port was unsuccessful, print the error to the console line and exit
        ROS_ERROR("Error opening the serial port"); //say where the error occured
        perror(port); //print the error
        throw port;
    }
    else
    {
        ROS_INFO("Successfully opened the serial port");


        tcgetattr(fd_serial, &oldtty); //save the previous port settings so the computer doesn't complain
        bzero(&tty, sizeof(tty)); //make sure the new structure is actually clear
        //lets set the new port settings
        /**
        * B19200 = Baud 19200
        * CS8 = 8N1
        * CLOCAL = no modem control. We control the port directly
        * CREAD = enable recieving characters
        **/
        tty.c_cflag = B19200 |CS8 | CLOCAL | CREAD;
        /**
        * ICRNL = map a carriage return to the new line character so when a return is sent, a new line is too
        **/
        tty.c_iflag = ICRNL;
        tty.c_oflag = 0; //set output to raw only
        tty.c_lflag = ICANON; //disable echo

        int err = tcflush(fd_serial, TCIFLUSH); //flush the old settings
        if (err < 0) ROS_ERROR("Error flushing old settings");

        err = tcsetattr(fd_serial, TCSANOW, &tty); //apply the new settings to the port
        if (err < 0) ROS_ERROR("Error applying new serial port settings");

        ROS_INFO("Serial Port Ready: %d", fd_serial);
        return 1;
    }
} //end try
catch(...) {
    ROS_ERROR("There was an error initializing the port");
    return -1;
}// end catch
}//=====================================================
/********************************************************************************************
* Function Name: readSerial
* Purpose: reads a byte from the serail port
* Arguments: data - a pointer to a character array to write to in memory
* Returns: nothing
*********************************************************************************************/
int serialDriver::readSerial(char* data){
try{
    short int t = 0;
    while (t < 1){
        t = read(fd_serial, data, 8);
        //cout << "t = " << t << endl;
        //usleep (1000);
    }
    if (t < 0) return -1;
    return 1;
}
catch(...){ROS_ERROR("Error reading port"); return -1;}
}//======================================================
/********************************************************************************************
* Function Name: readSerial
* Purpose: reads a byte from the serail port
* Arguments: data - a pointer to a character array to write to in memory
* Returns: nothing
*********************************************************************************************/
int serialDriver::readSerial(char* data, int len){
try{
    short int t = 0;
    while (t < 1){
        t = read(fd_serial, data, len);
        //cout << "t = " << t << endl;
        //usleep (1000);
    }
    if (t < 0) return -1;
    return 1;
}
catch(...){ROS_ERROR("Error reading port"); return -1;}
}//======================================================
/********************************************************************************************
* Function Name: writeSerial
* Purpose: writes a byte
* Arguments: data - the byte of data to write
* Returns: 1 on success
*********************************************************************************************/
int serialDriver::writeSerial(char *data, unsigned int size ){
try{
    int err = write(fd_serial, data, size);
    if (err < 0) {ROS_ERROR("Error writing to the port"); return -1;}
    return 1;
}
catch(...){ROS_ERROR("Error writing: %s", strerror(errno)); return -1;}
}//=======================================================
