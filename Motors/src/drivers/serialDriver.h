#ifndef serialDriver_h
#define serialDriver_h

using namespace std;

class serialDriver{
	public:
		serialDriver(void);
		~serialDriver();
		int initializeSerialPort(char port[]); //initialize the serial port
		int writeSerial(char *data, unsigned int size); //write data to the port
		int readSerial(char* data); //read data from the port
		int readSerial(char* data, int len);

//variables
		int fd_serial; //a system assigned int that identifies the port
		const char* sp; //name of the serial port to use
};//end class

#endif
