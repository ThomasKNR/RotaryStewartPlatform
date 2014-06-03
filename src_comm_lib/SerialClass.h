/* <Header file for library for communication with 
 * Arduino Controlled Rotary Stewart Platform>
 * Copyright (C) <2014>  <Tomas Korgo>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.*/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <stdint.h>

//header files used on Linux
#ifdef __linux__
#include <unistd.h>
#include <time.h>
using namespace std;

//header files and defines used on Windows Platform 
#elif defined _WIN32
#include <windows.h>
#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04
#define ASCII_BEL       0x07
#define ASCII_BS        0x08
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D
#define ASCII_XON       0x11
#define ASCII_XOFF      0x13
/**
 * Helper class for serial communication on Windows Platform
*/
class CSerial
{

public:
	CSerial();
	~CSerial();
    //open port, communication speed
	BOOL Open(int nPort = 2, int nBaud = 9600);
    //close COM port
	BOOL Close(void);
    //functions for reading and writing
	int ReadData(void *, int);
	int SendData(const unsigned char *, int);
	int ReadDataWaiting(void);
	BOOL IsOpened(void){ return(m_bOpened); }

protected:
	BOOL WriteCommByte(unsigned char);
	HANDLE m_hIDComDev;
	OVERLAPPED m_OverlappedRead, m_OverlappedWrite;
	BOOL m_bOpened;

};
#endif

/*
 * Main class used for communication with Arduino via serial and for control
 * of Rotary Stewart Platform, all methods return bool indicating success of failure
 *
 * */
class Platform {

public:
    //Constructor, portno is number of COM port on which arduino is connected on Windows
    //platform. It can be found in Arduino IDE. On linux, path to arduino is used default
    //(/dev/ttyACM0 or /dev/ttyACM1)
	Platform(int portno);
    //method to turn off backlight of LCD
	bool setBacklightOff();
    //method to turn on backlight of LCD
	bool setBacklightOn();
    //main method to control position, parameter is array of 6 floats, these contain:
    //translation move in x, y, z axis, unit is mm
    //rotation move in x,y,z, axis, unit is ordinary degree
    //precision is to 1/100,
    //return: 0- everything OK
    //        >0 - desired position is outside of platform range, bigger values means bigger error
    //        -1 - error during communication with arduino
	int setPositions(float *);
    //starts showing current position of platform on LCD
	bool printPositions();
    //finishes showing current position of platform on LCD
	bool endPrintPositions();
    //enables control by irda
	bool irDaOn();
    //disables control by irda
	bool irDaOff();
    //reserved for future use
	bool setPositionsInMs(int *);
    //returns current positions
	bool getPositions(float *);
    //properly ends communication with Arduino
	void endCommunication();
private:
	bool writeAction(unsigned char c);
	FILE* arduino;
	int error;
#ifdef _WIN32
	CSerial serial;
#endif
};
