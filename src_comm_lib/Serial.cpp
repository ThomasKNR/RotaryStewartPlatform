/* <Soure file for library used for communication with
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
#include "SerialClass.h"

//action numbers
#define SETBACKOFF 48
#define SETBACKON 49
#define SETPOSITIONS 50
#define PRINTPOS 51
#define STOPPRINTPOS 52
#define SWITCHIRDA 53
#define SETPOSITIONSINMS 54
#define SWITCHIRDAOFF 55
#define GEPOSITION 56


Platform::Platform(int portno){
#ifdef _WIN32
	if (!serial.Open(portno, 9600)){
        error=2;
        fprintf(stderr,"COM communication establish error\n");
    }
    //sleep for 2.5s in case Arduino restarts
	Sleep(2500);
#elif defined __linux__
    this->arduino = fopen("/dev/ttyACM0","r+b");
    if(this->arduino==NULL){
        this->arduino = fopen("/dev/ttyACM1","r+b");
        if(this->arduino==NULL){
            this->error=2;
            fprintf(stderr,"Path to Arduino is bad\n");
        }
    }
    sleep(3);
#endif
}

bool Platform::writeAction(unsigned char c){
#ifdef _WIN32
    if(serial.IsOpened()){
        if(!serial.SendData(&c,1)){
            error=1;
            return false;
        }
		return true;
    }else{
        error=2;
        return false;
    }
#elif defined __linux__
    if(this->arduino==NULL){
        this->error=2;
        return false;
    }else{
        if(fwrite(&c,sizeof(char), 1, this->arduino)<1){
            this->error=1;
            return false;
        }
        fflush(this->arduino);
        return true;
    }
#endif
}

bool Platform::setBacklightOff(){
    char val=SETBACKOFF;
    return(this->writeAction(val));
}
bool Platform::setBacklightOn(){
    char val=SETBACKON;
    return(this->writeAction(val));
}

bool Platform::printPositions(){
    char val=PRINTPOS;
    return(this->writeAction(val));
}

bool Platform::irDaOn(){
    char val=SWITCHIRDAOFF;
    return(this->writeAction(val));
}
bool Platform::irDaOff(){
    char val=SWITCHIRDA;
    return(this->writeAction(val));
}
bool Platform::endPrintPositions(){
    char val=STOPPRINTPOS;
    return(this->writeAction(val));
}
bool Platform::setPositionsInMs(int *values){
    char val=SWITCHIRDA;
    if(!this->writeAction(val)){
        return false;
    }
#ifdef _WIN32
    if(serial.IsOpened()){
        for(int i=0;i<6;i++){
            int32_t vall=(int32_t)(100*values[i]);
            unsigned char arr[4]={(vall),(vall>>8),(vall>>16),(vall>>24) };
            if(!this->serial.SendData(arr,4)){
				error=1;
                return false;
            }
        }
		return true;
    }else{
        error=2;
        return false;
    }
#elif defined __linux__
    for(int i=0;i<6;i++){
        int32_t vall=(int32_t)(values[i]);
        if(fwrite(&vall,sizeof(int32_t), 1, this->arduino)<sizeof(int32_t)){
            this->error=1;
            return false;
        }
        fflush(this->arduino);
    }
    return true;
#endif
}
int Platform::setPositions(float * values){
    char val=SETPOSITIONS;
    if(!this->writeAction(val)){
        return false;
    }
#ifdef _WIN32
	if (this->serial.IsOpened()){
		for (int i = 0; i < 6; i++){
			int32_t vall = (int32_t)(100 * values[i]);
			unsigned char arr[4] = { (vall & 255), (vall >> 8) & 255, (vall >> 16) & 255, (vall >> 24) & 255 };
			if (!this->serial.SendData(arr, 4)){
				error = 1;
				return -1;
			}
		}
		char vall;
		byte arr[1];
		if ((vall = this->serial.ReadData(arr, 1)) < 1){
			error = 1;
			return -1;
		}
		return (int)arr[0];
    }else{
        error=2;
        return -1;
    }
#elif defined __linux__
    for(int i=0;i<6;i++){
        int32_t val=(int32_t)(100*values[i]);
        int lol;
        if((lol=fwrite(&val,sizeof(int32_t), 1, this->arduino))<1) {
            this->error=1;
            return -1;
        }
    }
    fflush(this->arduino);
    char vall;
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 230000000;
    nanosleep(&tim , &tim2);
    if(fread(&vall,1,1,this->arduino)<1){
        this->error=1;
        return -1;
    }
    return (int)vall;
#endif
}
bool Platform::getPositions(float * values){
        char val=GEPOSITION;
    if(!this->writeAction(val)){
        return false;
    }
#ifdef _WIN32
    if(this->serial.IsOpened()){
        for(int i=0;i<6;i++){
            int32_t vall;
            byte arr[4];
            if((vall=this->serial.ReadData(arr,4))<4){
                error=1;
                return false;
            }
			vall = arr[3];
            vall=(vall<<8)|arr[2];
            vall=(vall<<8)|arr[1];
            vall=(vall<<8)|arr[0];
			values[i]=(float)(vall/100.00);
        }
		return true;
    }else{
        error=2;
        return false;
    }
#elif defined __linux__
            struct timespec tim, tim2;
            tim.tv_sec = 0;
            tim.tv_nsec = 150000000;
            nanosleep(&tim , &tim2);
        for(int i=0;i<6;i++){
            int32_t vall;
            if(fread(&vall,sizeof(int32_t),1,this->arduino)<1){
                this->error=1;
                return false;
            }
            values[i]=vall/100.00;

            fflush(this->arduino);
        }
        return true;
#endif
}


void Platform::endCommunication(){
#ifdef _WIN32
    this->serial.Close();
#elif defined __linux__
    if(this->arduino==NULL){
        this->error=1;
    }else{
        fflush(this->arduino);
        fclose(this->arduino);
    }
#endif
}
#ifdef _WIN32
CSerial::CSerial()
{

	memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
	memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));
	m_hIDComDev = NULL;
	m_bOpened = FALSE;

}

CSerial::~CSerial()
{

	Close();

}

BOOL CSerial::Open(int nPort, int nBaud)
{

	if (m_bOpened) return(TRUE);

	LPWSTR szPort = new TCHAR[10];
	LPWSTR szComParams = new TCHAR[50];
	DCB dcb;

	wsprintf(szPort, L"COM%d", nPort);
	m_hIDComDev = CreateFile(szPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
	if (m_hIDComDev == NULL) return(FALSE);

	memset(&m_OverlappedRead, 0, sizeof(OVERLAPPED));
	memset(&m_OverlappedWrite, 0, sizeof(OVERLAPPED));

	COMMTIMEOUTS CommTimeOuts;
	CommTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
	CommTimeOuts.WriteTotalTimeoutConstant = 5000;
	SetCommTimeouts(m_hIDComDev, &CommTimeOuts);

	wsprintf(szComParams, L"COM%d:%d,n,8,1", nPort, nBaud);

	m_OverlappedRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_OverlappedWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	dcb.DCBlength = sizeof(DCB);
	GetCommState(m_hIDComDev, &dcb);
	dcb.BaudRate = nBaud;
	dcb.ByteSize = 8;
	unsigned char ucSet;
	ucSet = (unsigned char)((FC_RTSCTS & FC_DTRDSR) != 0);
	ucSet = (unsigned char)((FC_RTSCTS & FC_RTSCTS) != 0);
	ucSet = (unsigned char)((FC_RTSCTS & FC_XONXOFF) != 0);
	if (!SetCommState(m_hIDComDev, &dcb) ||
		!SetupComm(m_hIDComDev, 10000, 10000) ||
		m_OverlappedRead.hEvent == NULL ||
		m_OverlappedWrite.hEvent == NULL){
		DWORD dwError = GetLastError();
		if (m_OverlappedRead.hEvent != NULL) CloseHandle(m_OverlappedRead.hEvent);
		if (m_OverlappedWrite.hEvent != NULL) CloseHandle(m_OverlappedWrite.hEvent);
		CloseHandle(m_hIDComDev);
		return(FALSE);
	}

	m_bOpened = TRUE;
	delete[] szComParams;
	delete[] szPort;
	return(m_bOpened);

}

BOOL CSerial::Close(void)
{

	if (!m_bOpened || m_hIDComDev == NULL) return(TRUE);

	if (m_OverlappedRead.hEvent != NULL) CloseHandle(m_OverlappedRead.hEvent);
	if (m_OverlappedWrite.hEvent != NULL) CloseHandle(m_OverlappedWrite.hEvent);
	CloseHandle(m_hIDComDev);
	m_bOpened = FALSE;
	m_hIDComDev = NULL;

	return(TRUE);

}

BOOL CSerial::WriteCommByte(unsigned char ucByte)
{
	BOOL bWriteStat;
	DWORD dwBytesWritten;

	bWriteStat = WriteFile(m_hIDComDev, (LPSTR)&ucByte, 1, &dwBytesWritten, &m_OverlappedWrite);
	if (!bWriteStat && (GetLastError() == ERROR_IO_PENDING)){
		if (WaitForSingleObject(m_OverlappedWrite.hEvent, 1000)) dwBytesWritten = 0;
		else{
			GetOverlappedResult(m_hIDComDev, &m_OverlappedWrite, &dwBytesWritten, FALSE);
			m_OverlappedWrite.Offset += dwBytesWritten;
		}
	}

	return(TRUE);

}

int CSerial::SendData(const unsigned char *buffer, int size)
{

	if (!m_bOpened || m_hIDComDev == NULL) return(0);

	DWORD dwBytesWritten = 0;
	int i;
	for (i = 0; i<size; i++){
		WriteCommByte(buffer[i]);
		dwBytesWritten++;
	}

	return((int)dwBytesWritten);

}

int CSerial::ReadDataWaiting(void)
{

	if (!m_bOpened || m_hIDComDev == NULL) return(0);

	DWORD dwErrorFlags;
	COMSTAT ComStat;

	ClearCommError(m_hIDComDev, &dwErrorFlags, &ComStat);

	return((int)ComStat.cbInQue);

}

int CSerial::ReadData(void *buffer, int limit)
{

	if (!m_bOpened || m_hIDComDev == NULL) return(0);

	BOOL bReadStatus;
	DWORD dwBytesRead, dwErrorFlags;
	COMSTAT ComStat;
	int counter = 1000;
	while (counter>0){
		counter--;
		ClearCommError(m_hIDComDev, &dwErrorFlags, &ComStat);
		if (ComStat.cbInQue >= limit) break;
		Sleep(10);
	}
	dwBytesRead = (DWORD)ComStat.cbInQue;
	if (limit < (int)dwBytesRead) dwBytesRead = (DWORD)limit;

	bReadStatus = ReadFile(m_hIDComDev, buffer, dwBytesRead, &dwBytesRead, &m_OverlappedRead);
	if (!bReadStatus){
		if (GetLastError() == ERROR_IO_PENDING){
			WaitForSingleObject(m_OverlappedRead.hEvent, 2000);
			return((int)dwBytesRead);
		}
		return(0);
	}

	return((int)dwBytesRead);

}
#endif
