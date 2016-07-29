
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include<mutex>
#include <thread>

#define _USE_MATH_DEFINES
#include <math.h>

#include <exception>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <algorithm>

#ifdef WIN32
#define _AFXDLL
#include <afx.h>
#include <windows.h>
#include <cctype>

#pragma warning( disable : 4996 ) // sprintf
#pragma warning( disable : 4355 ) // warning C4355: 'this' : used in base member initializer list
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data

#define DBG_PRINTF(x, ...)  {if(x) { printf(__VA_ARGS__);}}

#else // LINUX
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#pragma GCC diagnostic ignored "-Wformat"
#define DBG_PRINTF(x, args...)  {if(x) { printf(args);}}
#endif // WIN32


/*
*  debugging sw
*/
#define DEBUG_PLATFORM        false
#define DEBUG_ASCII_PROTOCOL  false
#define DEBUG_BINARY_PROTOCOL false
#define DEBUG_MYAHRS_PLUS     false

#ifndef SERIAL_H
#define SERIAL_H


namespace Serial
{


class myAhrsException
{
public:
    std::string err;
    myAhrsException(std::string e = "") : err(e){}
    virtual ~myAhrsException() {}

    const char* what() const throw() {
        return err.c_str();
    }
};

/**********************************************************************************************************
*
* Platform abstraction
*
**********************************************************************************************************/

#ifdef WIN32

class SerialPort
{
    std::string port_name;
    unsigned int baudrate;

    HANDLE m_hIDComDev;
    BOOL m_bOpened;

public:
    SerialPort(const char* port = "COM3", unsigned int brate = 115200)
        : port_name(port), baudrate(brate)
        , m_hIDComDev(NULL), m_bOpened(FALSE)
    {
    }

    ~SerialPort() {
        Close();
    }

    bool Open(const char* port, int brate) {
        port_name = port;
        baudrate = brate;
        return Open();
    }

    bool Open() {
        char szPort[32];
        sprintf(szPort, "\\\\.\\%s", port_name.c_str());
        CString port_str = CString::CStringT(CA2CT(szPort));

        DBG_PRINTF(DEBUG_PLATFORM, "portname : %s, baudrate %d\n", szPort, baudrate);

        m_hIDComDev = CreateFile((LPCTSTR)port_str,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

        if (m_hIDComDev == NULL) {
            DBG_PRINTF(DEBUG_PLATFORM, "ERROR : m_hIDComDev == NULL\n");
            return false;
        }

        DCB dcb = { 0 };

        if (!GetCommState(m_hIDComDev, &dcb)) {
            //error getting state
            CloseHandle(m_hIDComDev);
            printf("ERROR : GetCommState()\n");
            return false;
        }

        dcb.DCBlength = sizeof(DCB);
        GetCommState(m_hIDComDev, &dcb);
        dcb.BaudRate = baudrate;
        dcb.ByteSize = 8;

        if (!SetCommState(m_hIDComDev, &dcb)){
            //error setting serial port state
            CloseHandle(m_hIDComDev);
            DBG_PRINTF(DEBUG_PLATFORM, "ERROR : SetCommState()\n");
            return false;
        }

        COMMTIMEOUTS CommTimeOuts;

        CommTimeOuts.ReadIntervalTimeout = 1;
        CommTimeOuts.ReadTotalTimeoutMultiplier = 1;
        CommTimeOuts.ReadTotalTimeoutConstant = 1;
        CommTimeOuts.WriteTotalTimeoutMultiplier = 1;
        CommTimeOuts.WriteTotalTimeoutConstant = 10;

        if (!SetCommTimeouts(m_hIDComDev, &CommTimeOuts)) {
            CloseHandle(m_hIDComDev);
            DBG_PRINTF(DEBUG_PLATFORM, "ERROR : SetCommTimeouts()\n");
            return false;
        }

        m_bOpened = TRUE;

        return (m_bOpened == TRUE);
    }

    void Close() {
        if (!m_bOpened || m_hIDComDev == NULL) {
            return;
        }

        CloseHandle(m_hIDComDev);
        m_bOpened = FALSE;
        m_hIDComDev = NULL;
    }

    int Read(unsigned char* buf, unsigned int buf_len) {
        if (!m_bOpened || m_hIDComDev == NULL) return -1;

        BOOL bReadStatus;
        DWORD dwBytesRead, dwErrorFlags;
        COMSTAT ComStat;

        ClearCommError(m_hIDComDev, &dwErrorFlags, &ComStat);

        if (ComStat.cbInQue <= 0) {
            dwBytesRead = 1;
        }
        else {
            dwBytesRead = (DWORD)ComStat.cbInQue;
        }

        if (buf_len < (int)dwBytesRead) dwBytesRead = (DWORD)buf_len;

        bReadStatus = ReadFile(m_hIDComDev, buf, dwBytesRead, &dwBytesRead, NULL);
        if (!bReadStatus){
            //if( GetLastError() == ERROR_IO_PENDING ){
            //    WaitForSingleObject( m_OverlappedRead.hEvent, 2000 );
            //    return( (int) dwBytesRead );
            //}
            return 0;
        }

        return((int)dwBytesRead);
    }

    int Write(unsigned char* data, unsigned int data_len) {
        BOOL bWriteStat;
        DWORD dwBytesWritten = 0;

        if (m_bOpened != TRUE) {
            return -1;
        }

        bWriteStat = WriteFile(m_hIDComDev, (LPSTR)data, data_len, &dwBytesWritten, NULL);
        if (!bWriteStat) {
            if (GetLastError() != ERROR_IO_PENDING) {
                // WriteFile failed, but it isn't delayed. Report error and abort.
                return dwBytesWritten;
            }
            else {
                // Write is pending...
                //Sleep(10);
                // retry;
                return dwBytesWritten;
            }
        }
        else {
            return dwBytesWritten;
        }
    }

    int Flush() {
        int len = 0, count = 0;
        unsigned char buf[256];
        while ((len = Read(buf, sizeof(buf))) > 0) {
            count += len;
        }
        return count;
    }
};


#else
/*
* Unix-Like OS (Linux/OSX)
*/

class SerialPort
{
    std::string port_name; // ex) "/dev/tty.usbmodem14241"
    int port_fd;
    unsigned int baudrate;

public:
    SerialPort(const char* port = "", unsigned int brate = 115200)
        : port_name(port), baudrate(brate)
        , port_fd(-1)
    {
    }

    ~SerialPort() {
        Close();
    }

    bool Open(const char* port, int brate) {
        port_name = port;
        baudrate = brate;
        return Open();
    }

    bool Open() {
        int fd = 0;
        struct termios options;

        if (port_fd > 0) {
            return true;
        }

        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            return false;
        }

        fcntl(fd, F_SETFL, 0);   // clear all flags on descriptor, enable direct I/O
        tcgetattr(fd, &options); // read serial port options

        cfsetspeed(&options, baudrate);
        cfmakeraw(&options);

        options.c_cflag |= CREAD | CLOCAL; // turn on READ
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;

        // set the new port options
        tcsetattr(fd, TCSANOW, &options);

        // flush I/O buffers
        tcflush(fd, TCIOFLUSH);

        port_fd = fd;

        return (fd > 0);

    }

    void Close() {
        if (port_fd > 0) {
            close(port_fd);
            port_fd = -1;
        }
    }

    int Read(unsigned char* buf, unsigned int buf_len) {
        if (port_fd < 0) {
            return -1;
        }

        int n = read(port_fd, buf, buf_len - 1);
        if (n > 0) {
            buf[n] = 0;
        }

        return n;
    }

    int Write(unsigned char* data, unsigned int data_len) {
        if (port_fd < 0) {
            return -1;
        }

        return write(port_fd, data, data_len);
    }

    int Flush() {
        if (port_fd < 0) {
            return -1;
        }

        // flush I/O buffers
        return tcflush(port_fd, TCIOFLUSH);
    }
};

#endif // #ifdef WIN32


class communicate_device
{
public:
    SerialPort serial;

    //	callback loop for receiving raw data from device
    std::thread thread_receiver;
    std::mutex  mutex_communication;	//	avoid dead-lock

protected:
    bool activate_thread_receiver;		//	receive loop is running?
    bool activate_thread_event;

public:
    bool start(std::string port_name, int baudrate = -1)
    {
        std::lock_guard<std::mutex> _l(mutex_communication);

        if (port_name == "" || baudrate < 0)
            return false;
        else
        {
            if (serial.Open(port_name.c_str(), baudrate) == false)
                return false;
        }

        serial.Flush();

        if (activate_thread_receiver == false)
        {
            activate_thread_receiver = true;
            thread_receiver = std::thread(thread_proc_receiver, this);
        }

        return true;
    }
    void stop()
    {
        std::lock_guard<std::mutex> _l(mutex_communication);
        //	receiver thread exit
        activate_thread_receiver = false;
        thread_receiver.join();
    //	thread_receiver.detach();
    }

protected:
    //	packet ╨п╪╝
    virtual void parser(std::string packet) = 0;

protected:

    static void thread_proc_receiver(void* arg)
    {
        ((communicate_device*)arg)->proc_receiver();
    }
    void proc_receiver()
    {
        int len;
        unsigned char buffer[1024];
        while (activate_thread_receiver)
        {
            memset(buffer, 0, sizeof(buffer)-1);
            len = serial.Read(buffer, sizeof(buffer)-1);
            if (len == 0)
            {
                //sleep(1);
                 std::this_thread::sleep_for(std::chrono::milliseconds(1));
                //Platform::msleep(1);
                continue;
            }
            else if (len > 0)
            {
                std::string received;

                for (int i = 0; i < len; ++i)
                    received.push_back(buffer[i]);

                this->parser(received);
            }
        }
    }
    void send_message(std::string command)
    {
        //	lockю╩╟и╟М ╨╦Ё©.
        std::lock_guard<std::mutex> _l(mutex_communication);
        if (serial.Write((unsigned char*)command.c_str(), (int)command.length()) <= 0)
        {
            //	╣П╧Ж╠в а╓╨╦....
        }
    }
};

}
#endif // !SERIAL_H
