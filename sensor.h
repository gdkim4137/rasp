#ifndef SENSOR_H
#define SENSOR_H

#include "serial.h"
#include<thread>
#include<mutex>
#include<chrono>
#include<deque>

class Sensor
{
public:
    Serial::SerialPort serial;

    std::thread thread_receiver;
    std::mutex mutex_communication;

protected:
    //	is operating thread?
    bool activate_thread_receiver;
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
        thread_receiver.detach();
    }

protected:
    virtual void parser(std::string packet) = 0;

protected:
    static void thread_proc_receiver(void* arg)
    {
        ((Sensor*)arg)->proc_receiver();
    }
    void proc_receiver()
    {
        int len;
        unsigned char buffer[1024];
        while (activate_thread_receiver)
        {
            memset(buffer, 0, sizeof(buffer) - 1);
            len = serial.Read(buffer, sizeof(buffer) - 1);
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
public:
    void send_message(std::string command)
    {

        std::lock_guard<std::mutex> _l(mutex_communication);
        if (serial.Write((unsigned char*)command.c_str(), (int)command.length()) <= 0)
        {
        }
    }

};

#endif // SENSOR_H
