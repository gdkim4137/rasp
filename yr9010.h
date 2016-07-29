#ifndef YR9010_H
#define YR9010_H

#include "sensor.h"

class Transponder
{
public:
    unsigned int x;
    unsigned int y;

public:
    int rssi;
};

class YR9010 : public Sensor
{
protected:
    std::string raw_packet;	//	not yet processed packet when data is not received compltely for meaningful packet

 //   void(Background_Service::*data_callback)(Transponder& tr);
    void* data_callback_context;

    //	std::string head_info;
    std::string location_tag;	//	which packet lead to location of transponder
    std::string header_packet;

public:
    Transponder info;
    bool bRecognized;

public:
    YR9010()
    {
        char locate[] = { 0x30, 0x00, 0x00 };
        for (char c : locate)
        {
            location_tag.push_back(c);
        }

        bRecognized = false;

    }
    void register_data_callback(/*void(Background_Service::*callback)(Transponder& tr)*/) {

     //   data_callback_context = callback_context;
     //   data_callback = callback;
    }

    void request_rssi()
    {
        //	only one
        char command[] = { 0xA0, 0x04, 0x01, 0x89, 0x01, 0xD1 };

        //	request 255 times
        //	char command[] = { 0xA0, 0x04, 0x01, 0x89, 0xff, 0xD3 };

        std::string command_string = command;
        send_message(command_string);

    }

public:
    void request_without_rssi()
    {
        char command[] = { 0xA0, 0x0F, 0x01, 0x68, 232 };
        std::string command_string = command;
        send_message(command_string);

        //std::cout << "check sum is " << (int)CheckSum((unsigned char*)command, 4) << std::endl;

    }
public:
    unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
    {
        unsigned char i, uSum = 0;
        for (i = 0; i<uBuffLen; i++)
        {
            uSum = uSum + uBuff[i];
        }
        uSum = (~uSum) + 1;
        return uSum;
    }
    void send_pose(std::string command)
  {
        send_message(command);
    }

protected:
    virtual void parser(std::string packet)
    {
        raw_packet += packet;
        auto pos_location = raw_packet.find(location_tag);
        if (pos_location != std::string::npos && raw_packet.size() > pos_location + 14)
        {
            //	+3 ~ +4 : location information of Transponder in World frame
            info.x = static_cast<int>(raw_packet[pos_location + 3]);
            info.y = static_cast<int>(raw_packet[pos_location + 4]);

            //	+14	:	Received Signal Strength Indication
            info.rssi = static_cast<int>(raw_packet[pos_location + 14]);
            raw_packet.erase(raw_packet.begin(), raw_packet.begin() + pos_location + 14);

            std::mutex mtx;
            mtx.lock();
            bRecognized = true;
            mtx.unlock();
        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        request_rssi();

    }
};

#endif // YR9010_H
