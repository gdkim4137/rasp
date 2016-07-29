#ifndef ENCODER_H
#define ENCODER_H

#include "sensor.h"

#define ERR_GAP 10

class AngularVelocity
{
public:
    AngularVelocity()
    {
        insert(0,0,0,0,0);
    }

    AngularVelocity(const AngularVelocity& av)
    {
        key_pose = av.key_pose;
        left_top = av.left_top;
        right_top = av.right_top;
        left_bottom = av.left_bottom;
        right_bottom = av.right_bottom;
    }
public:
    void operator=(AngularVelocity av)
    {
        key_pose = av.key_pose;
        left_top = av.left_top;
        right_top = av.right_top;
        left_bottom = av.left_bottom;
        right_bottom = av.right_bottom;
    }

    AngularVelocity operator-(AngularVelocity& av)
    {
        AngularVelocity diff;
        diff.insert(this->key_pose,this->left_top - av.left_top,
                                    this->right_top - av.right_top,
                                    this->left_bottom -av.left_bottom,
                                    this->right_bottom - av.right_bottom);
        return diff;
    }

    void insert(int kp, int lt, int lb, int rt, int rb)
    {
        key_pose = kp;
        left_top = lt;
        right_top = rt;
        left_bottom = lb;
        right_bottom = rb;
    }

public:
    int key_pose;

public:
    int left_top;
    int left_bottom;
    int right_top;
    int right_bottom;
};

typedef AngularVelocity LinearVelocity;
typedef AngularVelocity Distance;
typedef AngularVelocity RelatedPosition;

class Encoder : public Sensor
{

protected:
    std::string raw_packet;	//	not yet processed packet when data is not received compltely for meaningful packet

    //	std::string head_info;
    std::string header_packet;	//	which packet lead to location of transponder

public:
    AngularVelocity diff;
    AngularVelocity curr;
    AngularVelocity prev;

    bool bReceived;     //  packet is arrived from robot
    bool bAcknowledge;  //  chek if mysystem(Raspberry PI) is ready or not to start operate?
    bool isFirst;       //  check if First packet about encoder information is arrived?
    int  delay_count;

public:
    Encoder()
    {
        char locate[] = { 0x55, 0x55 };
        for (char c : locate)
        {
            header_packet.push_back(c);
        }
        bReceived = false;
        isFirst = true;
        bAcknowledge = false;
    }

public:
    void send_pose(std::string command)
    {
        send_message(command);
    }

    //  this x,y,w
    void send_acknowlede()
    {
        char command[20] = {0};
        std::string command_string;

        //  head(A)
        command[0] = 0x55;
        command[1] = 0x55;

        //  key pose(STRT)
        command[2] = 'S';
        command[3] = 'T';
        command[4] = 'R';
        command[5] = 'T';

        //  tail(Z)
        command[18] = 0x5a;
        command[19] = 0x5a;

        for (int i = 0 ; i < sizeof(command) ; ++i)
            command_string += command[i];

        send_pose(command_string);

    }

    void send_pose(int key_pose, int x, int y, int w)
    {
        char command[20] = {0};
        std::string command_string;

        //  head
        command[0] = 0x55;
        command[1] = 0x55;

        //  key pose
        command[2] = (key_pose >> 24) & 0x000000ff;
        command[3] = (key_pose >> 16) & 0x000000ff;
        command[4] = (key_pose >> 8) & 0x000000ff;
        command[5] = (key_pose) & 0x000000ff;

        //  x(mm)
        command[6] = (x >> 24) & 0x000000ff;
        command[7] = (x >> 16) & 0x000000ff;
        command[8] = (x >> 8) & 0x000000ff;
        command[9] = (x) & 0x000000ff;

        //  y(mm)
        command[10] = (y >> 24) & 0x000000ff;
        command[11] = (y >> 16) & 0x000000ff;
        command[12] = (y >> 8) & 0x000000ff;
        command[13] = (y) & 0x000000ff;

        //  w(rad*10)
        command[14] = (w >> 24) & 0x000000ff;
        command[15] = (w >> 16) & 0x000000ff;
        command[16] = (w >> 8) & 0x000000ff;
        command[17] = (w) & 0x000000ff;

        //  tail
        command[18] = 0x5a;
        command[19] = 0x5a;


        for (int i = 0 ; i < sizeof(command) ; ++i)
            command_string += command[i];

        send_pose(command_string);

    }

protected:
    virtual void parser(std::string packet)
    {

        static int cnt = 0;
        //	receiving data from motor encoder
        raw_packet += packet;

        auto pos_encoder = raw_packet.find(header_packet);

        if(  pos_encoder == std::string::npos)
            return;

        if( raw_packet.length() > pos_encoder + 23 )
        {

            //  std::cout << "loop check(start)"<<std::endl;
            //  key pose
            int key_pose  = (raw_packet[pos_encoder + 2] << 24)  |
                             (raw_packet[pos_encoder + 3] << 16) |
                             (raw_packet[pos_encoder + 4] << 8)  |
                             (raw_packet[pos_encoder + 5]);

            //  left top wheel encoder
            int left_top  = (raw_packet[pos_encoder + 6] << 24)  |
                             (raw_packet[pos_encoder + 7] << 16) |
                             (raw_packet[pos_encoder + 8] << 8)  |
                             (raw_packet[pos_encoder + 9]);

            //  right top wheel encoder
            int right_top = (raw_packet[pos_encoder + 10] << 24)  |
                             (raw_packet[pos_encoder + 11] << 16) |
                             (raw_packet[pos_encoder + 12] << 8)  |
                             (raw_packet[pos_encoder + 13]);

            //  left bottom wheel encoder
            int left_bottom  = (raw_packet[pos_encoder + 14] << 24)  |
                                (raw_packet[pos_encoder + 15] << 16) |
                                (raw_packet[pos_encoder + 16] << 8)  |
                                (raw_packet[pos_encoder + 17]);

            //  right bottom wheel encoder
            int right_bottom = (raw_packet[pos_encoder + 18] << 24) |
                                (raw_packet[pos_encoder + 19] << 16) |
                                (raw_packet[pos_encoder + 20] << 8)  |
                                (raw_packet[pos_encoder + 21]);



            if( delay_count < 5 )
            {
                delay_count++;
            }
            else
            {
                delay_count = 0;

                if(isFirst == true)
                {
                    if(left_top < ERR_GAP*10 && right_top < ERR_GAP*10 && left_bottom < ERR_GAP*10 && right_bottom < ERR_GAP*10)
                    {
                        curr.insert(key_pose,left_top,left_bottom,-1*right_top,-1*right_bottom);
                        prev = curr;
                        isFirst = false;
                    }
               //     std::cout <<"first run "<< curr.left_top <<"  "<<curr.right_top <<"  " << curr.left_bottom<<"  "<< curr.right_bottom << std::endl;
                }
                else
                {
                    curr.insert(key_pose,left_top,left_bottom,-1*right_top,-1*right_bottom);
                    diff = curr - prev;

                    if(abs(diff.left_top) > 100 )
                    {
                        diff.left_top = 0;
                        curr.left_top = prev.left_top;
                    }
                    if(abs(diff.right_top) > 100 )
                    {
                        diff.right_top = 0;
                        curr.right_top = prev.right_top;
                    }
                    if(abs(diff.left_bottom) > 100 )
                    {
                        diff.left_bottom = 0;
                        curr.left_bottom = prev.left_bottom;
                    }
                    if(abs(diff.right_bottom) > 100 )
                    {
                        diff.right_bottom = 0;
                        curr.right_bottom = prev.right_bottom;
                    }

               //     printf("%4d %4d %4d %4d \n",curr.left_top, curr.right_top, curr.left_bottom, curr.right_bottom);
               //     printf("%4d %4d %4d %4d \n",prev.left_top, prev.right_top, prev.left_bottom, prev.right_bottom);
               //     printf("%dth item %4d %4d %4d %4d \n\n\n",cnt++,diff.left_top, diff.right_top,diff.left_bottom, diff.right_bottom);
                    prev = curr;
                }

                if(key_pose !=0)
                {
                    std::mutex mtx;
                    mtx.lock();
                    bReceived = true;
                    mtx.unlock();
                }


            }


            //  erase buffer
            raw_packet.erase(raw_packet.begin(), raw_packet.begin()+pos_encoder+24);
        }

    }
};
#endif // ENCODER_H
