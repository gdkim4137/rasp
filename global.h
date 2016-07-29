#ifndef GLOBAL_H
#define GLOBAL_H

#include <chrono>
#include <serial.h>
#include "ekf_algorithm.h"
#include "yr9010.h"
#include "myahrs_plus.hpp"
#include "encoder.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                 Must consider experimental value
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define _REF_ANGLE_ 186.0f   //  adjust y-axis coordinates
#define _REF_RSSI_  50.0f   //  cut off distance 50mm
#define _SLIP_X_
#define _SLIP_Y_

YR9010 yr;
WithRobot::MyAhrsPlus imu;
Encoder encoder;
EKF kalman;
Encoder PC;



void description(void)
{
    std::cout << "This app is for MOBILE INDUSTRY CLUSTER" << std::endl
              << "version : based on console command " << std::endl
              << "latest  :  2016/06/28(Tue)" <<std::endl;
}
double extraction_heading_angle(double yaw)
{
    double real = yaw;

   // printf("yaw : %3.2f \n",yaw);

    // -180 ~ 180 => 0 ~ 360 conversion
    if ( real < 0 ) real= real+360;

    // convert left hand to right hand coordinates
    // CCW : positive(+), CW : neagtive(-)
    real = 360 - real;
   // printf("yaw : %3.2f \n",real);

    double gap = _REF_ANGLE_ -90;
    double heading = real-gap;

    if( gap > 0 )
    {
        //  Case 1 : Leading Phase ( ref > 90 degree )
        if ( 0 < real && real <= gap )
        {
            // heading will be replaced to 360-gap ~ 0
            heading = real - gap + 360;
        }
    }
    else if( gap < 0 )
    {
        // Case 2 : Lagging case ( ref < 90 degree )
        if ( 360 + gap < real && real <= 360 )
        {
            heading = real - gap - 360;
        }
    }
    return heading;
}

bool device_setup()
{
    bool check = true;


    std::cout <<std::endl << "Devices are setting..." << std::endl;

    //  Encoder
    if(encoder.start("/dev/ttyAMA0",115200)==false)
    {
        std::cout << "Can not access the encoder"<<std::endl;
        check = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));



    //  IMU, myAHRS+
    //  COM3
    if(imu.start("/dev/ttyACM0",115200)==false)
    {
        std::cout <<"Can not access the myAHRS+"<<std::endl;
        check = false;
    }
    imu.cmd_ascii_data_format("RPY");
    imu.cmd_divider(1);
    imu.cmd_mode("AT");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));



    //  UHF RFID, YR9010
    //  COM4
    if(yr.start("/dev/ttyUSB0",115200) == false)
    {
        std::cout << "Can not access the RFID Reader"<<std::endl;
        check = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(PC.start("/dev/ttyUSB1",115200) == false)
    {
        std::cout << "Can not access the Serial port for showing location"<<std::endl;
        check = false;
    }



    std::cout << std::endl;
    return check;
}



#endif // GLOBAL_H
