#include <iostream>
#include <stdio.h>
#include "global.h"

using namespace std;
int main(int argc, char *argv[])
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    double heading_angle = 0.0f;
    static int cnt = 0;

    description();

    bool ok = device_setup();

    if (ok == false)
    {
        return false;
    }
    else
    {
        PC.send_message("all device ready for start \n");
        std::cout << "all device ready for start " << std::endl << std::endl;
    }


    encoder.send_acknowlede();
    WithRobot::SensorData imu_data;

    char str_array[50]={0};
    while(true)
    {
        imu.cmd_trigger();
        if( imu.wait_data() == true )
        {
            //  Get a heading angle from IMU
            imu.get_data(imu_data);
            WithRobot::EulerAngle& e = imu_data.euler_angle;
            heading_angle = extraction_heading_angle(e.yaw);
        }

        if ( kalman.firstTime == false /*&& encoder.bReceived == true*/ )
        {
        //    printf("calculate odometry!\n");
            //  Calculate Wheel odometry by 4-mecanum wheel
       //   printf("%4d %4d %4d %4d \n",encoder.curr.left_top, encoder.curr.right_top, encoder.curr.left_bottom, encoder.curr.right_bottom);
       //   printf("%4d %4d %4d %4d \n",encoder.prev.left_top, encoder.prev.right_top, encoder.prev.left_bottom, encoder.prev.right_bottom);
      //    printf("%d th counter : %4d %4d %4d %4d \n\n\n",cnt++,encoder.diff.left_top, encoder.diff.right_top, encoder.diff.left_bottom, encoder.diff.right_bottom);
            AngularVelocity diff;
            diff.insert(0,1,1,1,1);
            kalman.predict(diff,heading_angle);
        //    kalman.predict(encoder.diff,heading_angle);
        //    printf("estimated x : %4.2f, y : %4.2f\n",kalman.state[0],kalman.state[1]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        yr.request_rssi();
        if ( yr.bRecognized == true /*&& yr.info.rssi > _REF_RSSI_*/ )
        {
            //  Calculate Kalman filter using wheel odometry and tr location
            kalman.update(yr.info.x,yr.info.y);
            //  printf("x : %d, y : %d, rssi : %d \n",yr.info.x,yr.info.y,yr.info.rssi);
            yr.bRecognized = false;
        }

        sprintf(str_array, "x : %4.2f y : %4.2f \n",kalman.state[0],kalman.state[1]);
        PC.send_message(str_array);


     //   printf("kx : %4.2f ky : %4.2f \n", kalman.state[0],kalman.state[1]);
       // printf("rx : %d, ry : %d, rssi : %d , kx : %4.2f ky : %4.2f heading : %f \n", yr.info.x,yr.info.y,yr.info.rssi,kalman.state[0],kalman.state[1],heading_angle);
        // printf("q[0][0] : %4d, q[1][1] : %4d p[0][0] : %4d, p[1][1] : %4d \n\n",kalman.q[0][0],kalman.q[1][1],kalman.p[0][0],kalman.p[1][1]);

         //  encoder.send_pose(1,kalman.state[0],kalman.state[1],(heading_angle*M_PI/2)*10);
    }

    return 0;
}
