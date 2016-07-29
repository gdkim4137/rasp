#ifndef EKF_ALGORITHM_H
#define EKF_ALGORITHM_H


#include <math.h>
#include "encoder.h"

#define A 100           // distance of vehicle along x axis
#define B 100           // distance of vehicle along y axis
#define C 1/(A+B)
#define DIAMETER    160 // diameter of wheel(mm)

#define PULSE       4
#define GEAR_RATIO  20
#define pi 3.141592

#define INTERVAL    500 // unit (mm)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                 Must consider experimental value
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define R           50   //
#define Q           50   //
#define SLIP_XX     1.0f // ratio range [0.0, 1.0]
#define SLIP_YY     1.0f // ratio range [0.0, 1.0]
#define SLIP_XY     1.0f // ratio range [0.0, 1.0]
#define SLIP_YX     1.0f // ratio range [0.0, 1.0]

class EKF
{

public:
    //  x(mm), y(mm)
    double state[2];

public:
    //  convert coordinates from local to world for localization
    double coordinate_conversion_mat[4];

    double p[2][2]; //  error co-variance,  diag. mat.
    double q[2][2]; //  system co-variance  diag. mat.
    double r[2][2]; //  observation co-variance diag. mat.
    double k[2][2]; //  kalman gain

    double left_top;
    double right_top;
    double left_bottom;
    double right_bottom;

public:
    bool firstTime;

public:
    EKF()
    {
       firstTime = true;
       left_top = 0;
       right_top = 0;
       left_bottom=0;
       right_bottom=0;

       state[0] = 0;
       state[1] = 0;

       //   kalman filter co-variance
       p[0][0] = 0; p[0][1] = 0;
       p[1][0] = 0; p[1][1] = 0;

       //   system noise
       q[0][0] = Q; q[0][1] = 0;
       q[1][0] = 0; q[1][1] = Q;

       //   observation noise
       r[0][0] = R; r[0][1] = 0;
       r[1][0] = 0; r[1][1] = R;

       //   kalman gain
       k[0][0] = 0; k[0][1] = 0;
       k[1][0] = 0; k[1][1] = 0;
    }
public:
    void get_location(int &x, int &y)
    {
        std::cout << "x : " << state[0] << "y : " << state[1] <<std::endl;
        x = state[0]*10;  // up to scale 10^-4
        y = state[1]*10;  // up to scale 10^-4
    }

public:
    void predict(RelatedPosition diff, double heading)
    {
        static int cnt = 0;
        //  Local to global coordinate conversion mat, 2X2
        cacl_rot_mat(heading);

        //  Calculate linear velocity about each wheel using previous and current absolute encoder value
        calc_Distance(diff);

        //  Calculate moving distance about each direction
        double dx = 0.25*(left_top + left_bottom + right_top + right_bottom);
        double dy = 0.25*(-left_top + left_bottom + right_top - right_bottom);

        //  Calculate current location using previous location, moving distance and heading anlge
        state[0] = state[0] + (coordinate_conversion_mat[0]*dx + coordinate_conversion_mat[1]*dy) * SLIP_XX;
        state[1] = state[1] + (coordinate_conversion_mat[2]*dx + coordinate_conversion_mat[3]*dy) * SLIP_YY;

        //  Calculate Q paramter in Kalman filter
        //  Q is proportional to moving distance x,y
        q[0][0] = 10;//dx * SLIP_YX;
        q[1][1] = 10;//dy * SLIP_XY;

        //  Calculate error co-variance
        p[0][0] += q[0][0];
        p[1][1] += q[1][1];
    }

    void update(int x, int y)
    {

        // //////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  Location of transponder is restored to rectangular form
        //  thus, we must do that convert real world location( transponder interval is defined above to INTERVAL )
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////
        int location[] = { INTERVAL*x, INTERVAL*y};

        if ( firstTime == true )
        {
            printf("first tag is detected!\n");
            state[0] = location[0];
            state[1] = location[1];

            p[0][0]  =  R;
            p[1][1]  =  R;

            firstTime = false;
        }
        else
        {
            // //////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Calculate kalman gain
            //  k = P * inv(P+R), where R is diagonal matrix, so inv(r) is inverse of each diagonal elements
            //                    where P is also diag matrix
            // //////////////////////////////////////////////////////////////////////////////////////////////////////////
            k[0][0] = p[0][0] * (1/(p[0][0]+r[0][0]));
            k[1][1] = p[1][1] * (1/(p[1][1]+r[1][1]));

            // Uupdate pose of vehicle when UHF Transponder is detected by reader
            state[0] = state[0] + k[0][0]*(location[0]-state[0]);
            state[1] = state[1] + k[1][1]*(location[1]-state[1]);

            // Update error co-variance
            p[0][0] = p[0][0] - k[0][0]*p[0][0];
            p[1][1] = p[1][1] - k[1][1]*p[1][1];


        }
    }

protected:
    //  argument, heading : degree
    void cacl_rot_mat(double heading)
    {
        double rad = heading * M_PI / 180.0f;
        coordinate_conversion_mat[0] = std::cos(rad);
        coordinate_conversion_mat[1] = -std::sin(rad);
        coordinate_conversion_mat[2] = std::sin(rad);
        coordinate_conversion_mat[3] = std::cos(rad);
    }

public:
    void calc_Distance(RelatedPosition diff)
    {
       this->left_top      = ((diff.left_top)        *2*pi / (PULSE*GEAR_RATIO))  * (DIAMETER/2);
       this->left_bottom   = ((diff.left_bottom)     *2*pi / (PULSE*GEAR_RATIO))  * (DIAMETER/2);
       this->right_top     = ((diff.right_top)       *2*pi / (PULSE*GEAR_RATIO))  * (DIAMETER/2);
       this->right_bottom  = ((diff.right_bottom)    *2*pi / (PULSE*GEAR_RATIO))  * (DIAMETER/2);
    }
};


#endif // EKF_ALGORITHM_H
