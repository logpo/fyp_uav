
#include "EulerToQuaternion.h"
#include <math.h>
#include <stdio.h>
#include <iostream>
#define PI 3.14159

void e2q(float *euler_ang, float *y)
{
    
    
    double roll  = euler_ang[0] * 1;
    double pitch = euler_ang[1];
    double yaw   = euler_ang[2] - (PI / 2);
    
    double cosPhi_2   = cos(roll / 2.0);
    double sinPhi_2   = sin(roll / 2.0);
    double cosTheta_2 = cos(pitch / 2.0);
    double sinTheta_2 = sin(pitch / 2.0);
    double cosPsi_2   = cos(yaw / 2.0);
    double sinPsi_2   = sin(yaw / 2.0);

    /* operations executed in double to avoid loss of precision through
     * consecutive multiplications. Result stored as float.
     */
    y[0] = (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2) * -1;
    y[1] = (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2) * -1;
    y[2] = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
    y[3] = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
 

    return;
}

void q2e(float *quat, float *euler)
{

    double q0 = *(quat + 0);
    double q1 = *(quat + 1);
    double q2 = *(quat + 2);
    double q3 = *(quat + 3);
    

    euler[0] = atan2(2.0 * (q0 * q1 + q2 * q3),
            1.0 - 2.0 * (q1 * q1 + q2 * q2));
	euler[1] = asin(2.0 * (q0 * q2 - q3 * q1));
	euler[2] = atan2(2.0 * (q0 * q3 + q1 * q2),
            1.0 - 2.0 * (q2 * q2 + q3 * q3));

    euler[0] = euler[0] ;
    euler[1] = euler[1] * -1;
	euler[2] = (euler[2] - (PI / 2)) * -1;

            return;

}