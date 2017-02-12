#include "helper_functions.h"

#define PI 3.14159265



bool check_cmd(float *dist){
  //Function checks if the target given by the command is within the FOV.
  int fov_h = 34; //Pixy FOV values in degrees Exact 33.83
  int fov_v = 23; //In Half of the angle: Exact 22.73
  float hor_angle, vert_angle;

  if (*dist>3) {
    //Check for out of distance range
    return false;
  }

  vert_angle = abs( atan ( *(dist + 2) / (*dist)) * 180 / PI);

  if (vert_angle>fov_v) {
    //Check for out of vertical FOV
    std::cout << "FOV V:"<< vert_angle << std::endl;
    return false;
  }

  hor_angle = abs(atan ( *(dist + 1) / (*dist)) * 180 / PI);

  if (hor_angle>fov_h) {
    //Check for out of horizontal FOV
    std::cout << "FOV H: " << hor_angle <<std::endl;
    return false;
  }

  //Otherwise return true
  return true;
}
