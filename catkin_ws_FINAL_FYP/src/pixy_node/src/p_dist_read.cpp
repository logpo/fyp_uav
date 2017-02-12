//Functions for finding distances

#include "p_dist_read.h"
#include <iostream>
using namespace std;

//The logic to find the information on distances

//Function takes the spixy data, with 1 indicating left target and 2 is right
float * getDistances (uint32_t pixel_height1, uint32_t pixel_width1, uint32_t pix_pos_h1, uint32_t pix_pos_v1, uint32_t pixel_height2, uint32_t pixel_width2,  uint32_t pix_pos_h2, uint32_t pix_pos_v2){
  //Setup of targets assuming identical size
  //float target_x = 0.15;// Hor dist m
  float target_y = 0.15;// Vert dist m
  float focalL = 2.9; //mm//TODO Check units and values
  float pixel_h_avg, ratio, avg_pix_pos_h, dist_y, avg_pix_pos_v, dist_z;

  static float dist[3]; //Array in X,Y,Z coordinate, giving distance to target wrt to quad center

  //Center the image frame coordinate at 0,0
  int pix_position_v1 = pix_pos_v1 - 99;
  int pix_position_h1 = pix_pos_h1 - 159;
  int pix_position_v2 = pix_pos_v2 - 99;
  int pix_position_h2 = pix_pos_h2 - 159;

  //Error Checks
  if (pixel_height1>200) {
    pixel_height1 = pixel_height2;
  }
  if (pixel_height2>200) {
    pixel_height2 = pixel_height1;
  }

  //Convert to distance (X or Depth)
	//CC distance

  pixel_h_avg = (pix_position_h2 - pix_position_h1) / 2; //TODO Add check for bad values
  float dist_x = (target_y * focalL * 0.2484) / ( pixel_h_avg * 0.003);// TODO Verify working

  ratio = target_y/(pixel_h_avg); //Ratio in m/pix


  //Convert to Lateral distance (Y)
  avg_pix_pos_h = (pix_position_h1 + pix_position_h2) /2;
  dist_y = avg_pix_pos_h * ratio;

  // Convert to height (Z)
  avg_pix_pos_v = (pix_position_v1 + pix_position_v2) /2;
  dist_z = avg_pix_pos_v * ratio;

  dist[0] = dist_x;
  dist[1] = dist_y;
  dist[2] = dist_z;


  if (dist_x<0) {
    std::cout << "Dist error: "<<dist_x << std::endl;
    std::cout << "H1 = " << pixel_height1 << " H2 = "<< pixel_height2 << std::endl;
    std::cout << "Pixeh H avg "<<pixel_h_avg << std::endl;
    std::cout << "Dist y and z" << dist_y<< " "<<dist_z << std::endl;
    }

  return dist;
}
