//Node for computing control output of the system


// #include </opt/ros/indigo/include/ros/ros.h>
#include "ros/ros.h" //Include this for use on PI

#include "ControlLoop.cpp"
#include "Derivative.cpp"

#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>
#include <pixy_msgs/PixyDist.h>
#include <pixy_msgs/ControlOutput.h>
#include <pixy_msgs/Kalman_estimate.h>

#define MAX_RATE_OP 200

#define BLOCK_BUFFER_SIZE 100

using namespace std;

class TopControl
{
public:
  //void reControl(const pixy_msgs::PixyDist dist);
  void reControl(const pixy_msgs::Kalman_estimate dist);


  void spin();

private:
  //Variables for ROS use
  ros::NodeHandle t_node_handle_;

  ros::Rate rate_;

  ros::Publisher control_pub;
  ros::Subscriber distance_subscriber_;
  std::string frame_id;

  //Variables for ControlLoop
  ControlLoop pitchPosition;
	ControlLoop pitchVelocity;
	Derivative pitchV_estimate;

	ControlLoop rollPosition;
	ControlLoop rollVelocity;
	Derivative rollV_estimate;

	ControlLoop throttlePosition;
	ControlLoop throttleVelocity;
	Derivative throttleV_estimate;
  


public:
  TopControl():
    t_node_handle_(),
    rate_(50),
    pitchPosition(10,0,0), //P controller with kp=1
		pitchVelocity(5,0,0),
		pitchV_estimate(5),
		rollPosition(), //P controller with kp=1
		rollVelocity(5,0,0),
		rollV_estimate(5),
		throttlePosition(), //P controller with kp=1
		throttleVelocity(5,0,0),
		throttleV_estimate(5)
  {
   control_pub  = t_node_handle_.advertise<pixy_msgs::ControlOutput>("control_op", 50);
    //ros::Subscriber sub = t_node_handle_.subscribe("distance", 10 , &TopControl::reControl, this);
    

    pitchPosition.setKp(10);
  };

};

void TopControl::spin()
{
  while (t_node_handle_.ok()) {


		ros::spinOnce();

    rate_.sleep(); //Maybe not necessary
  }

}

//void TopControl::reControl(const pixy_msgs::PixyDist dist) {
void TopControl::reControl(const pixy_msgs::Kalman_estimate dist) {

  //Compute and publish the control output when a new dist estimate is reached

    pixy_msgs::ControlOutput output;
    float pitch_control_op, roll_control_op, throttle_control_op;
    //ros::Publisher control_pub  = t_node_handle_.advertise<pixy_msgs::ControlOutput>("control_op", 50);


    //Find Pitch pose command
    pitch_control_op = pitchPosition.computeOutput(2.0, (dist.x));
    //Find the pitch rate command
		output.pitch = pitchVelocity.computeOutput(pitch_control_op,dist.x_dot);

    // Section for limiting control output
      if (output.pitch > MAX_RATE_OP )
      {
        output.pitch = 200;
        ROS_INFO("Max rate OP: +ve Saturation reached");
      }

      if (output.pitch < (MAX_RATE_OP * -1))
      {
        output.pitch = -200;
        ROS_INFO("Max rate OP: -ve Saturation reached");

      }
       output.pitch = pitch_control_op;
      


      //Find roll pose command
    	//	roll_control_op = rollPosition.computeOutput(0, (dist.position.y));
      //Find the roll rate command
    	//	output.roll = rollVelocity.computeOutput(roll_control_op,rollV_estimate.computeDerivative((dist.position.y)));
      output.roll = 0;


      //Find Throttle pose command
    	//	throttle_control_op = throttlePosition.computeOutput(0, (dist.position.z));
      //Find the throttle rate command
      //  output.throttle = throttleVelocity.computeOutput(throttle_control_op,throttleV_estimate.computeDerivative((dist.position.z)));
      output.throttle = 0;


       // std::cout << dist.position.x <<std::endl;
      // std::cout << "Pitch report" << std::endl;
      // std::cout << pitch_control_op <<std::endl;
    

    //ROS_INFO(output.pitch);

    if (!control_pub) {
      ROS_WARN("INVALID PUB");
    } else {
      control_pub.publish(output);
    }
    return;
}

int main(int argc, char  **argv) {
  //Initiallise the NodeHandle
  ros::init(argc, argv, "quad_control_node");
  std::cout << "INIT" << std::endl;

  ros::NodeHandle my_node_handle_;
  TopControl myController;

  //ros::Subscriber sub = my_node_handle_.subscribe("distance", 10 , &TopControl::reControl, &myController);
  ros::Subscriber kalman_sub = my_node_handle_.subscribe("kalman_estimate", 10 , &TopControl::reControl, &myController);


  ros::spin();


  return 0;
}
