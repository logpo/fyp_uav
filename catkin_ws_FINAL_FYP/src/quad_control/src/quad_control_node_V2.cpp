//Node for computing control output of the system


// #include </opt/ros/indigo/include/ros/ros.h>
#include "ros/ros.h" //Include this for use on PI
#include <ros/package.h>

#include "ControlLoop.cpp"
#include "Derivative.cpp"
#include "EulerToQuaternion.cpp"
#include <tf/transform_datatypes.h>


#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>
#include <pixy_msgs/PixyDist.h>
#include <pixy_msgs/ControlOutput.h>
#include <pixy_msgs/Kalman_estimate.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>

#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3

#define MAX_RATE_OP 200
#define MAX_ANG_DEG 5
#define PI 3.14159

#define BLOCK_BUFFER_SIZE 100

using namespace std;

class TopControl
{
public:
  //void reControl(const pixy_msgs::PixyDist dist);
  void reControl(const pixy_msgs::Kalman_estimate dist);
  void update_IMU(const sensor_msgs::Imu::ConstPtr& data);
  void stick_input(const mavros_msgs::RCIn::ConstPtr& msg);

  float rc_roll;
  float rc_pitch;
  float rc_yaw;


  void spin();

private:
  //Variables for ROS use
  ros::NodeHandle t_node_handle_;

  ros::Rate rate_;

  ros::Publisher control_pub;
  ros::Publisher quat_angle;
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

  double current_roll, current_pitch, current_yaw;
 


public:
  TopControl():
	t_node_handle_(),
	rate_(50),
	pitchPosition(10,0,0), //P controller with kp=1
   	 pitchVelocity(0,0,0),
   	 pitchV_estimate(5),
   	 rollPosition(), //P controller with kp=1
   	 rollVelocity(5,0,0),
   	 rollV_estimate(5),
   	 throttlePosition(), //P controller with kp=1
   	 throttleVelocity(5,0,0),
   	 throttleV_estimate(5)
  {
   control_pub  = t_node_handle_.advertise<pixy_msgs::ControlOutput>("control_op", 5);
   quat_angle = t_node_handle_.advertise<geometry_msgs::Quaternion>("output_quaternion", 5);
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

void TopControl::update_IMU(const sensor_msgs::Imu::ConstPtr& data){

	ros::Time current;

	tf::Quaternion q(data->orientation.x,
	data->orientation.y,
	data->orientation.z,
	data->orientation.w);

	tf::Matrix3x3 m(q);
	m.getRPY(current_roll, current_pitch, current_yaw);

 /*   float RPY [3] = {current_roll , current_pitch , current_yaw};
	float result[4];

	e2q(RPY,result);
	std::cout <<"q0 "<< *result <<std::endl;
	std::cout <<"q1 "<< *(result+1) <<std::endl;
	std::cout <<"q2 "<< *(result+2) <<std::endl;
	std::cout <<"q3 "<< *(result+3) <<std::endl;*/


}

void TopControl::stick_input(const mavros_msgs::RCIn::ConstPtr& msg) {

	rc_roll = (msg->channels[ROLL] - 1500) / 15;
	rc_pitch =( msg->channels[PITCH] - 1500) /15;
	rc_yaw = (msg->channels[YAW] - 1500) / 15;
    
}
    

//void TopControl::reControl(const pixy_msgs::PixyDist dist) {
void TopControl::reControl(const pixy_msgs::Kalman_estimate dist) {



  //Compute and publish the control output when a new dist estimate is reached

	pixy_msgs::ControlOutput output;
	geometry_msgs::Quaternion output_quaternion;
	float pitch_control_op, roll_control_op, throttle_control_op;
  	//ros::Publisher control_pub  = t_node_handle_.advertise<pixy_msgs::ControlOutput>("control_op", 50);

	float output_pitch_angle, output_roll_angle, output_yaw_angle;

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
 	 
  	//Convert pitch rate into angle
  	//Pitch max angle in degrees == 2 deg, set this to eq of.. .5 rate input?

  	output_pitch_angle = pitch_control_op *MAX_ANG_DEG /5;  	 
  	std::cout << "Pitch angle is: " << output_pitch_angle <<std::endl;

  	if (output_pitch_angle>MAX_ANG_DEG)
  	{
    	output_pitch_angle = MAX_ANG_DEG;
  	}

  	if (output_pitch_angle < (MAX_ANG_DEG * -1))
  	{
    	output_pitch_angle = (MAX_ANG_DEG * -1);
   	}



  	//Find roll pose command
   	 // roll_control_op = rollPosition.computeOutput(0, (dist.y));
  	//Find the roll rate command
   	 //    output.roll = rollVelocity.computeOutput(roll_control_op,rollV_estimate.computeDerivative((dist.position.y)));
 	/* output.roll = roll_control_op;
  	output_roll_angle = output.roll;

  	if (output_roll_angle>MAX_ANG_DEG)
  	{
    	output_roll_angle = MAX_ANG_DEG;
  	}

  	if (output_roll_angle < (MAX_ANG_DEG * -1))
  	{
    	output_roll_angle = (MAX_ANG_DEG * -1);
   	}
*/
  	//Find Throttle pose command
   	 //    throttle_control_op = throttlePosition.computeOutput(0, (dist.position.z));
  	//Find the throttle rate command
  	//  output.throttle = throttleVelocity.computeOutput(throttle_control_op,throttleV_estimate.computeDerivative((dist.position.z)));
  	// output.throttle = 0;

  	// output_yaw_angle = 0;

  	//Convert angles to radians
  	output_pitch_angle = output_pitch_angle * PI /180;
  	// output_roll_angle = output_pitch_angle * PI /180;
 	// output_yaw_angle = current_yaw;

  	std::cout << "Desired corrected Pitch angle is: " << output_pitch_angle <<std::endl;
/*  	std::cout << "Desired corrected Roll angle is: " << output_roll_angle <<std::endl;
  	std::cout << "Desired corrected yaw angle is: " << output_yaw_angle <<std::endl;*/


/*  	output_quaternion.x = (*(result+3)) * -1;
  	output_quaternion.y = (*(result+0)) * -1;
  	output_quaternion.z = (*(result+1)) * -1;
  	output_quaternion.w = (*(result+2)) * -1;*/

   	// std::cout << dist.position.x <<std::endl;
  	// std::cout << "Pitch report" << std::endl;
  	// std::cout << pitch_control_op <<std::endl;

/*	std::cout <<"qX "<< output_quaternion.x <<std::endl;
	std::cout <<"qY "<< output_quaternion.y <<std::endl;
	std::cout <<"qZ "<< output_quaternion.z <<std::endl;
	std::cout <<"qW "<< output_quaternion.w <<std::endl;
	std::cout  <<std::endl;
	*/

/*	//ROS_INFO(output.pitch);


*/


	return;
}

int main(int argc, char  **argv) {
  //Initiallise the NodeHandle
  ros::init(argc, argv, "quad_control_node");
  std::cout << "INIT" << std::endl;

  //TopControl top_cont;
  geometry_msgs::Quaternion output_quaternion;

  ros::NodeHandle my_node_handle_;
  TopControl myController;

  ros::Subscriber kalman_sub = my_node_handle_.subscribe("kalman_estimate", 10 , &TopControl::reControl, &myController);
  ros::Subscriber IMU_sub = my_node_handle_.subscribe("/mavros/imu/data", 5 , &TopControl::update_IMU, &myController);
 
  ros::Subscriber stick_cmd_sub = my_node_handle_.subscribe("/mavros/rc/in", 5 , &TopControl::stick_input, &myController);


  while (ros::ok())
  {

  	float RPY [3] = {myController.rc_roll , myController.rc_pitch , myController.rc_yaw};
  	float result[4];

  	e2q(RPY,result);

  	output_quaternion.x = (*(result+1)) * -1;
  	output_quaternion.y = (*(result+2)) * -1;
  	output_quaternion.z = (*(result+3)) * -1;
  	output_quaternion.w = (*(result+0)) * -1;

  	if (!myController.quat_angle) {
      	ROS_WARN("INVALID PUB");
  	} else {
    	myController.quat_angle.publish(output_quaternion);
 	}
 

  	ros::spinOnce();
  }


  return 0;
}


