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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float64.h>

#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3

#define MAX_RATE_OP 200
#define MAX_ANG_DEG 3
#define PI 3.14159

#define BLOCK_BUFFER_SIZE 100

using namespace std;

class TopControl
{
  public:
    //void reControl(const pixy_msgs::PixyDist dist);
    void reControl(const pixy_msgs::Kalman_estimate dist);
    void update_IMU(const sensor_msgs::Imu::ConstPtr &data);
    void stick_input(const mavros_msgs::RCIn::ConstPtr &msg);

    float rc_roll;
    float rc_pitch;
    float rc_yaw;
    float rc_throttle;
    float last_rc_yaw;

    float rate_roll, rate_pitch, rate_yaw, rate_throttle;

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

    double current_roll, current_pitch, current_yaw;
	int count; // Variable to check when last image data was sent.

  public:
    TopControl() : t_node_handle_(),
		   rate_(50),
		   pitchPosition(2, 0, 0), //P controller with kp=1
		   pitchVelocity(0, 0, 0),
		   pitchV_estimate(5),
		   rollPosition(), //P controller with kp=1
		   rollVelocity(5, 0, 0),
		   rollV_estimate(5),
		   throttlePosition(), //P controller with kp=1
		   throttleVelocity(5, 0, 0),
		   throttleV_estimate(5)
    {

	pitchPosition.setKp(10);
	count = 0;
    };
};

void TopControl::spin()
{
    while (t_node_handle_.ok())
    {

	ros::spinOnce();

	rate_.sleep(); //Maybe not necessary
    }
}

void TopControl::update_IMU(const sensor_msgs::Imu::ConstPtr &data)
{


    float result[4] = {
	data->orientation.w,
	data->orientation.x,
	data->orientation.y,
	data->orientation.z};

    float RPY[3];

    q2e(result, RPY);
    RPY[0] = RPY[0] * -1;
    RPY[2] = RPY[2] - (PI / 2);

    current_yaw = RPY[2];
	count++;


}

void TopControl::stick_input(const mavros_msgs::RCIn::ConstPtr &msg)
{

    rc_roll = (msg->channels[ROLL] - 1500) / 15;
    rc_pitch = (msg->channels[PITCH] - 1500) / 15;
    rc_yaw = (msg->channels[YAW] - 1500) / 15;
    rc_throttle = (msg->channels[THROTTLE]);

    rate_throttle = (rc_throttle - 1000.0) / 1000.0;


    rate_roll = rc_roll * 3 / 180;
    //rate_pitch = rc_pitch * 3 / 180;
    rate_yaw = (current_yaw + (rc_yaw * 3 / 180)) + (PI / 2); //msg->channels[YAW];

    //	rc_yaw = last_rc_yaw + rc_yaw;
    //	last_rc_yaw = rc_yaw;
}

//void TopControl::reControl(const pixy_msgs::PixyDist dist) {
void TopControl::reControl(const pixy_msgs::Kalman_estimate dist)
{

	count = 0; //Reset count
    //Compute and publish the control output when a new dist estimate is reached

    pixy_msgs::ControlOutput output;
    geometry_msgs::Quaternion output_quaternion;

    //Control Loop outputs (Rate)
    float pitch_control_op, roll_control_op, throttle_control_op;
    //Angle output
    float output_pitch_angle, output_roll_angle, output_yaw_angle;

    //Find Pitch pose command
    pitch_control_op = pitchPosition.computeOutput(3.0, (dist.x));
    //Find the pitch rate command
    output.pitch = pitchVelocity.computeOutput(pitch_control_op, dist.x_dot);

    // Section for limiting control output
    if (output.pitch > MAX_RATE_OP)
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

    output_pitch_angle = pitch_control_op * MAX_ANG_DEG / 5;
    std::cout << "Pitch angle is: " << output_pitch_angle << std::endl;

    if (output_pitch_angle > MAX_ANG_DEG)
    {
	output_pitch_angle = MAX_ANG_DEG;
    }

    if (output_pitch_angle < (MAX_ANG_DEG * -1))
    {
	output_pitch_angle = (MAX_ANG_DEG * -1);
    }

	if (count >150) {
		ROS_WARN("No image Data");
		output_pitch_angle = 0;
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

    std::cout << "Desired corrected Pitch angle in deg is: " << output_pitch_angle << std::endl;

    //Convert angles to radians
    output_pitch_angle = output_pitch_angle * PI / 180;
    // output_roll_angle = output_pitch_angle * PI /180;
    // output_yaw_angle = current_yaw;

	rate_pitch = output_pitch_angle;





    return;
}

int main(int argc, char **argv)
{
    //Initiallise the NodeHandle
    ros::init(argc, argv, "quad_control_node");
    std::cout << "INIT" << std::endl;

    //TopControl top_cont;
    geometry_msgs::Quaternion output_quaternion;
    geometry_msgs::Twist rate_c;
    std_msgs::Float64 thr_cmd;

    ros::NodeHandle my_node_handle_;

    ros::Publisher quat_angle;

    ros::Publisher vel_cmd = my_node_handle_.advertise<geometry_msgs::Twist>("Rate_cmd", 5);
    ros::Publisher throttle = my_node_handle_.advertise<std_msgs::Float64>("custom_throttle", 5);

    TopControl myController;

    ros::Subscriber kalman_sub = my_node_handle_.subscribe("kalman_estimate", 10, &TopControl::reControl, &myController);
    ros::Subscriber IMU_sub = my_node_handle_.subscribe("/mavros/imu/data", 5, &TopControl::update_IMU, &myController);

    ros::Subscriber stick_cmd_sub = my_node_handle_.subscribe("/mavros/rc/in", 5, &TopControl::stick_input, &myController);
    quat_angle = my_node_handle_.advertise<geometry_msgs::Quaternion>("output_quaternion", 5);

    while (ros::ok())
    {

	thr_cmd.data = myController.rate_throttle;

	float RPY[3] = {myController.rate_roll, myController.rate_pitch, myController.rate_yaw};
	float result[4];

	e2q(RPY, result);
	q2e(result, RPY);

	output_quaternion.x = (*(result + 1)) * 1;
	output_quaternion.y = (*(result + 2)) * 1;
	output_quaternion.z = (*(result + 3)) * 1;
	output_quaternion.w = (*(result + 0)) * 1;

	//std::cout <<"RPY "<< myController.rate_roll<<"  "<<  myController.rate_pitch <<"  "<< myController.rate_yaw <<std::endl;

	vel_cmd.publish(rate_c);

	throttle.publish(thr_cmd);

	if (!quat_angle)
	{
	    ROS_WARN("INVALID PUB");
	}
	else
	{
	    quat_angle.publish(output_quaternion);
	}

	ros::spinOnce();
    }

    return 0;
}
