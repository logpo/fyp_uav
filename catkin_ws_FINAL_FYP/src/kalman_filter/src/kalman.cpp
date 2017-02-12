/*Class for kalman filter implementation in realtime using offline calculated gains.
Assumptions:
-Target always in view

Theo Drissner-Devine
15/8/16
theo.drissner@gmail.com

*/
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
//#include </opt/ros/indigo/include/ros/ros.h>

#include </home/theo/fyp/catkin_ws/src/kalman_filter/src/Eigen/Eigen>
#include </home/theo/fyp/catkin_ws/src/kalman_filter/src/Eigen/Eigen>
 
  //#include </home/fyp/catkin_ws/src/kalman_filter/src/Eigen/Eigen>
//  #include </home/fyp/catkin_ws/src/kalman_filter/src/Eigen/Eigen>

// #include </Eigen/Dense>
// #include </Eigen/Eigen>


#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>
#include <pixy_msgs/PixyDist.h>
#include <pixy_msgs/ControlOutput.h>
#include <pixy_msgs/Kalman_estimate.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

//#include <Quaternion.h>
#include <math.h>

#define PI 3.14159

using namespace Eigen;
using namespace std;

class kalman_filter
{
public:
  void predict();
  void corrector();
  void IMU_update(const sensor_msgs::Imu::ConstPtr& data) ;
  void camera_update(const pixy_msgs::PixyDist dist);
  void initialise(float start_val);

private:
  Vector2f _x_last;
  Vector2f _x_hat;
  Matrix2f _A;
  Matrix2f _B;
  Matrix2f _C;
  Vector2f _D;
  Matrix2f _I; //Identity matrix

  float Y; //measured output, Y

  float _u; // Input angle vector

  Matrix2f _Pe;
  Matrix2f _Pe_last;

  Matrix2f _Qd;
  float _Rd;

  Matrix2f _Ke;

  //Discretised state vectors
  Matrix2f _A_d;
  Matrix2f _B_d;

  uint64_t  last_time; //Last time update was made
  uint64_t  current_time; //

  float imu_var = (1.42e-2)/100;
  float cam_var = 0.1/100;

//Ros Variables
  ros::NodeHandle node_handle_;
  ros::Publisher kalman_pub;
  ros::Publisher roll_pub;
  ros::Publisher pitch_pub;
  ros::Publisher yaw_pub;
  ros::Publisher euler_pub;
  ros::Rate rate_;

  float imu_pitch_init;


  float g =9.81;

public:
  kalman_filter():
    node_handle_(),
    rate_(100)
    {
    ROS_INFO("class init");
    initialise(2.0);


    kalman_pub = node_handle_.advertise<pixy_msgs::Kalman_estimate>("kalman_estimate", 5);
    roll_pub      = node_handle_.advertise<std_msgs::Float64>("roll", 5);
    pitch_pub     = node_handle_.advertise<std_msgs::Float64>("pitch", 5);
    yaw_pub       = node_handle_.advertise<std_msgs::Float64>("yaw", 5);

    euler_pub     = node_handle_.advertise<geometry_msgs::Vector3>("Euler_angle", 5);

    //Initialise with junk value to be set in first callback;
    imu_pitch_init = -12; 

    if (!kalman_pub) {
      ROS_WARN("INVALID PUB");
    } else {
      ROS_INFO(" PUB GOOD ");
    }

    ROS_INFO("class init end");


  };
};

  void kalman_filter::predict()
  {
    //Recalculate discrete system and gains based on time difference
    float T = (current_time - last_time)*1e-9;

	 
	if (T>100){
		return;
	}
	

    _A_d = _I + _A * T;
    _B_d = T * _B;

    //Recalculate Qd Matrix

    _Qd(0,0) =   _B_d(0,0) * _B_d(0,0) * imu_var;
    _Qd(0,1) =  0;
    _Qd(1,0) =  0;
    _Qd(1,1) =  _B_d(1,1) * _B_d(1,1) * imu_var;

    _x_hat(0) = (_A_d(0,0) * _x_last (0)) + (_A_d(0,1) * 2 * _x_last (1)) + _B_d(0,0) * _u;
    _x_hat(1) = (_A_d(1,1) * _x_last (1)) +  _B_d(1,0) * _u;

	

    float p1 = _Pe_last(0,0);
    float p2 = _Pe_last(0,1);
    float p3 = _Pe_last(1,0);
    float p4 = _Pe_last(1,1);
    float a1 = _A_d(0,0);
    float a2 = _A_d(0,1);
    float a4 = _A_d(1,1);


    _Pe(0,0) = a1*(a1*p1+a2*p3) + a2*(a1*p2+a2*p4) + _Qd(0,0);
    _Pe(0,1) = a4*(a1*p2+a2*p4);
    _Pe(1,0) = a4*(a1*p3+a2*p4);
    _Pe(1,1) = a4*a4*p4 + _Qd(1,1);


    return;
  }

  void kalman_filter::corrector() {
    pixy_msgs::Kalman_estimate estimate;

    float p1 = _Pe(0,0);
    float p2 = _Pe(0,1);
    float p3 = _Pe(1,0);
    float p4 = _Pe(1,1);

    float x1 = _x_hat(0);
    float x2 = _x_hat(1);

    _Ke(0) = p1/(p1+_Rd);
    _Ke(1) = p3/(p1+_Rd);


    _x_hat(0) = x1 + _Ke(0)*(Y - x1);
    _x_hat(1) = x2 + _Ke(1)*(Y - x1);


    _Pe(0,0) = p1*(1-_Ke(0));
    _Pe(0,1) = p2*(1-_Ke(0));
    _Pe(1,0) = p3-_Ke(1)*p1;
    _Pe(1,1) = p4-_Ke(1)*p2;


    //Set past values
    _x_last = _x_hat;
    _Pe_last = _Pe;


    if (isnan(_x_hat(0)))
    {
      _x_hat(0) = Y;
ROS_WARN("Nan");

    }
    if (isnan(_x_hat(1))) {
      _x_hat(1) = 0;
ROS_WARN("Nan");

    }

    estimate.x = _x_hat(0);
    //Divide by 2, found by testing
    estimate.x_dot = _x_hat(1) / 2;

     std::cout << "Distance " << Y << std::endl;

    if ((estimate.x <= 0) || (estimate.x > 10) || (estimate.x_dot > 10) || (estimate.x_dot  < -10) ){

 			std::cout << "Est x is  " << estimate.x << std::endl;
			std::cout << "Est x dot is  " << _x_hat(1) / 2 << std::endl;
        initialise(Y);
       estimate.x = Y;
       estimate.x_dot = 0;
       ROS_WARN("ERROR");

    }
    std::cout << "Estimate" << estimate << std::endl;
    kalman_pub.publish(estimate);
    return;
  }

  void kalman_filter::IMU_update(const sensor_msgs::Imu::ConstPtr& data) {

    ros::Time current;

    tf::Quaternion q(data->orientation.x,
    data->orientation.y,
    data->orientation.z,
    data->orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (imu_pitch_init < -10)
    {
      imu_pitch_init = pitch;
    }

    //Set to reference from inital, assumed flat start;
    pitch = pitch - imu_pitch_init;

//    ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

    _u = pitch;

    last_time = current_time;

    current = (data->header.stamp);
    current_time = current.toNSec();

    // std_msgs::Float64  roll_msg, pitch_msg, yaw_msg;
    geometry_msgs::Vector3 euler;

    if (yaw<0)
    {
      yaw = yaw + PI;
    } else
    {
      yaw = yaw - PI;
    }

    euler.x = roll;
    euler.y = pitch;
    euler.z = yaw;

    euler_pub.publish(euler);


    /*   
        //roll_msg.data = static_cast<float64>(roll);
    //roll_msg.data = static_cast<float64>(roll_msg);
    roll_msg.data = roll;
    roll_pub.publish(roll_msg);

    pitch_msg.data = pitch;
    pitch_pub.publish(pitch_msg);
    yaw_msg.data = (yaw);
    yaw_pub.publish(yaw_msg);*/

    predict();

    return;
  }

  void kalman_filter::camera_update(const pixy_msgs::PixyDist dist) {
    /* code */
    ros::Time current;


    Y = dist.position.x;

    last_time = current_time;
    current = dist.header.stamp;
    current_time = current.toNSec();

 
    corrector();
    return;

  }

  void kalman_filter::initialise(float start_val){

    ROS_INFO("INITIALISING...");
    float damp = 1;

    //Initialise matrices
    _A(0,0) =  0;
    _A(0,1) =  1;
    _A(1,0) =  0;
    _A(1,1) =  -damp;

    _B(0,0) =  0;
    _B(0,1) =  0;
    _B(1,0) =  0;
    _B(1,1) =  g;

    _C(0,0) =  0;
    _C(0,1) =  0;
    _C(1,0) =  0;
    _C(1,1) =  1;

    _D(0) = 0;
    _D(1) = 0;

    _I(0,0) =  1;
    _I(0,1) =  0;
    _I(1,0) =  0;
    _I(1,1) =  1;


    _x_hat(0) =start_val;
    _x_hat(1) =0;

    _x_last(0) =start_val;
    _x_last(1) =0;

    _u = 0;


    //Initialise other Vars
    float T = 0.0125;

    _A_d = _I + _A * T;
    _B_d = T * _B;

    //Recalculate Qd Matrix

    _Qd(0,0) =   _B_d(0,0) * _B_d(0,0) * imu_var;
    _Qd(0,1) =  0;
    _Qd(1,0) =  0;
    _Qd(1,1) =  _B_d(1,1) * _B_d(1,1) * imu_var;

    _Rd = cam_var;

    _Pe = _Qd;
    _Pe_last = _Pe;

  }

  int main(int argc, char  **argv) {
    ros::init(argc,argv,"kalman_filter");

    ros::NodeHandle my_node_handle_;
    kalman_filter My_K_Filter;

    //ros::Publisher pixy_pub_sig = my_handle.advertise<pixy_msgs::PixyBlock>("/RosAria/pixyOutput",1);

    ROS_INFO("Kalman filter");

    ros::Subscriber IMU_sub = my_node_handle_.subscribe("/mavros/imu/data", 5 , &kalman_filter::IMU_update, &My_K_Filter);
    ros::Subscriber cam_sub = my_node_handle_.subscribe("distance", 5 , &kalman_filter::camera_update, &My_K_Filter);

    ROS_INFO("main init end");

    ros::spin();
    return 0;
  }
