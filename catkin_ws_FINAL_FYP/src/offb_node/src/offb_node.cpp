/***************************************************************************
 by Logan Porter
 ***************************************************************************/

//Standard Include
#include <ros/ros.h>
#include <cstdlib>
#include <mavros/mavros.h>
#include <iostream>

//#include <unistd.h>

//Include Message Types Used
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
geometry_msgs::Quaternion qt_in;
geometry_msgs::TwistStamped rate_input;
std_msgs::Float64 thr_input;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void qt_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
	qt_in = *msg;
}

void rate_cb(const geometry_msgs::Twist::ConstPtr& msg){
	rate_input.twist = *msg;
}

void thr_cb(const std_msgs::Float64::ConstPtr& msg){
	thr_input = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

	//Sets up subscribers, publishers and clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber angles_in = nh.subscribe<geometry_msgs::Quaternion>
			("output_quaternion",10, qt_cb);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber rate_in = nh.subscribe<geometry_msgs::Twist>
			("Rate_cmd",10, rate_cb);

    ros::Publisher set_thr = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 10);    
    ros::Subscriber throttle_in = nh.subscribe<std_msgs::Float64>
			("custom_throttle",10, thr_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    std_msgs::Float64 throttle_sp; 
    throttle_sp.data = 1;

    // wait for FCU connection (checks for heartbeat)
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	//Sets the position for the quad to go to
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
	pose.pose.orientation.x = qt_in.x;
	pose.pose.orientation.y = qt_in.y;
	pose.pose.orientation.z = qt_in.z;
	pose.pose.orientation.w = qt_in.w;


    geometry_msgs::PoseStamped local_pos;
    local_pos.pose.position.x = 0;
    local_pos.pose.position.y = 0;
    local_pos.pose.position.z = 1.0;
    local_pos.pose.orientation.z = qt_in.z;
	local_pos.pose.orientation.w = qt_in.w;
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_att_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


	//Sets mode change command
    //mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";

	//Sets arming command
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;



		pose.pose.orientation.x = qt_in.x;
		pose.pose.orientation.y = qt_in.y;
		pose.pose.orientation.z = qt_in.z;
		pose.pose.orientation.w = qt_in.w;

	    std::cout <<"Sending qW "<< pose.pose.orientation.w <<std::endl;
        std::cout <<"Sending qX "<< pose.pose.orientation.x <<std::endl;
	    std::cout <<"Sending qY "<< pose.pose.orientation.y <<std::endl;
	    std::cout <<"Sending qZ "<< pose.pose.orientation.z <<std::endl;

        throttle_sp.data = thr_input.data;
        

       // local_rate_pub.publish(rate_input);

        local_att_pub.publish(pose);
        set_thr.publish(throttle_sp);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}







