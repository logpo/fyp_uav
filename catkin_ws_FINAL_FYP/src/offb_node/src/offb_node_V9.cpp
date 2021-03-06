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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

	//Sets up subscribers, publishers and clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection (checks for heartbeat)
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    		

	//Sets the position for the quad to go to
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.0;
	//pose.pose.orientation.x = 0;
	//pose.pose.orientation.y = 0;
	//pose.pose.orientation.z = 0;
	//pose.pose.orientation.w = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
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
		pose.pose.position.z = 1.0;
		//pose.pose.orientation.x = 0;
		//pose.pose.orientation.y = 0;
		//pose.pose.orientation.z = 0;
		//pose.pose.orientation.w = 1;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}







