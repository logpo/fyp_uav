/***************************************************************************
Credit to Varanon Austin Pukasamsombut for the base of the code below

Altered by Logan Porter
 ***************************************************************************/

//Standard Include
#include "ros/ros.h"
#include <cstdlib>
#include <mavros/mavros.h>
#include <iostream>

#include <unistd.h>

//Include Message Types Used
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/RCIn.h>

//Define Radio Channels to Numbers [May differ depending on RC Transmitter]
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3
#define LEFT_TRIGGER 4

//Define Checks
#define PRELIMINARY_CHECK 1
#define LAND_CHECK 2

//Define RC Throttle Values [Differs for all controllers]
#define MIDDLE 1500  //PWM Value with Stick at Middle 
#define RISING 1690  //PWM Value with Stick Slightly Above Middle
#define RELEASE 1100 //PWM Value when Stick is completely Lowered
#define NO_RC 900	 //PWM Value when no RC Controller has been connected.

using namespace std;
//-------------------------------------------------------------------------
class Receiver
{
    public:
        //Callbacks
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);
        
        //Looping Checks
        bool state_finished; //If False, then runs stateCallback on spin.
        bool rc_finished;
        bool terminate; //Release All Channels and Terminate Program

		int test_count = 1200;

        //Miscellaneous Variables
        int state_check;
        int rc_check_ch; //Channel to Check
        int rc_check_val; //Value to Check
        
};
//-------------------------------------------------------------------------
void Receiver::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    //ROS_INFO("ENTER: State Callback");
    if(terminate) return;
    if(state_finished) return;

	//ROS_INFO(msg->mode);
	if(msg->armed){cout << msg->mode << " --- " << "Armed" << endl;}
	else{cout << msg->mode << " --- " << "Disarmed" << endl;}

	state_finished = true;

	if(state_check == LAND_CHECK)
    {
        if(msg->mode == "LAND") 
        {
            ROS_INFO("Completed: Control Switch");
            state_finished = true;
        }
        else 
        {
			state_finished = false;
            ROS_INFO("Attempting: Control SWITCH");
            system("rosrun mavros mavsys mode -c LAND");
        }
    }
}
//-------------------------------------------------------------------------
void Receiver::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    //ROS_INFO("Enter: RC Callback");

    //Terminates all Programs if Left Trigger Moves
    if(msg->channels[LEFT_TRIGGER] > 1700) 
	{
		terminate = true;
		//cout << "trigger val: " << msg->channels[LEFT_TRIGGER] << endl;
	} else {
		terminate = false;
	}
	

    if(rc_finished) return;

    if(msg->channels[rc_check_ch] <= (rc_check_val + 7)
            ||msg->channels[rc_check_ch] >= (rc_check_val - 7)) //RC Values Fluctuates, so +-7 takes that into account.
    {
        rc_finished = true; //If RC_Value is close to Required Value.
        cout << "RC Channel " << rc_check_ch << " change success" << endl;
    }else {
		cout << "SIGNAL IGNORED" << endl;
        rc_finished = false;
	}
}
//-------------------------------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node"); //Initialization
	ros::NodeHandle nh; //Way to Connect Node to Master

	ROS_INFO("Offboard Quadcopter Control");

	//Setting Up All Subscribers and Publishers

	Receiver receiver;
	ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, &Receiver::stateCallback, &receiver);
	ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, &Receiver::rcCallback, &receiver);

	receiver.state_finished = true;
	receiver.rc_finished = true;
	receiver.terminate = true;

	ros::Publisher rc_message = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
	mavros_msgs::OverrideRCIn rc_command;

	ros::spinOnce();
	cout << "terminate status: " << receiver.terminate << endl;

	while(receiver.terminate)
	{
		ros::spinOnce();
	}

	receiver.rc_finished = true;

	//First Arm and Set to Altitude_Hold Mode
	ROS_INFO("Commencing: Autonomous Setup");

	//system("rosrun mavros mavsys mode -c ALT_HOLD");
	system("rosrun mavros mavsys mode -c STABILIZE");

	receiver.state_finished = false; //Calls State Callback
	while ((!receiver.state_finished) && (ros::ok()) && (!receiver.terminate))
		ros::spinOnce();

	for (int i = 0; i < 8; i++) rc_command.channels[i] = 0;//65535;

	receiver.rc_finished = false;
	while ((ros::ok()) && (!receiver.terminate))
	{
		if(receiver.test_count < 1700)
		{
			receiver.test_count++;
		} else {
			receiver.test_count = 1200;
		}	
		rc_command.channels[PITCH] = receiver.test_count;
		rc_command.channels[ROLL] = receiver.test_count;
		cout << "Pitch input: " << receiver.test_count << endl;

		receiver.rc_finished = false;
		receiver.rc_check_ch = PITCH;
		receiver.rc_check_val = receiver.test_count;
		ros::spinOnce();

		receiver.rc_finished = false;
		receiver.rc_check_ch = ROLL;
		receiver.rc_check_val = receiver.test_count;
		ros::spinOnce();

		rc_message.publish(rc_command);
	}

/*	system("rosrun mavros mavsys mode -c LAND");
	receiver.state_check = LAND_CHECK;
	receiver.state_finished = false;
	while((!receiver.state_finished) && (ros::ok())){
		ros::spinOnce();
	}

	cout << "Quad Landing --- Program Complete" << endl;
*/
	for (int i = 0; i < 8; i++){
		rc_command.channels[i] = 0;//Releases all Channels
	}
	
	receiver.rc_check_ch = PITCH;
	receiver.rc_check_val = MIDDLE;
	receiver.rc_finished = false;	

	while (ros::ok() && (!receiver.rc_finished))
	{
		ros::spinOnce();
		rc_message.publish(rc_command);
	} 
	ROS_INFO("Control Handover Complete --- Program Finished");
}










