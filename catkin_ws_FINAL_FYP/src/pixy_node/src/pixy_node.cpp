/*
 * pixy_node is part of the pixy_ros package for interfacing with
 * a CMUcam5 pixy with ROS.
 * Copyright (C) 2014 Justin Eskesen
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>
#include <pixy_msgs/PixyDist.h>
#include <pixy_msgs/ControlOutput.h>
#include "std_msgs/Bool.h"

#include "PixyFilter.h"
#include "p_dist_read.h"
#include <iomanip>
#include "Undistort.cpp"
#include "OutlierFilter.h"
#include "Filter.cpp"

#include "pixy.h"
#define BLOCK_BUFFER_SIZE 100

//pixy_msgs::PixyBlock pixyOutput;

using namespace std;

class PixyNode
{
  public:
    //PixyNode();

    void spin();
    void refind_corners(uint32_t *pixy_vals);
	bool skipFilter;

  private:
    void update();
    void setServo(const pixy_msgs::Servo &msg) { pixy_rcs_set_position(msg.channel, msg.position); }

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Rate rate_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher publisher_;
    ros::Publisher dist_publisher_;
    ros::Subscriber servo_subscriber_;
    std::string frame_id;

    ros::Publisher pixy_f1;
    ros::Publisher pixy_f2;
    ros::Publisher pixy_f3;
    ros::Publisher pixy_f4;

    ros::Publisher target_check;

    bool use_servos_;

    //Declare filterspu
    PixyFilter L_filter;
    PixyFilter R_filter;

    //Make Distance filters
    OutlierFilter dist_x_filter;
    OutlierFilter dist_y_filter;
    OutlierFilter dist_z_filter;

    Filter d_x_avg;
    Filter d_y_avg;
    Filter d_z_avg;

    //Declare Undisort function
    Undistort calibrate;

  public:
    PixyNode() : node_handle_(),
		 private_node_handle_("~"),
		 use_servos_(false),
		 rate_(50.0),
		 L_filter(10),
		 R_filter(10),
		 calibrate(),
		 dist_x_filter(5),
		 dist_y_filter(5),
		 dist_z_filter(5),
		 d_x_avg(5),
		 d_y_avg(5),
		 d_z_avg(5)

    {

	private_node_handle_.param<std::string>(std::string("frame_id"), frame_id,
						std::string("pixy_frame"));

	double rate = 50;
	private_node_handle_.param("rate", rate, 50.0);
	//	rate_=ros::Rate(rate);
	std::cout << "Rate " << rate_.expectedCycleTime() << std::endl;

	private_node_handle_.param("use_servos", use_servos_, false);

	if (use_servos_)
	{
	    //servo_subscriber_ = node_handle_.subscribe("servo_cmd", 20, &PixyNode::setServo, this);
	}

	int ret = pixy_init();
	if (ret != 0)
	{
	    ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
		      __FUNCTION__, ret);
	    ROS_BREAK();
	}
	publisher_ = node_handle_.advertise<pixy_msgs::PixyData>("block_data", 50.0);
	dist_publisher_ = node_handle_.advertise<pixy_msgs::PixyDist>("distance", 50.0);

	pixy_f1 = node_handle_.advertise<pixy_msgs::PixyDist>("Filter1", 10);
	pixy_f2 = node_handle_.advertise<pixy_msgs::PixyDist>("Filter2", 10);
	pixy_f3 = node_handle_.advertise<pixy_msgs::PixyDist>("Filter3", 10);
	pixy_f4 = node_handle_.advertise<pixy_msgs::PixyDist>("Filter4", 10);

	target_check = node_handle_.advertise<std_msgs::Bool>("Target_seen", 10);
    };
};

void PixyNode::update()
{

    // Pixy Block buffer //
    struct Block blocks[BLOCK_BUFFER_SIZE];

    // Get blocks from Pixy //
    int blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, blocks);

    pixy_msgs::PixyData data;
    pixy_msgs::PixyDist filtered_val;

    float *d, *temp_d; // Pointer to distances

    uint32_t block_1[4];
    uint32_t block_2[4];
    uint32_t block1[4];
    uint32_t block2[4];

    pixy_msgs::PixyBlock pixy_temp1;
    pixy_msgs::PixyBlock pixy_temp2;

    std_msgs::Bool target_seen;
    target_seen.data = false;

    if (blocks_copied == 2)
    {
		data.header.stamp = ros::Time::now();
		for (int i = 0; i < blocks_copied; i++)
		{
			pixy_msgs::PixyBlock pixy_block;
			pixy_block.type = blocks[i].type;
			pixy_block.signature = blocks[i].signature;
			//cout << blocks.type << endl;
			//ROS_INFO_STREAM("signature  " << blocks[i].signature << " x_pos " << blocks[i].x << " y_pos " << blocks[i].y << " height " << blocks[i].height << " width " << blocks[i].width);
			pixy_block.roi.x_offset = blocks[i].x;
			pixy_block.roi.y_offset = blocks[i].y;
			pixy_block.roi.height = blocks[i].height;
			pixy_block.roi.width = blocks[i].width;
			pixy_block.roi.do_rectify = false;
			pixy_block.angle =
			(pixy_block.type == TYPE_COLOR_CODE) ? angles::from_degrees((double)blocks[i].angle) : 0.0;
			
			if (i == 0)
			{
				pixy_temp1 = pixy_block;
			}
			
			if (i == 1)
			{
				pixy_temp2 = pixy_block;
			}

			target_seen.data = true;

			data.blocks.push_back(pixy_block);
		}
    }
    else
    {
		//ROS_WARN("Pixy read error.");
		return;
    }

    //Publish if target is seen
    target_check.publish(target_seen);

    //Only if new pixy information is avaliable
    if (blocks_copied == 2)
    {
	//Arrange the blocks so left is 1 and right is 2
	if (pixy_temp1.roi.x_offset < pixy_temp2.roi.x_offset)
	{
		if (skipFilter == false)
			{
				L_filter.addElement(pixy_temp1.roi.x_offset, pixy_temp1.roi.y_offset, pixy_temp1.roi.height, pixy_temp1.roi.width);
				R_filter.addElement(pixy_temp2.roi.x_offset, pixy_temp2.roi.y_offset, pixy_temp2.roi.height, pixy_temp2.roi.width);
			}
	    //Measurement 1: Raw input data
	    if (true)
			{
				block_1[0] = pixy_temp1.roi.height;
				block_1[1] = pixy_temp1.roi.width;
				block_1[2] = pixy_temp1.roi.x_offset;
				block_1[3] = pixy_temp1.roi.y_offset;
				block_2[0] = pixy_temp2.roi.height;
				block_2[1] = pixy_temp2.roi.width;
				block_2[2] = pixy_temp2.roi.x_offset;
				block_2[3] = pixy_temp2.roi.y_offset;
			}
	}
	else
	{
		if (skipFilter == false)
			{
				R_filter.addElement(pixy_temp1.roi.x_offset, pixy_temp1.roi.y_offset, pixy_temp1.roi.height, pixy_temp1.roi.width);
				L_filter.addElement(pixy_temp2.roi.x_offset, pixy_temp2.roi.y_offset, pixy_temp2.roi.height, pixy_temp2.roi.width);
			}
	    if (true)
			{
				block_2[0] = pixy_temp1.roi.height;
				block_2[1] = pixy_temp1.roi.width;
				block_2[2] = pixy_temp1.roi.x_offset;
				block_2[3] = pixy_temp1.roi.y_offset;
				block_1[0] = pixy_temp2.roi.height;
				block_1[1] = pixy_temp2.roi.width;
				block_1[2] = pixy_temp2.roi.x_offset;
				block_1[3] = pixy_temp2.roi.y_offset;
			}
	}

	//Filter 1: raw
	refind_corners(block_1);
	refind_corners(block_2);
	temp_d = getDistances(block_1[0], block_1[1], block_1[2], block_1[3], block_2[0], block_2[1], block_2[2], block_2[3]);

	filtered_val.position.x = *temp_d;
	filtered_val.position.y = *(temp_d + 1);
	filtered_val.position.z = *(temp_d + 2);
	filtered_val.header.stamp = ros::Time::now();

	pixy_f1.publish(filtered_val);

	if(skipFilter == false)
	{	
		//Undisort corners
		block1[0] = static_cast<int>(L_filter.getLastH_Value());
		block1[1] = static_cast<int>(L_filter.getLastW_Value());
		block1[2] = static_cast<int>(L_filter.getLastX_Value());
		block1[3] = static_cast<int>(L_filter.getLastY_Value());
		block2[0] = static_cast<int>(R_filter.getLastH_Value());
		block2[1] = static_cast<int>(R_filter.getLastW_Value());
		block2[2] = static_cast<int>(R_filter.getLastX_Value());
		block2[3] = static_cast<int>(R_filter.getLastY_Value());
		
		refind_corners(block1);
		refind_corners(block2);

		//COnvert block to distance
		temp_d = getDistances(block1[0], block1[1], block1[2], block1[3], block2[0], block2[1], block2[2], block2[3]);
		d = getDistances(block1[0], block1[1], block1[2], block1[3], block2[0], block2[1], block2[2], block2[3]);

		//Filter 2: Pixy val filter
		filtered_val.position.x = *temp_d;
		filtered_val.position.y = *(temp_d + 1);
		filtered_val.position.z = *(temp_d + 2);
		filtered_val.header.stamp = ros::Time::now();

		pixy_f2.publish(filtered_val);

		dist_x_filter.addElement(*temp_d);
		dist_y_filter.addElement(*(temp_d + 1));
		dist_z_filter.addElement(*(temp_d + 2));


		(*temp_d) = dist_x_filter.getLastValue();
		*(temp_d + 1) = dist_y_filter.getLastValue();
		*(temp_d + 2) = dist_z_filter.getLastValue();

		//Filter 3: Outlier removal
		filtered_val.position.x = *temp_d;
		filtered_val.position.y = *(temp_d + 1);
		filtered_val.position.z = *(temp_d + 2);
		filtered_val.header.stamp = ros::Time::now();

		pixy_f3.publish(filtered_val);

		d_x_avg.addElement(*temp_d);
		d_y_avg.addElement(*(temp_d + 1));
		d_z_avg.addElement(*(temp_d + 2));

		(*d) = d_x_avg.get_average();
		*(d + 1) = d_y_avg.get_average();
		*(d + 2) = d_z_avg.get_average();

		//Filter 4: Outlier removal + Smoothing
		filtered_val.position.x = *d;
		filtered_val.position.y = *(d + 1);
		filtered_val.position.z = *(d + 2);
		filtered_val.header.stamp = ros::Time::now();

		pixy_f4.publish(filtered_val);

		if (isinf(*d) || isinf(*(d + 1)) || isinf(*(d + 2)))
		{
			/* code */
			std::cout << "Inf.." << std::endl;
			R_filter.clearFilter();
			L_filter.clearFilter();
		}
		

		// publish the message
		filtered_val.header.stamp = ros::Time::now();}

	dist_publisher_.publish(filtered_val);
	publisher_.publish(data);
    }
}

void PixyNode::spin()
{

    while (node_handle_.ok())
    {
	//cout << "in loop" << endl;
	update();

	ros::spinOnce();
	rate_.sleep();
    }
}

void PixyNode::refind_corners(uint32_t *pixy_vals)
{
    //Function refinds corners from undistorted vals

    //Declare corner pairs of values. 1 is Top left, continue clockwise
    //Extract info from input
    uint32_t corner1[2] = {pixy_vals[2] - (pixy_vals[1] / 2), pixy_vals[3] + (pixy_vals[0] / 2)};
    uint32_t corner2[2] = {pixy_vals[2] + (pixy_vals[1] / 2), pixy_vals[3] + (pixy_vals[0] / 2)};
    uint32_t corner3[2] = {pixy_vals[2] + (pixy_vals[1] / 2), pixy_vals[3] - (pixy_vals[0] / 2)};
    uint32_t corner4[2] = {pixy_vals[2] - (pixy_vals[1] / 2), pixy_vals[3] - (pixy_vals[0] / 2)};

    //Recalculate all the points
    uint32_t new_corner1[2] = {calibrate.findNewXVal(corner1[0], corner1[1]), calibrate.findNewYVal(corner1[0], corner1[1])};
    uint32_t new_corner2[2] = {calibrate.findNewXVal(corner2[0], corner2[1]), calibrate.findNewYVal(corner2[0], corner2[1])};
    uint32_t new_corner3[2] = {calibrate.findNewXVal(corner3[0], corner3[1]), calibrate.findNewYVal(corner3[0], corner3[1])};
    uint32_t new_corner4[2] = {calibrate.findNewXVal(corner4[0], corner4[1]), calibrate.findNewYVal(corner4[0], corner4[1])};

    //Refind original input values h ,w , x , y
    *(pixy_vals) = (new_corner1[1] + new_corner2[1] - new_corner3[1] - new_corner4[1]) / 2;
    *(pixy_vals + 1) = (-new_corner1[0] + new_corner2[0] + new_corner3[0] - new_corner4[0]) / 2;
    *(pixy_vals + 2) = (new_corner1[0] + new_corner2[0] + new_corner3[0] + new_corner4[0]) / 4;
    *(pixy_vals + 3) = (new_corner1[1] + new_corner2[1] + new_corner3[1] + new_corner4[1]) / 4;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixy_node");

    ros::NodeHandle my_handle;
    PixyNode myPixy;

	if (argc > 2){
		myPixy.skipFilter = true;
		std::cout << "Skipping filters" << std::endl;
	} else {
		myPixy.skipFilter = false;
	}

    //ros::Publisher pixy_pub_sig = my_handle.advertise<pixy_msgs::PixyBlock>("/RosAria/pixyOutput",1);

    ROS_INFO("PixyNode for ROS");

    myPixy.spin();

    /*
	while(ros::ok())
	{
		std::cout << "Loop Running" << std::endl;
		//ROS_INFO("%s", pixyOutput.type.c_str());
		//ROS_INFO("%s", pixyOutput.signature.c_str());
		//ROS_INFO(string(data.type));

		myPixy.spin();

		//ros::spinOnce();
		//loop_rate.sleep();

		//pixy_pub_sig.publish(pixyOutput);
	}
*/
    return (0);
}

// EOF
