#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

#include "hk_camera.h"
#include "hk_camera_node.h"

HkCamNode::~HkCamNode()
{
	cam_.shutdown();
}

HkCamNode::HkCamNode()
	: node_("~")
{
	image_transport::ImageTransport image_topic(node_);
	image_pub_ = image_topic.advertiseCamera("image_raw", 1);
	node_.param("camera_frame_id", img_msg.header.frame_id, std::string("hk_camera"));

	service_start_ = node_.advertiseService("start_capture", &HkCamNode::service_start_cap, this);
	service_stop_ = node_.advertiseService("stop_capture", &HkCamNode::service_stop_cap, this);

	ROS_INFO("Starting ");
	cam_.start();
}

bool HkCamNode::service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	return true;
}

bool HkCamNode::service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	cam_.shutdown();
	return true;
}

bool HkCamNode::send_image()
{
	if (cam_.grab_rgb_image())
	{
		img_msg.header.stamp = ros::Time::now();
		fillImage(img_msg, "rgb8", cam_.stConvertParam.nHeight, cam_.stConvertParam.nWidth, 3 * cam_.stConvertParam.nWidth, cam_.stConvertParam.pDstBuffer);
		
		sensor_msgs::CameraInfo ci;
		ci.header.frame_id = img_msg.header.frame_id;
		ci.header.stamp = img_msg.header.stamp;

		image_pub_.publish(img_msg, ci);
		return true;
	}
	return false;
}

bool HkCamNode::spin()
{
	ros::Rate loop_rate(10);
	while (node_.ok())
	{
		if (!send_image())
			ROS_WARN("HIKROBOT camera did not respond in time.");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hk_camera_node");
	HkCamNode a;
	a.spin();
	return EXIT_SUCCESS;
}