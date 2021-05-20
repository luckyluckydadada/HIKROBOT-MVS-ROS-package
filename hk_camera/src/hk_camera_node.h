
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include "hk_camera.h"

#include "MvCameraControl.h"

class HkCamNode
{
public:
	ros::NodeHandle node_;
	ros::ServiceServer service_start_, service_stop_;
	sensor_msgs::Image img_msg;
	image_transport::CameraPublisher image_pub_;
	HkCam cam_;

	HkCamNode();
	~HkCamNode();
	bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	bool send_image();
	bool spin();
};
