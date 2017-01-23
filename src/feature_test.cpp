#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>

//#include <robot_calibration/capture/chain_manager.h>
#include <robot_calibration/capture/feature_finder.h>

#include <camera_calibration_parsers/parse.h>
//#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/camera_info.h>

#include <boost/foreach.hpp>  // for rosbag iterator

using namespace robot_calibration;

int main(int argc, char** argv)
{
	std::cout << "test_feature" << std::endl;
	ros::init(argc, argv,"robot_calibration");
	ros::NodeHandle nh("~");
	//robot_calibration::ChainManager chain_manager_(nh);
	robot_calibration_msgs::CalibrationData msg;
	std::vector<robot_calibration_msgs::CaptureConfig> poses;

	robot_calibration::FeatureFinderMap finders_;
	if (!robot_calibration::loadFeatureFinders(nh, finders_))
	{
		ROS_FATAL("Unable to load feature finders!");
		return -1;
	}

	bool found_all_features = true;
	unsigned pose_idx = 0;
	// In manual mode, we need to capture all features
	for (robot_calibration::FeatureFinderMap::iterator it = finders_.begin(); it != finders_.end(); it++)
	{
		if (!it->second->find(&msg))
		{
			ROS_WARN("%s failed to capture features.", it->first.c_str());
			found_all_features = false;
			break;
		}
	}
	// Make sure we succeeded
	if (found_all_features)
	{
		ROS_INFO("Captured pose %u", pose_idx);
	}
	else
	{
		ROS_WARN("Failed to capture sample %u.", pose_idx);
	}
}