#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <std_msgs/String.h>
#include <robot_calibration_msgs/CalibrationData.h>
#include <robot_calibration_msgs/CaptureConfig.h>

#include <robot_calibration/capture/chain_manager.h>
#include <robot_calibration/capture/feature_finder.h>

#include <camera_calibration_parsers/parse.h>
//#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/camera_info.h>

#include <boost/foreach.hpp>  // for rosbag iterator

int main(int argc, char** argv){
	ros::init(argc, argv,"robot_calibration");
	ros::NodeHandle nh("~");
	robot_calibration::ChainManager manager(nh);
  robot_calibration::FeatureFinderMap finders_;

	std::string pose_bag_name("calibration_poses.bag");
	if (argc > 1)
		pose_bag_name = argv[1];

	std::vector<robot_calibration_msgs::CaptureConfig> poses;
  std::vector<robot_calibration_msgs::CalibrationData> data;
  if (!robot_calibration::loadFeatureFinders(nh, finders_))
    {
      ROS_FATAL("Unable to load feature finders!");
      return -1;
    }

  ros::Publisher pub = nh.advertise<robot_calibration_msgs::CalibrationData>("/calibration_data", 10);

	if (pose_bag_name.compare("--manual") != 0)
	{
	  ROS_INFO_STREAM("Opening " << pose_bag_name);
	  rosbag::Bag bag;
	  try
	  {
		bag.open(pose_bag_name, rosbag::bagmode::Read);
	  }
	  catch (rosbag::BagException)
	  {
		ROS_FATAL_STREAM("Cannot open " << pose_bag_name);
		return -1;
	  }
	  ROS_ERROR_STREAM("HOLAA");
	  rosbag::View data_view(bag, rosbag::TopicQuery("calibration_joint_states"));

	  BOOST_FOREACH (rosbag::MessageInstance const m, data_view)
	  {
  		robot_calibration_msgs::CaptureConfig::ConstPtr msg = m.instantiate<robot_calibration_msgs::CaptureConfig>();
        if (msg == NULL)
        {
          // Try to load older style bags
          sensor_msgs::JointState::ConstPtr js_msg = m.instantiate<sensor_msgs::JointState>();
          if (js_msg != NULL)
          {
            robot_calibration_msgs::CaptureConfig config;
            config.joint_states = *js_msg;
            for (robot_calibration::FeatureFinderMap::iterator it = finders_.begin();
                 it != finders_.end();
                 it++)
            {
              config.features.push_back(it->first);
            }
            poses.push_back(config);
          }
        }		
		else
		{
			poses.push_back(*msg);
	    }
	  }

    }
    for (unsigned pose_idx = 0;
         (pose_idx < poses.size() || poses.size() == 0) && ros::ok();
         ++pose_idx)
    {
      robot_calibration_msgs::CalibrationData msg;

      if (poses.size() == 0)
      {
        // Manual calibration, wait for keypress
        ROS_INFO("Press key when arm is ready... (type 'done' to finish capture)");
        std::string throwaway;
        std::getline(std::cin, throwaway);
        if (throwaway.compare("done") == 0)
          break;
        if (!ros::ok())
          break;
      }
      else
      {
        // Move head/arm to pose
        if (!manager.moveToState(poses[pose_idx].joint_states))
        {
          ROS_WARN("Unable to move to desired state for sample %u.", pose_idx);
          continue;
        }
      }

      // Regardless of manual vs. automatic, wait for joints to settle
      manager.waitToSettle();

      // Make sure sensor data is up to date after settling
      ros::Duration(0.1).sleep();

      // Get pose of the features
      bool found_all_features = true;
      if (poses.size() == 0)
      {
        // In manual mode, we need to capture all features
        for (robot_calibration::FeatureFinderMap::iterator it = finders_.begin();
             it != finders_.end();
             it++)
        {
          if (!it->second->find(&msg))
          {
            ROS_WARN("%s failed to capture features.", it->first.c_str());
            found_all_features = false;
            break;
          }
        }
      }
      else
      {
        // Capture only the intended features for this sample
        // NOTE: while you can capture multiple sensors at once for a single
        //       pose, such things probably won't currently calibrate. Most
        //       of the existing finders will override the entire observation
        //       vector
        for (size_t i = 0; i < poses[pose_idx].features.size(); i++)
        {
          std::string feature = poses[pose_idx].features[i];
          if (!finders_[feature]->find(&msg))
          {
            ROS_WARN("%s failed to capture features.", feature.c_str());
            found_all_features = false;
            break;
          }
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
        continue;
      }

      // Fill in joint values
      manager.getState(&msg.joint_states);

      // Publish calibration data message.
      pub.publish(msg);

      // Add to samples
      data.push_back(msg);
   }

   ROS_INFO("Done capturing samples");

   return 0;
	
}