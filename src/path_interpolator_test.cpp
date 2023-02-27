// #include "path_interpolator.h"
#include "controller.h"

int main(int argc, char** argv)
{
  // Step 1: Construct Path


  // PathInterpolator interpolator;
  // nav_msgs::Path path_msg;
  // path_msg.header.stamp = ros::Time::now();
  // path_msg.header.frame_id = "map";
  // geometry_msgs::PoseStamped pose;
  // pose.header.stamp = ros::Time::now();
  // pose.header.frame_id = "map";

  // XmlRpc::XmlRpcValue poseList;
  // if (!nh.getParam("/path_poses", poseList))
  // {
  //   ROS_FATAL("Failed to load path point list");
  //   exit(EXIT_FAILURE);
  // }

  // for (int i = 0; i < poseList.size(); i++)
  // {
  //   pose.pose.position.x = static_cast<double>(poseList[i]["x"]);
  //   pose.pose.position.y = static_cast<double>(poseList[i]["y"]);
  //   pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseList[i]["yaw"]);
  //   path_msg.poses.push_back(pose);
  // }


  // Step 2: Path Interpolator

  // Step 3: Create Target Pose

  // Step 4: Get Target Speed
  // pid_controller.setEnabled(controller_enabled);
  // cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, delta_t, &pid_debug);


  // sleep(2);
  // while(ros::ok()){
  //   if(interpolator.is_free()){
  //       interpolator._process_path(path_msg);
  //   }
  //   ros::spinOnce();
  // }
  
  return 0;
}