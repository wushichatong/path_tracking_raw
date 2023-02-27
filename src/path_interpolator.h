
#include "ros/ros.h"
#include "tf/tf.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include "section_interpolator.h"
#include <tracking_pid/FollowPathAction.h>
#include <tracking_pid/FollowPathFeedback.h>
#include <tracking_pid/FollowPathGoal.h>
#include <tracking_pid/FollowPathResult.h>
#include <tracking_pid/traj_point.h>
#include <tracking_pid/PidConfig.h>
#include <tracking_pid/TargetVelocityConfig.h>
#include <tracking_pid/PidDebug.h>
#include <visualization_msgs/Marker.h>

#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/ExePathGoal.h"
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "cubic_spline_interpolator.h"


#include <dynamic_reconfigure/server.h>
#include <queue>
class PathInterpolator
{
private:
    ros::NodeHandle node_priv;
    // Topic and node names and message objects
    ros::Publisher _trajectory_pub;
    ros::Publisher _visualization_pub;
    ros::Publisher _dense_pub;
    ros::Publisher _pub_finished;
    ros::Publisher _smooth_path_pub;
    ros::Publisher _raw_path_pub;
    ros::Subscriber _pause_sub;
    ros::Subscriber _path_sub;
    // ros::Subscriber subs_odom;
    ros::ServiceClient enable_srv;

    // Debugging of controller
    ros::Publisher _debug_pub;
    // Rviz visualization
    ros::Publisher _marker_pub;

    // Timer

    ros::Timer _timer;

    std::string robot_frame;
    bool _paused;
    bool _is_free = true;

    geometry_msgs::PoseStamped _latest_subgoal_pose;
    geometry_msgs::Twist  _latest_subgoal_vel;

    nav_msgs::Path _latest_path_msg;
    std::vector<geometry_msgs::PoseStamped> _path_poses;
    float _rate;

    float _target_x_vel;
    float _target_x_acc;
    float _target_yaw_vel;
    float _target_yaw_acc;

    SectionInterpolator _current_section;

    std::queue<std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>> _sections;

    actionlib::SimpleActionClient<mbf_msgs::ExePathAction> _client ;

    // create a cubic spline interpolator
    path_smoothing::CubicSplineInterpolator _cubic_spline_interpolator;
public:
    PathInterpolator(/* args */);
    ~PathInterpolator();
    bool is_free();

    void start_path();
    void stop_path();
    void continue_path(ros::Time start_time);
    void _process_pause(const std_msgs::Bool& bool_msg);
    void _accept_path_from_topic(const nav_msgs::Path& path_msg);

    void _preempt_goal();
    void _process_velocity(const tracking_pid::TargetVelocityConfig& config);
    nav_msgs::Path _path_smoother(const nav_msgs::Path& path_msg);
    void _process_path(const nav_msgs::Path& path_msg);
    void _update_target(const ros::TimerEvent& event);
    void _publish_marker(const geometry_msgs::PoseStamped pose_stamped);


};
