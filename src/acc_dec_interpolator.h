#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tracking_pid/traj_point.h>

class AccDecInterpolator
{
private:
    geometry_msgs::Point _start_xyz; 
    geometry_msgs::Point _end_xyz;
    geometry_msgs::Point _delta;

    geometry_msgs::Twist _start_vel;

    float _start_yaw;
    float _end_yaw;
    float _delta_yaw;   
    float _last_yaw;
    bool _first_time = true;



    geometry_msgs::PoseStamped section_start_pose_stamped; 
    geometry_msgs::PoseStamped section_end_pose_stamped;

    float _x_vel;  // type: float
    float _x_acc_decc;
    float _yaw_vel;
    float _yaw_acc_decc;

    float _time_x_acc_decc;   
    float _length_x_acc_decc;
    float _length_x_vel;      
    float _time_x_vel;        
    float _x_vel_adjusted;  

    float _time_yaw_acc_decc;  
    float _length_yaw_acc_decc;
    float _length_yaw_vel;     
    float _time_yaw_vel;       
    float _yaw_vel_adjusted;    

    float x_progress = 0.0;
    float yaw_progress = 0.0;
    float current_x_vel = 0.0;
    float current_yaw_vel = 0.0;

    float _length_of_section;     
    float _length_of_section_ang; 
public:
    AccDecInterpolator(/* args */);
    ~AccDecInterpolator();
    AccDecInterpolator(geometry_msgs::PoseStamped &from_, geometry_msgs::PoseStamped &to, geometry_msgs::Twist start_vel,
                        TimeStamp start_time, float x_vel, float x_acc, float yaw_vel, float yaw_acc);
    geometry_msgs::PoseStamped interpolate(float progress_ratio);
    tracking_pid::traj_point interpolate_with_acceleration(TimeStamp current_time);

    TimeStamp section_start_time;
    TimeStamp section_end_time;
    double duration_for_section = 0.0;
};
