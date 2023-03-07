#include "acc_dec_interpolator.h"


AccDecInterpolator::AccDecInterpolator(/* args */)
{
}

AccDecInterpolator::~AccDecInterpolator()
{
}


AccDecInterpolator::AccDecInterpolator(geometry_msgs::PoseStamped& from_, geometry_msgs::PoseStamped& to,
                             geometry_msgs::Twist start_vel, TimeStamp start_time, 
                             float x_vel, float x_acc, float yaw_vel, float yaw_acc)
{
    /*
    Interpolate over the given section with some x and yaw velocities
    :param from_: start of the section
    :type from_: PoseStamped
    :param to: end of the section
    :type to: PoseStamped
    :param x_vel: translational velocity to move the trajectory target pose
    :type x_vel: float
    :param x_acc: translational acceleration to move the trajectory target pose
    :type x_acc: float
    :param yaw_vel: rotational velocity to rotate the trajectory target pose
    :type yaw_vel: float
    :param yaw_acc: rotational acceleration to rotate the trajectory target pose
    :type yaw_acc: float
    */
    _x_vel = x_vel;  // type: float
    _x_acc_decc = x_acc;
    _yaw_vel = yaw_vel;
    _yaw_acc_decc = yaw_acc;


    section_start_pose_stamped = from_;  // type: PoseStamped
    section_end_pose_stamped   = to;  // type: PoseStamped


    _start_xyz = section_start_pose_stamped.pose.position;
    _end_xyz   = section_end_pose_stamped.pose.position;

    _start_vel = start_vel;
    
    _start_yaw = tf::getYaw(section_start_pose_stamped.pose.orientation);

    _end_yaw = tf::getYaw(section_end_pose_stamped.pose.orientation);

    // Warning! These calculations are only valid for yaw. So not to be used in 3D

    _delta.x = _end_xyz.x - _start_xyz.x;
    _delta.y = _end_xyz.y - _start_xyz.y;
    _delta.z = _end_xyz.z - _start_xyz.z;

    _delta_yaw = _end_yaw - _start_yaw;
    // _delta_yaw = ( _delta_yaw + M_PI) / (2 * M_PI) +  M_PI;


    _length_of_section     = sqrt(_delta.x * _delta.x  + _delta.y * _delta.y  +_delta.z * _delta.z);
    _length_of_section_ang = fabs(_delta_yaw);

    _time_x_acc_decc   = _x_vel / _x_acc_decc;  // Time during (de) and acceleration phases t = v/a
    _length_x_acc_decc =  0.5 * _x_acc_decc * (_time_x_acc_decc * _time_x_acc_decc) ;// Translation during acceleration phase x = 0.5a*t^2
    _length_x_vel      = _length_of_section - _length_x_acc_decc - _length_x_acc_decc ;// Translation during constant velocity phase
    _time_x_vel        = _length_x_vel/_x_vel ; // Time during constant velocity phase t = v/a
    _x_vel_adjusted    = _x_vel;

    if (_time_x_vel < 0)  // No constant acceleration phase. Recompute (de)-acceleration phase
    {
        _length_x_acc_decc = 0.5 * _length_of_section;
        _time_x_acc_decc = sqrt(2 * _length_x_acc_decc / _x_acc_decc); // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
        _x_vel_adjusted = _x_acc_decc * _time_x_acc_decc;
        _length_x_vel = 0.0;
        _time_x_vel = 0.0;
    }    

    _time_yaw_acc_decc   = _yaw_vel/_yaw_acc_decc ; // Time during acceleration phase t = v/a
    _length_yaw_acc_decc =  0.5*_yaw_acc_decc*(_time_yaw_acc_decc*_time_yaw_acc_decc); // Translation during acceleration phase x = 0.5a*t^2
    _length_yaw_vel      = _length_of_section_ang - _length_yaw_acc_decc - _length_yaw_acc_decc; // Translation during constant velocity phase
    _time_yaw_vel        = _length_yaw_vel/_yaw_vel;  // Time during constant velocity phase t = v/a
    _yaw_vel_adjusted    = _yaw_vel;

    if (_time_yaw_vel < 0)
    {
        _length_yaw_acc_decc = 0.5*_length_of_section_ang;
        _time_yaw_acc_decc = sqrt(2*_length_yaw_acc_decc/_yaw_acc_decc); // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
        _yaw_vel_adjusted = _yaw_acc_decc*_time_yaw_acc_decc;
        _length_yaw_vel = 0.0;
        _time_yaw_vel = 0.0;
    }  // No constant acceleration phase. Recompute (de)-acceleration phase



    float duration_for_section_x = _time_x_acc_decc + _time_x_vel + _time_x_acc_decc;
    float duration_for_section_yaw = _time_yaw_acc_decc + _time_yaw_vel + _time_yaw_acc_decc;
    duration_for_section = ros::Duration(std::max(duration_for_section_x, duration_for_section_yaw));

    ROS_INFO("delta_x:%f delta_y: %f delta_yaw: %f", _delta.x, _delta.y, _delta_yaw);
    // ROS_INFO("_x_acc_decc: %f",_x_acc_decc);
    // ROS_INFO("_x_vel: %f",_x_vel);
    // ROS_INFO("_length_of_section: %f",_length_of_section);
    // ROS_INFO("_time_x_acc_decc: %f",_time_x_acc_decc);
    // ROS_INFO("_time_x_vel: %f",_time_x_vel);
    // ROS_INFO("_length_x_acc_decc: %f",_length_x_acc_decc);
    // ROS_INFO("_length_x_vel: %f",_length_x_vel);
    // ROS_INFO("duration_for_section_x: %f",duration_for_section_x);

    // ROS_INFO("_yaw_acc_decc: %f",_yaw_acc_decc);
    // ROS_INFO("_yaw_vel: %f",_yaw_vel);
    // ROS_INFO("_length_of_section_ang: %f",_length_of_section_ang);
    // ROS_INFO("_time_yaw_acc_decc: %f",_time_yaw_acc_decc);
    // ROS_INFO("_time_yaw_vel: %f",_time_yaw_vel);
    // ROS_INFO("_length_yaw_acc_decc: %f",_length_yaw_acc_decc);
    // ROS_INFO("_length_yaw_vel: %f",_length_yaw_vel);

    // ROS_INFO("duration_for_section_yaw: %f",duration_for_section_yaw);
    // ROS_INFO("duration_for_section: %f", duration_for_section.toSec());



    section_start_time = start_time;
    x_progress = 0.0;
    yaw_progress = 0.0;
    current_x_vel = 0.0;
    current_yaw_vel = 0.0;
    section_end_time = section_start_time + duration_for_section;
}


geometry_msgs::PoseStamped AccDecInterpolator::interpolate(float progress_ratio)
{
    /*
    Calculate where we should be along the section given a ratio of progress.
    0.0 means we're at the start, 1.0 means finished
    :param progress_ratio: How far along the section are we?
    :type progress_ratio: float
    :return: an interpolation between the Section's start and end
    :rtype: PoseStamped
    */
    // target_x = start_x + delta_x * progress_on_section

    float next_yaw = _start_yaw + _delta_yaw * progress_ratio;
    geometry_msgs::PoseStamped next_pose;
    // next_pose.header.stamp is to be filled by the caller
    next_pose.header.frame_id = section_start_pose_stamped.header.frame_id;
    next_pose.pose.position.x = _start_xyz.x + _delta.x * progress_ratio;
    next_pose.pose.position.y = _start_xyz.y + _delta.y * progress_ratio;
    next_pose.pose.position.z = _start_xyz.z + _delta.z * progress_ratio;
    // Compute orientation. PID can use it for holonomic robots
    next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);

    return next_pose;
}


tracking_pid::traj_point AccDecInterpolator::interpolate_with_acceleration(TimeStamp current_time)
{
    /*
    Calculate where we should be along the section given a ratio of progress.
    0.0 means we're at the start, 1.0 means finished
    :param progress_ratio: How far along the section are we?
    :type progress_ratio: float
    :return: an interpolation between the Section's start and end
    :rtype: PoseStamped
    */

    ros::Duration current_section_time = current_time - section_start_time;
    float t = current_section_time.toSec();
    float tr;


    if(t < _time_x_acc_decc){
        tr = t;
        x_progress = 0.5 * _x_acc_decc * tr * tr;
        current_x_vel = _x_acc_decc*tr;
        if (x_progress > _length_x_acc_decc){
            current_x_vel = _x_vel_adjusted;
            x_progress = _length_x_acc_decc;
        }   
    }
    else if (t < (_time_x_acc_decc + _time_x_vel)){
        tr = (t - _time_x_acc_decc);
        x_progress = _length_x_acc_decc + current_x_vel*tr;
        current_x_vel = _x_vel_adjusted;
        if (x_progress > (_length_x_acc_decc+_length_x_vel))
            x_progress = (_length_x_acc_decc+_length_x_vel);
    }
    else if (t < (_time_x_acc_decc + _time_x_vel + _time_x_acc_decc)){
        tr = (t - _time_x_acc_decc - _time_x_vel);
        x_progress = (_length_x_acc_decc + _length_x_vel) + _x_vel_adjusted*tr - 0.5*_x_acc_decc*tr*tr;
        current_x_vel = _x_vel_adjusted - _x_acc_decc*tr;
        if (x_progress > _length_of_section){
            current_x_vel = 0.0;
            x_progress = _length_of_section;
        }    
    }
    else{
        current_x_vel = 0.0;
        x_progress = _length_of_section;
    }


    if (t < _time_yaw_acc_decc){
        tr = t;
        yaw_progress =0.5*_yaw_acc_decc*tr*tr;
        current_yaw_vel = _yaw_acc_decc*tr;
        if (yaw_progress > _length_yaw_acc_decc){
            current_yaw_vel = _yaw_vel_adjusted;
            yaw_progress = _length_yaw_acc_decc;
        }
            
    }
    else if (t < (_time_yaw_acc_decc + _time_yaw_vel))
    {
        tr = (t - _time_yaw_acc_decc);
        yaw_progress = _length_yaw_acc_decc + current_yaw_vel*tr;
        current_yaw_vel = _yaw_vel_adjusted;
        if (yaw_progress > (_length_yaw_acc_decc+_length_yaw_vel))
            yaw_progress = (_length_yaw_acc_decc+_length_yaw_vel);
    }        
    else if (t < (_time_yaw_acc_decc + _time_yaw_vel + _time_yaw_acc_decc)){
        tr = (t - _time_yaw_acc_decc - _time_yaw_vel);
        yaw_progress = (_length_yaw_acc_decc + _length_yaw_vel) + _yaw_vel_adjusted*tr - 0.5*_yaw_acc_decc*tr*tr;
        current_yaw_vel = _yaw_vel_adjusted - _yaw_acc_decc*tr;
        if (yaw_progress > _length_of_section_ang)
        {
            current_yaw_vel = 0.0;
            yaw_progress = _length_of_section_ang;
        }
    }
    else{
        current_yaw_vel = 0.0;
        yaw_progress = _length_of_section_ang;
    }
        
    float x_progress_ratio, yaw_progress_ratio;
    if (_length_of_section > 0)
        x_progress_ratio = x_progress / _length_of_section;
    else
        x_progress_ratio = 1.0;

    if (_length_of_section_ang > 0)
        yaw_progress_ratio = yaw_progress/_length_of_section_ang;
    else
        yaw_progress_ratio = 1.0;
    // target_x = start_x + delta_x * progress_on_section

    float next_yaw = _start_yaw + _delta_yaw * yaw_progress_ratio;

    tracking_pid::traj_point tp;

    // next_pose.header.stamp is to be filled by the caller
    tp.pose.header.frame_id = section_start_pose_stamped.header.frame_id;
    tp.pose.pose.position.x = _start_xyz.x + _delta.x * x_progress_ratio;
    tp.pose.pose.position.y = _start_xyz.y + _delta.y * x_progress_ratio;
    tp.pose.pose.position.z = _start_xyz.z + _delta.z * x_progress_ratio;
    tp.velocity.linear.x = current_x_vel;
    // Compute orientation. PID can use it for holonomic robots
    tp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);
    // ROS_INFO("next_yaw: %f", next_yaw);
    _last_yaw = next_yaw;
    tp.velocity.angular.z = _delta_yaw > 0 ? current_yaw_vel : -current_yaw_vel;

    return tp;
}

