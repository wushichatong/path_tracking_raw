#include "path_interpolator.h"

PathInterpolator::PathInterpolator(/* args */):

{
    _rate = 50.0;
    _target_x_vel   = 1.0; //To be overridden by parameters;
    _target_x_acc   = 0.2;
    _target_yaw_vel = 0.1;
    _target_yaw_acc = 0.1;
}

PathInterpolator::~PathInterpolator()
{

}
bool PathInterpolator::is_free(){
    return _is_free;
}
void PathInterpolator::start_path(){
    printf("start_path()");
    _is_free = false;
    _timer = node_priv.createTimer(_rate, &PathInterpolator::_update_target, this);
}

void PathInterpolator::stop_path(){
    printf("stop_path()");
    _timer.stop();
}

void PathInterpolator::continue_path(ros::Time start_time){

}

void PathInterpolator::_process_pause(const std_msgs::Bool& bool_msg){
    if(bool_msg.data && bool_msg.data != _paused)
    {
        printf("Pausing path_interpolator");
        printf("No acceleration limits implemented when pausing!");
        _paused = bool_msg.data;
    }else if(!bool_msg.data && bool_msg.data != _paused){
        printf("Unpausing path_interpolator");
        ros::Time resume_time = ros::Time::now() - ros::Duration(1.0 / _rate); 
        _paused = bool_msg.data;
        continue_path(resume_time);
    }
}

void PathInterpolator::_accept_path_from_topic(const nav_msgs::Path& path_msg)
{
    printf("%ld", path_msg.poses.size());
    _process_path(path_msg);
}


nav_msgs::Path PathInterpolator::_path_smoother(const nav_msgs::Path& raw)
{
    float weight_data = 0.5;
    float weight_smooth = 0.2;
    nav_msgs::Path smooth = raw;
    float tolerance = 0.000001;
    size_t length = raw.poses.size();
    assert(length >= 3);
    float change = tolerance;
    float d1, d2;
    while (change >= tolerance){
        change = 0;
        for(size_t idx = 1; idx < length - 1; idx++){
            // x dirction
            d1 = weight_data * (raw.poses[idx].pose.position.x - smooth.poses[idx].pose.position.x);
            d2 = weight_smooth * (smooth.poses[idx - 1].pose.position.x + smooth.poses[idx + 1].pose.position.x 
                                        - 2 * smooth.poses[idx].pose.position.x);
            change += fabs(d1 + d2);
            smooth.poses[idx].pose.position.x += d1 + d2;
            // y dirction
            d1 = weight_data * (raw.poses[idx].pose.position.y - smooth.poses[idx].pose.position.y);
            d2 = weight_smooth * (smooth.poses[idx - 1].pose.position.y + smooth.poses[idx + 1].pose.position.y
                                        - 2 * smooth.poses[idx].pose.position.y);
            change += fabs(d1 + d2);
            smooth.poses[idx].pose.position.y += d1 + d2;
        }
    }
    for(size_t idx = 0; idx < length; idx++){
        printf("raw: (%f, %f), interpolator:(%f, %f)", raw.poses[idx].pose.position.x, raw.poses[idx].pose.position.y,
                                            smooth.poses[idx].pose.position.x, smooth.poses[idx].pose.position.y);
    }          
    return smooth; 
}

void PathInterpolator::_process_path(const nav_msgs::Path& path_msg)
{
    printf("_process_path(...). Path has %ld poses", path_msg.poses.size());
    if(path_msg.poses.size() == 0){
        printf("There are no poses in the given path with header");
        return ;
    }
    // If empty frame_ids are supplied, use the global headers frame_id
    std::vector<geometry_msgs::PoseStamped> undifined_poses;
    for(size_t idx = 0; idx < path_msg.poses.size(); idx++){
        geometry_msgs::PoseStamped pose = path_msg.poses[idx];
        if(pose.header.frame_id == ""){
            pose.header.frame_id = path_msg.header.frame_id;
        }
    }
    //step 1 光滑一下
    nav_msgs::Path smooth_path_msg = _path_smoother(path_msg);

    nav_msgs::Path smoothedPath;

        // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    // step 2 稠密化路径
    _cubic_spline_interpolator.densifyPath(smooth_path_msg, smoothedPath);



    _raw_path_pub.publish(path_msg);
    // TODO  All the header_ids are the same, 
    _smooth_path_pub.publish(smoothedPath);

    // _client.waitForServer();
    // mbf_msgs::ExePathGoal goal;
    // goal.path = smoothedPath;

    // _client.sendGoal(goal);

    // bool finished_before_timeout = _client.waitForResult(ros::Duration(30.0));

    // return;


    float target_x_vel   = node_priv.param("target_x_vel", 1.0);
    float target_x_acc   = node_priv.param("target_x_acc", 1.0);
    float target_yaw_vel = node_priv.param("target_yaw_vel", 1.0);
    float target_yaw_acc = node_priv.param("target_yaw_acc", 1.0);
    if(fabs(target_x_vel - 0.0) < 0.0001 || fabs(target_yaw_vel - 0.0) < 0.0001){
        printf("Ignoring ~target_x_vel of %f, ~target_yaw_vel of %f, keeping %f, %f, consider using the pause function", 
        target_x_vel, target_yaw_vel, _target_x_vel, _target_yaw_vel);
    }else{
        _target_x_vel   = target_x_vel;
        _target_x_acc   = target_x_acc;
        _target_yaw_vel = target_yaw_vel;
        _target_yaw_acc = target_yaw_acc;
    }

    _latest_path_msg.header = smoothedPath.header;
    _path_poses = smoothedPath.poses;

    // 将速度约束放到插值函数中
    _cubic_spline_interpolator.setMaxVelociy(_target_x_vel, _target_yaw_vel);

    // 计算每一段的时间和总时间
    _cubic_spline_interpolator.calculateTime();

    printf("smoothed_pose size: %ld", _path_poses.size());

    if(!_timer.isValid() || !_timer.hasStarted()){
        start_path();
    }

}

void PathInterpolator::_update_target(const ros::TimerEvent& event)
{


    if(_path_poses.size() == 0){
        printf_THROTTLE(1.0, "No path poses set");
        return;
    }

    if(_paused){
        printf_THROTTLE(5.0, "Path_interpolator is paused");
        return;
    }


    if( ros::Time::now() > _cubic_spline_interpolator.mAllSegmentsEndTime)  // or when past end time of current section, go to next
    {
        _dense_pub.publish(_latest_path_msg);
        _timer.stop();
        printf("Trajectory finished, timer stop");
        return;

    }

    
    ros::Duration duration_on_section = event.current_real - _cubic_spline_interpolator.mAllSegmentsStartTime;
    // printf_THROTTLE(1.0, "this duration: %f / %f",duration_on_section.toSec(),  _current_section.duration_for_section.toSec());

    float progress_on_section =  duration_on_section.toSec() / _cubic_spline_interpolator.mAllSegmentsDuration;

    // float progress_on_section = count / 10.0;
    tracking_pid::traj_point tp = _cubic_spline_interpolator.interpolatePoint(progress_on_section);

    tp.pose.header.stamp = event.current_real;

    // TODO: Rotate in the corners, using controller mode 3 tp.controller.data = 3

    // Remember the last interpolated sub-goal on our way to the next waypoint in the Path
    _latest_subgoal_pose = tp.pose;
    _latest_subgoal_vel  = tp.velocity;
    // printf("position x: %f,  y: %f", tp.pose.pose.position.x, tp.pose.pose.position.y );
    // printf("velocity x: %f, angular z: %f", tp.velocity.linear.x, tp.velocity.angular.z );

}






