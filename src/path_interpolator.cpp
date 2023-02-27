#include "path_interpolator.h"

PathInterpolator::PathInterpolator(/* args */)
{
    _rate = 50.0;
    _target_x_vel   = 1.0; 
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
    // _timer = node_priv.createTimer(_rate, &PathInterpolator::_update_target, this);
}

void PathInterpolator::stop_path(){
    printf("stop_path()");
    // _timer.stop();
}

void PathInterpolator::continue_path(TimeStamp start_time){

}

void PathInterpolator::_process_pause(const bool& pause){
    if(pause && pause != _paused)
    {
        printf("Pausing path_interpolator");
        printf("No acceleration limits implemented when pausing!");
        _paused = pause;
    }else if(!pause && pause != _paused){
        printf("Unpausing path_interpolator");
        // TODO
        // TimeStamp resume_time = TimeStamp::now() - ros::Duration(1.0 / _rate); 
        _paused = pause;
        // continue_path(resume_time);
    }
}

void PathInterpolator::_accept_path_from_topic(const Path& path)
{
    printf("%ld", path.poses.size());
    _process_path(path);
}


Path PathInterpolator::_path_smoother(const Path& raw)
{
    float weight_data = 0.5;
    float weight_smooth = 0.2;
    Path smooth = raw;
    float tolerance = 0.000001;
    size_t length = raw.poses.size();
    assert(length >= 3);
    float change = tolerance;
    float d1, d2;
    while (change >= tolerance){
        change = 0;
        for(size_t idx = 1; idx < length - 1; idx++){
            // x dirction
            d1 = weight_data * (raw.poses[idx].x - smooth.poses[idx].x);
            d2 = weight_smooth * (smooth.poses[idx - 1].x + smooth.poses[idx + 1].x 
                                        - 2 * smooth.poses[idx].x);
            change += fabs(d1 + d2);
            smooth.poses[idx].x += d1 + d2;
            // y dirction
            d1 = weight_data * (raw.poses[idx].y - smooth.poses[idx].y);
            d2 = weight_smooth * (smooth.poses[idx - 1].y + smooth.poses[idx + 1].y
                                        - 2 * smooth.poses[idx].y);
            change += fabs(d1 + d2);
            smooth.poses[idx].y += d1 + d2;
        }
    }
    for(size_t idx = 0; idx < length; idx++){
        printf("raw: (%f, %f), interpolator:(%f, %f)", raw.poses[idx].x, raw.poses[idx].y,
                                            smooth.poses[idx].x, smooth.poses[idx].y);
    }          
    return smooth; 
}

void PathInterpolator::_process_path(const Path& path)
{
    printf("_process_path(...). Path has %ld poses", path.poses.size());
    if(path.poses.size() == 0){
        printf("There are no poses in the given path with header");
        return ;
    }
    // If empty frame_ids are supplied, use the global headers frame_id
    std::vector<Point3D> undifined_poses;
    for(size_t idx = 0; idx < path.poses.size(); idx++){
        Point3D pose = path.poses[idx];
    }
    //step 1 光滑一下
    Path smooth_path_msg = _path_smoother(path);

    Path smoothedPath;

    // step 2 稠密化路径
    _cubic_spline_interpolator.densifyPath(smooth_path_msg, smoothedPath);
    _latest_path.time = smoothedPath.time;
    _path_poses = smoothedPath.poses;

    // 将速度约束放到插值函数中
    _cubic_spline_interpolator.setMaxVelociy(_target_x_vel, _target_yaw_vel);

    // 计算每一段的时间和总时间
    _cubic_spline_interpolator.calculateTime();

    printf("smoothed_pose size: %ld", _path_poses.size());
    // TODO
    // if(!_timer.isValid() || !_timer.hasStarted()){
    //     start_path();
    // }
}

void PathInterpolator::_update_target(const TimeStamp& time)
{


    if(_path_poses.size() == 0){
        printf("No path poses set");
        return;
    }

    if(_paused){
        printf("Path_interpolator is paused");
        return;
    }


    if( Now() > _cubic_spline_interpolator.mAllSegmentsEndTime)  // or when past end time of current section, go to next
    {
        // _timer.stop();
        printf("Trajectory finished, timer stop");
        return;

    }


    std::chrono::duration<double>  duration_on_section = time - _cubic_spline_interpolator.mAllSegmentsStartTime;
    // printf(1.0, "this duration: %f / %f",duration_on_section.toSec(),  _current_section.duration_for_section.toSec());

    float progress_on_section =  duration_on_section.count() / _cubic_spline_interpolator.mAllSegmentsDuration;

    TrajectoryPoint tp = _cubic_spline_interpolator.interpolatePoint(progress_on_section);

    tp.time = time;

    // TODO: Rotate in the corners, using controller mode 3 tp.controller.data = 3

    // Remember the last interpolated sub-goal on our way to the next waypoint in the Path
    // _latest_subgoal_pose = tp.pose;
    // _latest_subgoal_vel  = tp.velocity;
    // printf("position x: %f,  y: %f", tp.pose.x, tp.pose.y );
    // printf("velocity x: %f, angular z: %f", tp.velocity.linear.x, tp.velocity.angular.z );

}






