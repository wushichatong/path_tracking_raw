

#include "cubic_spline_interpolator.h"
#include <assert.h>
#include <queue>

class PathInterpolator
{
private:



    std::string robot_frame;
    bool _paused;
    bool _is_free = true;
    float _rate;

    float _target_x_vel;
    float _target_x_acc;
    float _target_yaw_vel;
    float _target_yaw_acc;


    // create a cubic spline interpolator
    path_smoothing::CubicSplineInterpolator _cubic_spline_interpolator;
    std::vector<Point3D> _path_poses;
    Path _latest_path;
public:
    PathInterpolator(/* args */);
    ~PathInterpolator();
    bool is_free();

    void start_path();
    void stop_path();
    void continue_path(TimeStamp start_time);
    void _process_pause(const bool& bool_msg);
    void _accept_path_from_topic(const Path& path_msg);

    void _preempt_goal();
    Path _path_smoother(const Path& path_msg);
    void _process_path(const Path& path_msg);
    void _update_target(const TimeStamp& time);
    void _publish_marker(const Point3D pose_stamped);


};
