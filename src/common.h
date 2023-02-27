#ifndef COMMON_H
#define COMMON_H
#include <cmath>
#include <vector>
#include <chrono>
#include <cstdio>
#define TimeStamp std::chrono::high_resolution_clock::time_point
#define Now() std::chrono::high_resolution_clock::now()
struct Point3D
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

struct Velocity
{
    float x = 0.0;
    float yaw = 0.0;
};


struct TrajectoryPoint{
    TimeStamp time = Now();
    Point3D pose;
    Velocity velocity;
};

struct Path{
    TimeStamp time = Now();
    std::vector<Point3D> poses;
};


/*!
* \brief normalize
*
* Normalizes the angle to be -M_PI circle to +M_PI circle
* It takes and returns radians.
*
*/
static inline double normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0*M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
}


static inline float shortest_angular_distance(float from, float to)
{
    return normalize_angle(to-from);
}


#endif  // COMMON_H

