/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "cubic_spline_interpolator.h"

namespace path_smoothing
{

  CubicSplineInterpolator::CubicSplineInterpolator(
    double pointsPerUnit,
    unsigned int skipPoints,
    bool useEndConditions,
    bool useMiddleConditions)
    :
      pointsPerUnit_(pointsPerUnit),
      skipPoints_(skipPoints),
      useEndConditions_(useEndConditions),
      useMiddleConditions_(useMiddleConditions)
  {
  }

  CubicSplineInterpolator::~CubicSplineInterpolator()
  {
  }


  bool CubicSplineInterpolator::calculateTime(){
    if(mDensifiedPath.size() <= 1){
      // ROS_ERROR_THROTTLE(1.0, "No mDensifiedPath, please make sure you call densifyPath() first");
      return false;
    }

    if(fabs(_x_vel)< 0.0001 || fabs(_yaw_vel)< 0.0001){
      // ROS_INFO_THROTTLE(1.0, "No target speed , please make sure you call setMaxVelociy() first");
      return false;
    }
    mAllSegmentsDuration = 0.0;
    mSegmentsDurations.clear();
    float dis = 0.0;
    float ang = 0.0;
    float duration_time = 0.0;

    for(size_t idx = 1; idx < mDensifiedPath.size(); idx++){
      float dis = calcDistance(mDensifiedPath, idx);
      float ang = shortest_angular_distance(mDensifiedPath[idx].yaw, mDensifiedPath[idx + 1].yaw);
      float duration_time = 1.5 * std::max(fabs(dis / _x_vel), (double)0.0);
      // printf("dis: %f duration: %f", dis, duration_time);
      mSegmentsDurations.push_back(duration_time);
      // if( idx <= 8) 
      mAllSegmentsDuration += duration_time;
    }
    mTimeRatios.push_back(0.0);
    for(size_t idx = 0; idx < mSegmentsDurations.size(); idx++){
      mTimeRatios.push_back(mTimeRatios.back() + mSegmentsDurations[idx] / mAllSegmentsDuration);
      // printf("ratio: %f", mTimeRatios.back());
    }
    mAllSegmentsStartTime = Now();

    mAllSegmentsEndTime   = mAllSegmentsStartTime  + std::chrono::milliseconds(int(mAllSegmentsDuration * 1000));
    return true;
  }

  void CubicSplineInterpolator::densifyPath(
    const Path& path,
    Path& densifiedPath)
  {
    densifiedPath.time = path.time;

    densifyPath(path.poses, densifiedPath.poses);
  }


  void CubicSplineInterpolator::densifyPath(
    const std::vector<Point3D>& path,
    std::vector<Point3D>& densifiedPath)
  {
    // clear new smoothed path vector in case it's not empty
    densifiedPath.clear();

    // set skipPoints_ to 0 if the path contains has too few points
    unsigned int oldSkipPoints = skipPoints_;
    skipPoints_ = std::min<int>(path.size() - 2, skipPoints_);

    // create cummulative distances vector
    std::vector<double> distanceRatios;
    calcDistanceRatios(path, distanceRatios);

    // create temp pose
    Point3D pose;

    unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);

    densifiedPath.resize(numPoints);

    // interpolate points on the smoothed path using the points in the original path
    for (unsigned int i = 0; i < numPoints; i++)
    {
      double u = static_cast<double>(i) / (numPoints-1);
      interpolatePoint(path, distanceRatios, pose, u);

      if (std::isnan(pose.x) || std::isnan(pose.y))
        pose = densifiedPath[std::max(static_cast<int>(i)-1, 0)];
      densifiedPath[i] = pose;
    }

    // copy start and goal orientations to smoothed path
    densifiedPath.front().yaw = path.front().yaw;
    densifiedPath.back().yaw = path.back().yaw;

    // interpolate orientations of intermediate poses
    for (unsigned int i = 1; i < densifiedPath.size()-1; i++)
    {
      double dx = densifiedPath[i+1].x - densifiedPath[i].x;
      double dy = densifiedPath[i+1].y - densifiedPath[i].y;
      double th = atan2(dy, dx);
      densifiedPath[i].yaw = th;
      printf("first position x: %f,  y: %f angle: %f\n", densifiedPath[i].x, densifiedPath[i].y, th );
    }
    mDensifiedPath = densifiedPath;
    mDistanceRatios = distanceRatios;
    // revert skipPoints to original value
    skipPoints_ = oldSkipPoints;
  }

/**
 * @brief 
 * 
 * @param group : 属于第i段
 * @param relative_ratio ： 在第i段中所占的时间比例
 */
 TrajectoryPoint CubicSplineInterpolator::interpolatePoint(double ratio)
  {
      TrajectoryPoint tp;
      
      if(mDensifiedPath.size() <= 2){
        // ROS_INFO_THROTTLE(1.0, "your path is too short and guess densify step is missiing");
        return tp;
      }
      unsigned int group = findGroup(mTimeRatios, ratio);
      float relative_ratio = calcRelativeRatio(mTimeRatios, group, ratio);
      // printf("group: %d, relative_ratio: %f\n", group, relative_ratio);
      float cur_seg_ratio = mTimeRatios[group +1 ] - mTimeRatios[group];

      double p_a = calcAlphaCoeff(relative_ratio);
      double p_b = calcBetaCoeff(relative_ratio);
      double p_c = calcGammaCoeff(relative_ratio, cur_seg_ratio);
      double p_d = calcDeltaCoeff(relative_ratio, cur_seg_ratio);

      double v_a = calcVelocityAlphaCoeff(relative_ratio, cur_seg_ratio);
      double v_b = calcVelocityBetaCoeff (relative_ratio, cur_seg_ratio);
      double v_c = calcVelocityGammaCoeff(relative_ratio);
      double v_d = calcVelocityDeltaCoeff(relative_ratio);
      // printf("~~~~~~~~~~~group: %d ratio: %f relative_ratio:%f\n", group, ratio, relative_ratio);
      // printf("p coeefs: %f, %f, %f, %f\n", p_a, p_b, p_c, p_d);
      std::vector<double> grad, nextGrad;
      calcPointGradient(mDensifiedPath, mTimeRatios, group, grad);
      calcPointGradient(mDensifiedPath, mTimeRatios, group+1, nextGrad);
      // printf("grad: %f %f  nextGrad: %f :%f\n", grad[0], nextGrad[0], grad[1], nextGrad[1]);

      tp.time = Now(); 
      tp.pose.x =
        + p_a * mDensifiedPath[group    *(skipPoints_+1)].x
        + p_b * mDensifiedPath[(group+1)*(skipPoints_+1)].x
        + p_c * grad[0]
        + p_d * nextGrad[0];

      tp.pose.y =
        + p_a * mDensifiedPath[group    *(skipPoints_+1)].y
        + p_b * mDensifiedPath[(group+1)*(skipPoints_+1)].y
        + p_c * grad[1]
        + p_d * nextGrad[1];

      float end_yaw   = mDensifiedPath[(group+1)*(skipPoints_+1)].yaw;
      float start_yaw = mDensifiedPath[  group  *(skipPoints_+1)].yaw;
      float delta_yaw = shortest_angular_distance(start_yaw, end_yaw) ;
      float next_yaw  = start_yaw + delta_yaw * relative_ratio;
      tp.pose.yaw = next_yaw;

      float x_vel = + v_a * mDensifiedPath[group*(skipPoints_+1)].x
        + v_b * mDensifiedPath[(group+1)*(skipPoints_+1)].x
        + v_c * grad[0]
        + v_d * nextGrad[0];

      float y_vel = + v_a * mDensifiedPath[group*(skipPoints_+1)].y
        + v_b * mDensifiedPath[(group+1)*(skipPoints_+1)].y
        + v_c * grad[1]
        + v_d * nextGrad[1];

      float robot_vel = x_vel * cos(next_yaw) + y_vel * sin(next_yaw);
      float d_x = mDensifiedPath[(group+1)*(skipPoints_+1)].x - mDensifiedPath[group*(skipPoints_+1)].x;
      float d_y = mDensifiedPath[(group+1)*(skipPoints_+1)].y - mDensifiedPath[group*(skipPoints_+1)].y;
      float robot_x = d_x * cos(next_yaw) + d_y * sin(next_yaw);

      tp.velocity.x = robot_x /  mSegmentsDurations[group];// fabs(robot_vel) > _x_vel ?  sign(robot_vel) * _x_vel : robot_vel;


      float yaw_vel = delta_yaw / mSegmentsDurations[group];
      tp.velocity.yaw = yaw_vel;

      return tp;

  }
  
  int CubicSplineInterpolator::sign(float num){
    if(num < 0){
      return -1;
    }
    return 1;
  }
  
  void CubicSplineInterpolator::interpolatePoint(
    const std::vector<Point3D>& path,
    const std::vector<double>& distanceRatios,
    Point3D& point,
    double ratio)
  {
    unsigned int group = findGroup(distanceRatios, ratio);
    float relative_ratio = calcRelativeRatio(distanceRatios, group, ratio);
    float cur_seg_ratio = distanceRatios[group +1 ] - distanceRatios[group];
    double p_a = calcAlphaCoeff(relative_ratio);
    double p_b = calcBetaCoeff(relative_ratio);
    double p_c = calcGammaCoeff(relative_ratio, cur_seg_ratio);
    double p_d = calcDeltaCoeff(relative_ratio, cur_seg_ratio);
    std::vector<double> grad, nextGrad;
    calcPointGradient(path, distanceRatios, group, grad);
    calcPointGradient(path, distanceRatios, group+1, nextGrad);

    point.x =
      + p_a * path[group*(skipPoints_+1)].x
      + p_b * path[(group+1)*(skipPoints_+1)].x
      + p_c * grad[0]
      + p_d * nextGrad[0];

    point.y =
      + p_a * path[group*(skipPoints_+1)].y
      + p_b * path[(group+1)*(skipPoints_+1)].y
      + p_c * grad[1]
      + p_d * nextGrad[1];
  }


  void CubicSplineInterpolator::calcDistanceRatios(
    const std::vector<Point3D> path,
    std::vector<double>& distanceRatios)
  {
    distanceRatios.clear();
    distanceRatios.push_back(0);
    double totalDist = calcTotalDistance(path);
    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1){
      float ratio = distanceRatios.back() + calcDistance(path, i) / totalDist;
      distanceRatios.push_back(ratio);
    }
      
  }


  double CubicSplineInterpolator::calcTotalDistance(
    const std::vector<Point3D>& path)
  {
    double totalDist = 0;

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
      totalDist += calcDistance(path, i);

    return totalDist;
  }


  double CubicSplineInterpolator::calcDistance(
    const std::vector<Point3D>& path,
    unsigned int idx)
  {
    if (idx <= 0 || idx >=path.size())
      return 0;

    double dist =
      hypot(
        path[idx].x - path[idx-skipPoints_-1].x,
        path[idx].y - path[idx-skipPoints_-1].y);

    return dist;
  }


  double CubicSplineInterpolator::calcAlphaCoeff(
    double relativeRatio)
  {
    double alpha =
      + 2 * pow(relativeRatio, 3)
      - 3 * pow(relativeRatio, 2)
      + 1;

    return alpha;
  }


  double CubicSplineInterpolator::calcBetaCoeff(
    double relativeRatio)
  {
    double beta =
      - 2 * pow(relativeRatio, 3)
      + 3 * pow(relativeRatio, 2);

    return beta;
  }


  double CubicSplineInterpolator::calcGammaCoeff(
    double relativeRatio, 
    double curSegmentRatio)
  {
    double gamma =
      (pow(relativeRatio, 3)
       - 2 * pow(relativeRatio, 2) + relativeRatio)
      * curSegmentRatio;
    return gamma;
  }


  double CubicSplineInterpolator::calcDeltaCoeff(
    double relativeRatio, 
    double curSegmentRatio)
  {
    double delta =
      (pow(relativeRatio, 3)
       - pow(relativeRatio, 2))
      * curSegmentRatio;

      return delta;
  }

  double CubicSplineInterpolator::calcVelocityAlphaCoeff(
    double relativeRatio, 
    double curSegmentRatio)
  {
    double alpha =
      6 * (pow(relativeRatio, 2)
       - pow(relativeRatio, 1))
      / curSegmentRatio;

    return alpha;
  }


  double CubicSplineInterpolator::calcVelocityBetaCoeff(
    double relativeRatio, 
    double curSegmentRatio)
  {
    double beta =
     6 * ( - pow(relativeRatio, 2)
       + pow(relativeRatio, 1))
      / curSegmentRatio;

    return beta;
  }


  double CubicSplineInterpolator::calcVelocityGammaCoeff(
    double relativeRatio)
  {
    double gamma =
      + 3 * pow(relativeRatio, 2)
      - 4 * pow(relativeRatio, 1)
      + 1;

    return gamma;
  }


  double CubicSplineInterpolator::calcVelocityDeltaCoeff(
    double relativeRatio)
  {
    double delta =
      + 3 * pow(relativeRatio, 2)
      - 2 * pow(relativeRatio, 1);

      return delta;
  }


  double CubicSplineInterpolator::calcRelativeRatio(
    const std::vector<double>& distanceRatios,
    const unsigned int idx,
    const double ratio)
  {
    double relDist =
      (ratio - distanceRatios[idx])
      / (distanceRatios[idx+1] - distanceRatios[idx]);
    return relDist;
  }


  void CubicSplineInterpolator::calcPointGradient(
    const std::vector<Point3D>& path,
    const std::vector<double>& distanceRatios,
    unsigned int idx,
    std::vector<double>& gradient)
  {
    double dx, dy, du;
    gradient.assign(2, 0);
    // printf("useEndConditions_ : %d useMiddleConditions_: %d", useEndConditions_, useMiddleConditions_);
    // use either yaw or interpolation to find gradient of points
    if ((useEndConditions_ && (idx == 0 || idx == distanceRatios.size()-1))
      || useMiddleConditions_)
    {
      double th = path[idx*(skipPoints_+1)].yaw;
      int sign = (fabs(th) < M_PI / 2) ? 1 : -1;

      gradient[0] = sign * calcTotalDistance(path)
        * sqrt(1 + pow(tan(th),2)) / (1 + pow(tan(th), 2));
      gradient[1] = tan(th) * gradient[0];
    }
    else  // gradient interpolation using original points
    {
      if (idx == 0 || idx == distanceRatios.size()-1)
        return;

      dx = path[(idx)*(skipPoints_+1)].x - path[(idx-1)*(skipPoints_+1)].x;
      dy = path[(idx)*(skipPoints_+1)].y - path[(idx-1)*(skipPoints_+1)].y;
      du = distanceRatios[idx] - distanceRatios[idx-1];

      gradient[0] =  dx / du;
      gradient[1] =  dy / du;
    }
  }


  unsigned int CubicSplineInterpolator::findGroup(
    const std::vector<double>& distanceRatios,
    double ratio)
  {
    unsigned int i;
    for (i = 0; i < distanceRatios.size()-1; i++)
    {
      if (ratio <= distanceRatios[i+1])
        return i;
    }
    return i;
  }

}  // namespace path_smoothing
