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

#ifndef PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H
#define PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H

#include "common.h"

namespace path_smoothing
{

  class CubicSplineInterpolator
  {
    public:

      CubicSplineInterpolator(
        double pointsPerUnit = 5.0,
        unsigned int skipPoints = 0,
        bool useEndConditions = false,
        bool useMiddleConditions = false);

      ~CubicSplineInterpolator();

      void setMaxVelociy(float target_x_vel, float target_yaw_vel){
          _x_vel = target_x_vel;
          _yaw_vel = target_yaw_vel;
      }

      bool calculateTime();

      void densifyPath(
        const Path& path, Path& smoothedPath);

      void densifyPath(
        const std::vector<Point3D>& path,
        std::vector<Point3D>& smoothedPath);
      TrajectoryPoint interpolatePoint(double ratio);

      void interpolatePoint(
        const std::vector<Point3D>& path,
        const std::vector<double>& distanceRatios,
        Point3D& point,
        double ratio);

      void calcDistanceRatios(
        const std::vector<Point3D> path,
        std::vector<double>& distanceRatios);

      double calcTotalDistance(const std::vector<Point3D>& path);

      double calcDistance(
        const std::vector<Point3D>& path,
        unsigned int idx);

      double calcAlphaCoeff(
        double relativeRatio);

      double calcBetaCoeff(
        double relativeRatio);

      double calcGammaCoeff(
        double relativeRatio, 
        double curSegmentRatio);

      double calcDeltaCoeff(
        double relativeRatio, 
        double curSegmentRatio);

      double calcVelocityAlphaCoeff(
        double relativeRatio, 
      double curSegmentRatio);

      double calcVelocityBetaCoeff(
        double relativeRatio, 
        double curSegmentRatio);

      double calcVelocityGammaCoeff(
        double relativeRatio);

      double calcVelocityDeltaCoeff(
        double relativeRatio);


      double calcRelativeRatio(
        const std::vector<double>& distanceRatios,
        unsigned int idx,
        double ratio);

      void calcPointGradient(
        const std::vector<Point3D>& path,
        const std::vector<double>& distanceRatios,
        unsigned int idx, std::vector<double>& gradient);

      unsigned int findGroup(
        const std::vector<double>& distanceRatios,
        double ratio);
      unsigned int findGroup(
        double  ratio);
      double getPointsPerUnit() {return pointsPerUnit_;}
      unsigned int skipPoints() {return skipPoints_;}
      bool getUseEndConditions() {return useEndConditions_;}
      bool getUseMiddleConditions() {return useMiddleConditions_;}

      void setPointsPerUnit(double ppu) {pointsPerUnit_ = ppu;}
      void setSkipPoints(unsigned int sp) {skipPoints_ = sp;}
      void setUseEndConditions(bool uec) {useEndConditions_ = uec;}
      void setUseMiddleConditions(bool umc) {useMiddleConditions_ = umc;}

      int sign(float num);

      float mAllSegmentsDuration;
      std::vector<double> mSegmentsDurations;
      TimeStamp mAllSegmentsStartTime; 
      TimeStamp mAllSegmentsEndTime;  

    private:
      double pointsPerUnit_;
      unsigned int skipPoints_;
      bool useEndConditions_;
      bool useMiddleConditions_;
      std::vector<Point3D> mDensifiedPath;
      std::vector<double> mDistanceRatios;
      std::vector<double> mTimeRatios;

      float _x_vel;  // type: float
      float _yaw_vel;
  };

}  // namespace path_smoothing

#endif  // PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H
