//
// Created by nobleo on 11-9-18.
//

#include "controller.h"

namespace tracking_pid
{
Controller::Controller()
{

}

Velocity Controller::update(const Point3D current, const Point3D  goal,
                            const float sample_time)
{
  // Compute location of the point to be controlled
  double theta = current.yaw;
  Point3D newOrigin;
  newOrigin.x = current.x + l * cos(theta);
  newOrigin.y = current.y + l * sin(theta);

  Point3D newCurrent = newOrigin;
  newCurrent.yaw = theta;

  Point3D newGoal;
  if (track_base_link_enabled)
  {
    double theta_goal = goal.yaw;
    Point3D newGoalOrigin;
    newGoalOrigin.x = goal.x + l * cos(theta_goal);
    newGoalOrigin.y = goal.y + l * sin(theta_goal);
    newGoalOrigin.yaw = 0;
    newGoal =  newGoalOrigin;
    newGoal.yaw = theta_goal;
  }
  else
  {
    newGoal = goal;
  }

  // Compute errorPose between controlPose and goalPose
  Point3D error;
  error.yaw = newGoal.yaw - newCurrent.yaw;
  error.x =  (newGoal.x - newCurrent.x) * cos(newCurrent.yaw) + (newGoal.y - newCurrent.y) * sin(newCurrent.yaw); // TODO radian to degree
  error.y = -(newGoal.x - newCurrent.x) * sin(newCurrent.yaw) + (newGoal.y - newCurrent.y) * cos(newCurrent.yaw); 


  //***** Feedback control *****//
  if (!((Kp_long <= 0. && Ki_long <= 0. && Kd_long <= 0.) ||
        (Kp_long >= 0. && Ki_long >= 0. && Kd_long >= 0.)))  // All 3 gains should have the same sign
    printf("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  if (!((Kp_lat <= 0. && Ki_lat <= 0. && Kd_lat <= 0.) ||
        (Kp_lat >= 0. && Ki_lat >= 0. && Kd_lat >= 0.)))  // All 3 gains should have the same sign
    printf("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  if (!((Kp_ang <= 0. && Ki_ang <= 0. && Kd_ang <= 0.) ||
        (Kp_ang >= 0. && Ki_ang >= 0. && Kd_ang >= 0.)))  // All 3 gains should have the same sign
    printf("All three gains (Kp, Ki, Kd) should have the same sign for stability.");

  error_long.at(2) = error_long.at(1);
  error_long.at(1) = error_long.at(0);
  error_long.at(0) = error.x;  // Current error goes to slot 0
  error_lat.at(2) = error_lat.at(1);
  error_lat.at(1) = error_lat.at(0);
  error_lat.at(0) = error.y;  // Current error goes to slot 0
  error_ang.at(2) = error_ang.at(1);
  error_ang.at(1) = error_ang.at(0);



  Point3D error_ang_tf;

  error_ang_tf.yaw = newGoal.yaw - current.yaw;
  error_ang_tf.x =  (newGoal.x - current.x) * cos(current.yaw) 
                  + (newGoal.y - current.y) * sin(current.yaw); // TODO radian to degree
  error_ang_tf.y = -(newGoal.x - current.x) * sin(current.yaw) 
                  + (newGoal.y - current.y) * cos(current.yaw); 
  // Current error goes to slot 0
  double angle_error = std::atan2(error_ang_tf.y, error_ang_tf.x);
  if (l < 0)
  {
    angle_error += M_PI;
    angle_error *= -1;
  }
  error_ang.at(0) = normalize_angle(angle_error);  // TODO


  // integrate the error
  error_integral_long += error_long.at(0) * sample_time;
  error_integral_lat += error_lat.at(0) * sample_time;
  error_integral_ang += error_ang.at(0) * sample_time;

  // Apply windup limit to limit the size of the integral term
  if (error_integral_long > std::fabs(windup_limit))
    error_integral_long = std::fabs(windup_limit);
  if (error_integral_long < -fabsf(windup_limit))
    error_integral_long = -std::fabs(windup_limit);
  if (error_integral_lat > std::fabs(windup_limit))
    error_integral_lat = std::fabs(windup_limit);
  if (error_integral_lat < -std::fabs(windup_limit))
    error_integral_lat = -std::fabs(windup_limit);
  if (error_integral_ang > std::fabs(windup_limit))
    error_integral_ang = std::fabs(windup_limit);
  if (error_integral_ang < -std::fabs(windup_limit))
    error_integral_ang = -std::fabs(windup_limit);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  if (cutoff_frequency_long != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_long * 6.2832) * sample_time / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_long = 1 / tan_filt;
  }
  if (cutoff_frequency_lat != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_lat * 6.2832) * sample_time / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_lat = 1 / tan_filt;
  }
  if (cutoff_frequency_ang != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_ang * 6.2832) * sample_time / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_ang = 1 / tan_filt;
  }

  filtered_error_long.at(2) = filtered_error_long.at(1);
  filtered_error_long.at(1) = filtered_error_long.at(0);
  filtered_error_long.at(0) = (1 / (1 + c_long * c_long + 1.414 * c_long)) *
                              (error_long.at(2) + 2 * error_long.at(1) + error_long.at(0) -
                               (c_long * c_long - 1.414 * c_long + 1) * filtered_error_long.at(2) -
                               (-2 * c_long * c_long + 2) * filtered_error_long.at(1));

  filtered_error_lat.at(2) = filtered_error_lat.at(1);
  filtered_error_lat.at(1) = filtered_error_lat.at(0);
  filtered_error_lat.at(0) =
    (1 / (1 + c_lat * c_lat + 1.414 * c_lat)) * (error_lat.at(2) + 2 * error_lat.at(1) + error_lat.at(0) -
        (c_lat * c_lat - 1.414 * c_lat + 1) * filtered_error_lat.at(2) -
        (-2 * c_lat * c_lat + 2) * filtered_error_lat.at(1));

  filtered_error_ang.at(2) = filtered_error_ang.at(1);
  filtered_error_ang.at(1) = filtered_error_ang.at(0);
  filtered_error_ang.at(0) =
    (1 / (1 + c_ang * c_ang + 1.414 * c_ang)) * (error_ang.at(2) + 2 * error_ang.at(1) + error_ang.at(0) -
        (c_ang * c_ang - 1.414 * c_ang + 1) * filtered_error_ang.at(2) -
        (-2 * c_ang * c_ang + 2) * filtered_error_ang.at(1));

  // Take derivative of error, first the raw unfiltered data:
  error_deriv_long.at(2) = error_deriv_long.at(1);
  error_deriv_long.at(1) = error_deriv_long.at(0);
  error_deriv_long.at(0) = (error_long.at(0) - error_long.at(1)) / sample_time;
  filtered_error_deriv_long.at(2) = filtered_error_deriv_long.at(1);
  filtered_error_deriv_long.at(1) = filtered_error_deriv_long.at(0);
  filtered_error_deriv_long.at(0) = (1 / (1 + c_long * c_long + 1.414 * c_long)) *
                                    (error_deriv_long.at(2) + 2 * error_deriv_long.at(1) + error_deriv_long.at(0) -
                                     (c_long * c_long - 1.414 * c_long + 1) * filtered_error_deriv_long.at(2) -
                                     (-2 * c_long * c_long + 2) * filtered_error_deriv_long.at(1));

  error_deriv_lat.at(2) = error_deriv_lat.at(1);
  error_deriv_lat.at(1) = error_deriv_lat.at(0);
  error_deriv_lat.at(0) = (error_lat.at(0) - error_lat.at(1)) / sample_time;
  filtered_error_deriv_lat.at(2) = filtered_error_deriv_lat.at(1);
  filtered_error_deriv_lat.at(1) = filtered_error_deriv_lat.at(0);
  filtered_error_deriv_lat.at(0) = (1 / (1 + c_lat * c_lat + 1.414 * c_lat)) *
                                   (error_deriv_lat.at(2) + 2 * error_deriv_lat.at(1) + error_deriv_lat.at(0) -
                                    (c_lat * c_lat - 1.414 * c_lat + 1) * filtered_error_deriv_lat.at(2) -
                                    (-2 * c_lat * c_lat + 2) * filtered_error_deriv_lat.at(1));

  error_deriv_ang.at(2) = error_deriv_ang.at(1);
  error_deriv_ang.at(1) = error_deriv_ang.at(0);
  error_deriv_ang.at(0) = (error_ang.at(0) - error_ang.at(1)) / sample_time;
  filtered_error_deriv_ang.at(2) = filtered_error_deriv_ang.at(1);
  filtered_error_deriv_ang.at(1) = filtered_error_deriv_ang.at(0);
  filtered_error_deriv_ang.at(0) = (1 / (1 + c_ang * c_ang + 1.414 * c_ang)) *
                                   (error_deriv_ang.at(2) + 2 * error_deriv_ang.at(1) + error_deriv_ang.at(0) -
                                    (c_ang * c_ang - 1.414 * c_ang + 1) * filtered_error_deriv_ang.at(2) -
                                    (-2 * c_ang * c_ang + 2) * filtered_error_deriv_ang.at(1));

  // calculate the control effort
  proportional_long = Kp_long * filtered_error_long.at(0);
  integral_long = Ki_long * error_integral_long;
  derivative_long = Kd_long * filtered_error_deriv_long.at(0);

  proportional_lat = Kp_lat * filtered_error_lat.at(0);
  integral_lat = Ki_lat * error_integral_lat;
  derivative_lat = Kd_lat * filtered_error_deriv_lat.at(0);

  proportional_ang = Kp_ang * filtered_error_ang.at(0);
  integral_ang = Ki_ang * error_integral_ang;
  derivative_ang = Kd_ang * filtered_error_deriv_ang.at(0);


  //  //***** Feedforward control *****//
  //  // Transform trajectory velocities from map frame to control-point frame
  //  theta_cp = tf::getYaw(tfControlPose.getRotation());
  //  xvel = cos(theta_cp) * goalPoint.velocity.x + sin(theta_cp) * goalPoint.velocity.y;
  //  yvel = -sin(theta_cp) * goalPoint.velocity.x + cos(theta_cp) * goalPoint.velocity.y;
  //  thvel = goalPoint.velocity.z;
  //  feedforward_long = xvel;
  //  feedforward_lat = yvel;
  //  feedforward_ang = thvel;




  //***** Overall control *****//
  // Controller logic && overall control effort
  control_effort_long = 0;
  control_effort_lat = 0;
  control_effort_ang = 0;
  if (feedback_long_enabled)
    control_effort_long = control_effort_long + proportional_long + integral_long + derivative_long;
  if (feedforward_long_enabled)
    control_effort_long = control_effort_long + feedforward_long;
  if (feedback_lat_enabled)
    control_effort_lat = control_effort_lat + proportional_lat + integral_lat + derivative_lat;
  if (feedforward_lat_enabled)
    control_effort_lat = control_effort_lat + feedforward_lat;
  if (feedback_ang_enabled)
    control_effort_ang = control_effort_ang + proportional_ang + integral_ang + derivative_ang;
  if (feedforward_ang_enabled)
    control_effort_ang = control_effort_ang + feedforward_ang;

  // Apply saturation limits
  if (control_effort_long > upper_limit)
    control_effort_long = upper_limit;
  else if (control_effort_long < lower_limit)
    control_effort_long = lower_limit;

  if (control_effort_lat > upper_limit)
    control_effort_lat = upper_limit;
  else if (control_effort_lat < lower_limit)
    control_effort_lat = lower_limit;

  if (control_effort_ang > ang_upper_limit)
    control_effort_ang = ang_upper_limit;
  else if (control_effort_ang < ang_lower_limit)
    control_effort_ang = ang_lower_limit;


  // Couple angular loop with forwards loop
  if (coupling_ang_long_enabled)
  {
    double scale_long_control_1min = (std::fabs(error_ang.at(0)) - dead_zone_yaw_error_cal)
                                     / (max_yaw_error_cal - dead_zone_yaw_error_cal);
    if (scale_long_control_1min < 0.0) scale_long_control_1min = 0.0;
    if (scale_long_control_1min > 1.0) scale_long_control_1min = 1.0;
    scale_long_control = 1.0 - scale_long_control_1min;
  }
  else
  {
    scale_long_control = 1.0;
  }

  Velocity output_combined;
  // Generate twist message

  output_combined.x = scale_long_control * control_effort_long;
  output_combined.yaw = std::copysign(1.0, l) * control_effort_ang;  // Take the sign of l for the lateral control effort


  // Publish control effort if controller enabled
  if (!enabled)  // Do nothing and reset integral action
  {
    error_integral_long = 0.0;
    error_integral_lat = 0.0;
    error_integral_ang = 0.0;
  }
  // printf("errors (in cm/deg): (%.2f, %.2f, %.2f)", error_x*100, error_y*100,error_th);
  return output_combined;
}


void Controller::selectMode(ControllerMode mode)
{
  switch (mode)
  {
  case ControllerMode::frontAxleLateral:
    // Front axle lateral controller (default)
    l = 0.5;
    feedback_long_enabled = true;
    feedback_lat_enabled = true;
    feedback_ang_enabled = false;
    feedforward_long_enabled = true;
    feedforward_lat_enabled = true;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::rearAxleLateral:
    // Rear axle lateral control
    l = 0.0;  // To prevent singular configuration
    feedback_long_enabled = true;
    feedback_lat_enabled = true;
    feedback_ang_enabled = false;
    feedforward_long_enabled = true;
    feedforward_lat_enabled = true;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::rearAxleAngular:
    // Rear axle angular controller
    l = 0.0;
    feedback_long_enabled = true;
    feedback_lat_enabled = false;
    feedback_ang_enabled = true;
    feedforward_long_enabled = false;
    feedforward_lat_enabled = false;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::fixOrientation:
    // Fix orientation controller
    l = 0.0;
    feedback_long_enabled = false;
    feedback_lat_enabled = false;
    feedback_ang_enabled = true;
    feedforward_long_enabled = false;
    feedforward_lat_enabled = false;
    feedforward_ang_enabled = true;
    break;
  }

  printParameters();
}

void Controller::printParameters()
{
  printf("CONTROLLER PARAMETERS");
  printf("-----------------------------------------");
  printf("Controller enabled: %i", enabled);
  printf("Controller DEBUG enabled: %i", debug_enabled);
  printf("Distance L: %f", l);
  printf("Feedback (long, lat, ang): (%i, %i, %i)", feedback_long_enabled, feedback_lat_enabled,
           feedback_ang_enabled);
  printf("Feedforward (long, lat, ang): (%i, %i, %i)", feedforward_long_enabled, feedforward_lat_enabled,
           feedforward_ang_enabled);
  printf("Longitudinal gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_long, Ki_long, Kd_long);
  printf("Lateral gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_lat, Ki_lat, Kd_lat);
  printf("Angular gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_ang, Ki_ang, Kd_ang);

  printf("Robot type (holonomic): (%i)", holonomic_robot_enable);

  printf("Track base link: (%i)", track_base_link_enabled);

  if (cutoff_frequency_long == -1)  // If the cutoff frequency was not specified by the user
    printf("LPF cutoff frequency: 1/4 of sampling rate");
  else
    printf("LPF cutoff frequency: %f", cutoff_frequency_long);

  printf("Integral-windup limit: %f", windup_limit);
  printf("Saturation limits xy: %f/%f", upper_limit, lower_limit);
  printf("Saturation limits ang: %f/%f", ang_upper_limit, ang_lower_limit);
  printf("map frame: %s", map_frame.c_str());
  printf("base_link frame: %s", base_link_frame.c_str());
  printf("-----------------------------------------");
}


void Controller::setEnabled(bool value)
{
  printf("Controller::setEnabled(%d)", value);
  enabled = value;
}


void Controller::setTrackBaseLink(bool value)
{
  printf("Controller::setTrackBaseLink(%d)", value);
  track_base_link_enabled = value;
}

}  // namespace tracking_pid
