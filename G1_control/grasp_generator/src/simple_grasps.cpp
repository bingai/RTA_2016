/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

#include <grasp_generator/simple_grasps.h>
// #include <world_model_msgs/Object.h>


namespace grasp_generator
{

// Constructor
SimpleGrasps::SimpleGrasps(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose) :
  visual_tools_(visual_tools),
  verbose_(verbose),
  grasp_id(0)
{
  ROS_DEBUG_STREAM_NAMED("grasps","Loaded simple grasp generator");
}

// Deconstructor
SimpleGrasps::~SimpleGrasps()
{
}

// get the object height
double SimpleGrasps::get_height(const world_model_msgs::Object &object){
  double height;
  if (object.primitives[0].type == object.primitives[0].BOX) {
    height = object.primitives[0].dimensions[object.primitives[0].BOX_Z];
  }
  else if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    height = object.primitives[0].dimensions[object.primitives[0].CYLINDER_HEIGHT];
  }
  else {
    ROS_ERROR("Cannot get the object height! Use the default value 0 as the object height.");
    height = 0;
  }
  return height;
}

// // get the object radius
// double SimpleGrasps::get_radius(const world_model_msgs::Object &object){
//   double radius;
//   if (object.primitives[0].type == object.primitives[0].BOX) {
//     radius = std::max(object.primitives[0].dimensions[object.primitives[0].BOX_X], object.primitives[0].dimensions[object.primitives[0].BOX_Y]);
//   }
//   else if (object.primitives[0].type == object.primitives[0].CYLINDER) {
//     radius = object.primitives[0].dimensions[object.primitives[0].CYLINDER_RADIUS];
//   }
//   else {
//     ROS_ERROR("Cannot get the object height! Use the default value 0 as the object height.");
//     radius = 0.0;
//   }
//   return radius;
// }

// get the object radius
double SimpleGrasps::get_radius(const world_model_msgs::Object &object, double theta_sampled){
  double radius;
  if (object.primitives[0].type == object.primitives[0].BOX) {
    double X = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_X]);
    double Y = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_Y]);
    // Eigen::Affine3d rot_mat = get_rotation_matrix(object);
    double theta = atan(Y/X);
    if((theta_sampled == 0.5 * M_PI) || theta_sampled == 3*M_PI/2){
      radius = 0.5*object.primitives[0].dimensions[object.primitives[0].BOX_Y];
    }
    else{
      if((theta_sampled >= 0 && theta_sampled < theta) ||
         ((theta_sampled > (2* M_PI - theta)) && (theta_sampled <= 2*M_PI)) ||
         ((theta_sampled > (M_PI - theta)) && (theta_sampled < M_PI + theta))) {
        radius = 0.5*object.primitives[0].dimensions[object.primitives[0].BOX_X];
      }
      else if( ((theta_sampled > theta) && (theta_sampled < (M_PI - theta))) ||
         (theta_sampled > (M_PI + theta)) && (theta_sampled < (2*M_PI - theta)) ) {
        radius = 0.5*object.primitives[0].dimensions[object.primitives[0].BOX_Y];
      }
      else if( theta_sampled == theta ||
        theta_sampled == (M_PI - theta) ||
        theta_sampled == (M_PI + theta) ||
        theta_sampled == (2*M_PI - theta)){
        radius = 0.5*std::max(object.primitives[0].dimensions[object.primitives[0].BOX_X], object.primitives[0].dimensions[object.primitives[0].BOX_Y]);
      }
    }
  }
  else if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    radius = object.primitives[0].dimensions[object.primitives[0].CYLINDER_RADIUS];
  }
  else {
    ROS_ERROR("Cannot get the object radius! Use the default value 1000 as the object radius.");
    radius = 100000;
  }
  // ROS_INFO_STREAM_NAMED("DEBUG"," ++++++++radius = "<<radius);
  return radius;
}

// get the grasp type for the object
// grasp_type = 4: wrap grasp with Preshape = 0; x > 0 right(camera DOWN); Horizontal; example: mug #1 
// grasp_type = 0: wrap grasp with Preshape = 0; x > 0 left(camera UP); Horizontal; example: mug #1 
// grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 the third finger will close
// grasp_type = 2: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: small(short) mug #? the third finger will NOT close
// grasp_type = 3: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball

// int SimpleGrasps::get_grasp_type(const world_model_msgs::Object &object){
//   int grasp_type;
//   double height = get_height(object);
//   double radius = get_radius(object);
//   if(height < 0.05){
//     grasp_type = 3;
//   }
//   else{
//     if(height < 0.12){
//       if (radius > 0.05){
//         grasp_type = 1;
//       }
//       else {
//         grasp_type = 2;
//       }
//     }
//     else{
//       if(height > 0.13){
//         grasp_type = 4;
//         }
//       else{
//         grasp_type = 0;
//       }
//     }
//   }
//   ROS_INFO_STREAM_NAMED("DEBUG","grasp_type: " << grasp_type);
//   return grasp_type;
// }

double SimpleGrasps::get_score_cylinderb(double theta){
  if (theta <= - M_PI){
    return get_score_cylinderb(theta + M_PI);
  }
  else if (theta > M_PI){
    return get_score_cylinderb(theta - M_PI);
  }
  else{
//     return exp(-pow(theta,2));
    return cos(theta / 2.0);
  }
}

double SimpleGrasps::get_score_cylinder(double theta, std::string hand, const world_model_msgs::Object &object, 
  geometry_msgs::Pose &current_pose){
  double mean = 0;
  double penalty_factor = 1.0;
  double hand_offset = -0.5;
  double y_delta = object.primitive_poses[0].position.y - current_pose.position.y;
  double x_delta = object.primitive_poses[0].position.x - current_pose.position.x;
  double theta_anchor = 0;
  if (x_delta == 0.0) {
    theta_anchor = M_PI / 2.0;
  } 
  else {
    theta_anchor = atan( y_delta / x_delta);
  }

  if (object.primitives.size() > 1 && (object.primitives[1].type == object.primitives[1].CYLINDER || object.primitives[1].type == object.primitives[1].BOX)){
    mean = 0;
  }
  else {
    mean = theta_anchor;
  } 

  return penalty_factor * get_score_cylinderb(theta - mean);
  
}

// double SimpleGrasps::get_score_cylinder(double theta, std::string hand, const world_model_msgs::Object &object){
//   if (object.primitives[1].type == object.primitives[1].CYLINDER || object.primitives[1].type == object.primitives[1].BOX){
//     double adjacent = object.primitive_poses[1].position.x - object.primitive_poses[0].position.x;
//     double opposite = object.primitive_poses[1].position.y - object.primitive_poses[0].position.y;
//     double hypotenuse = sqrt(pow(adjacent,2)+pow(opposite,2)); 
//     double theta_rotation = asin(abs(opposite)/hypotenuse);
//     // ROS_INFO_STREAM_NAMED("DEBUG", "theta_rotation = " << theta_rotation);
//     if (adjacent >=0 && opposite >= 0){
//       theta_rotation = theta_rotation;
//     }
//     else if (adjacent < 0 && opposite > 0){
//       theta_rotation = M_PI - theta_rotation;
//     }
//     else if (adjacent < 0 && opposite > 0){
//       theta_rotation = 2*M_PI - theta_rotation;
//     }
//     else {
//       theta_rotation = M_PI + theta_rotation;
//     }
//     double best_point = theta_rotation + M_PI;
//     if(best_point >= 2*M_PI){
//       best_point = best_point - 2*M_PI;
//     }
//     double mean = theta - best_point;
//     return get_score_cylinderb(mean);
//   }
//   else {
//     double y_position = object.primitive_poses[0].position.y;
//     if (hand == "left_hand" && y_position >= 0){
//       double mean = theta + M_PI/4.0;
//       return get_score_cylinderb(mean);
//     }
//     else if (hand == "left_hand" && y_position < 0){
//       double mean = theta - 3*M_PI/2.0;
//       return get_score_cylinderb(mean);
//     }
//     else if (hand == "right_hand" && y_position >= 0){
//       double mean = theta - M_PI/4.0;
//       return get_score_cylinderb(mean);
//     } 
//     else if (hand == "right_hand" && y_position < 0){
//       double mean = theta + M_PI/4.0;
//       return get_score_cylinderb(mean);
//     }
//     else {
//       ROS_ERROR("Cannot get the score, please input the Hand Side and the object!");
//       return -1000;
//     }
//   } 
// }


double SimpleGrasps::get_score_boxb(double theta){
  if (theta <= - M_PI){
    return get_score_boxb(theta + M_PI);
  }
  else if (theta > M_PI){
    return get_score_boxb(theta - M_PI);
  }
  else{
    return exp(-pow(theta,2));
  }
}

// Assuming box is always pointing out from robot.
double SimpleGrasps::get_score_box(double theta, std::string hand, const world_model_msgs::Object &object,
  geometry_msgs::Pose &current_pose){
  double X = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_X]);
  double Y = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_Y]);
  double theta_anchor = atan(Y/X);
  double y_position = object.primitive_poses[0].position.y;
  double hand_position = current_pose.position.y;
  
  double mean = 0;
  double penalty_factor = 1.0;

  // ROS_ERROR_STREAM("HAND:" << hand_position << ", y_position:" << y_position);

  if (theta >= 0 && theta < theta_anchor ) {
    if (Y < 0.14) {
      mean = 0;
    }
    else if (y_position - hand_position >= -0.5) {
      mean = theta_anchor;
      penalty_factor *= 0.9;
    }
    else {
      mean = 2*M_PI - theta_anchor;
      penalty_factor *= 0.9;
    }
    if (Y > X) {
      penalty_factor *= 0.9; 
    }
  }
  else if (theta >= theta_anchor && theta < M_PI - theta_anchor) {
    if (X < 0.14) {
      mean = M_PI / 2.0 ;
    }
    else {
      mean = theta_anchor;
      penalty_factor *= 0.9;
    }
    if (X > Y) {
      penalty_factor *= 0.9;
    }
    if (y_position - hand_position < -0.5) {
      penalty_factor *= 0.9;
    }
    ROS_ERROR_STREAM("CHECK Right");
  }
  else if (theta >= M_PI - theta_anchor && theta < M_PI + theta_anchor) {
    if (Y < 0.14) {
      mean = M_PI;
    }
    else if (y_position - hand_position >= -0.5) {
      mean = M_PI + theta_anchor;
      penalty_factor *= 0.9; 
    }
    else {
      mean = M_PI - theta_anchor;
      penalty_factor *= 0.9; 
    }
    if (Y > X) {
      penalty_factor *= 0.9;
    }
    // Penalty for grasping from backside
    penalty_factor *= 0.8;
  }
  else if (theta >= M_PI + theta_anchor && theta < 2*M_PI - theta_anchor) {
    if (X < 0.14) {
      mean = M_PI / 2.0 + M_PI ;
    }
    else {
      mean = 2*M_PI - theta_anchor;
      penalty_factor *= 0.9;
    }
    if (X > Y) {
      penalty_factor *= 0.9;
    }
    if (y_position - hand_position >= -0.5) {
      penalty_factor *= 0.9;
    }
    penalty_factor = 0;
    ROS_ERROR_STREAM("CHECK left");

  }
  else if (theta >= 2* M_PI - theta_anchor && theta <= 2*M_PI) {
    if (Y < 0.14) {
      mean = 0;
    }
    else if (y_position - hand_position >= -0.5) {
      mean = theta_anchor;
      penalty_factor *= 0.9;
    }
    else {
      mean = 2*M_PI - theta_anchor;
      penalty_factor *= 0.9;
    }
    if (Y > X) {
      penalty_factor *= 0.9; 
    }
  }
  else {
    ROS_ERROR_STREAM("Cannot get score for grasping");
    return -1;
  }

  double score = penalty_factor*get_score_boxb(theta - mean);
  ROS_ERROR_STREAM("Theta: " << theta);
  ROS_ERROR_STREAM("Mean: " << mean);
  ROS_ERROR_STREAM("Penalty: " << penalty_factor);
  ROS_ERROR_STREAM("SCORE: " << score);
  return score;
}

double SimpleGrasps::get_score(double theta_sampled, std::string hand, const world_model_msgs::Object &object,
  geometry_msgs::Pose &current_pose){
  double x_angle0, y_angle0, z_angle0, x_angle1, y_angle1, z_angle1;
  get_angle_diff(object, x_angle0, y_angle0, z_angle0, x_angle1, y_angle1, z_angle1);

  // Map the sampled theta to system that consider front face of object is x axis.
  double theta = 0.0;

  // double theta = theta_sampled + x_angle0;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    if (theta_sampled > x_angle1) {
      theta = theta_sampled - x_angle1;
    } 
    else {
      theta = 2*M_PI + theta_sampled - x_angle1; 
    } 
    return get_score_cylinder(theta, hand, object, current_pose);
  }
  else if(object.primitives[0].type == object.primitives[0].BOX){
    if (theta_sampled > x_angle0) {
      theta = theta_sampled - x_angle0;
    } 
    else {
      theta = 2*M_PI + theta_sampled - x_angle0; 
    }     
    return get_score_box(theta, hand, object, current_pose);
  }
  else {
    ROS_ERROR("Cannot get score for this grasp candidate!");
  }
}

// Eigen::Affine3d SimpleGrasps::get_rotation_matrix(const world_model_msgs::Object &object){
//   Eigen::Affine3d pose_origin;
//   pose_origin = Eigen::Matrix<double, 3, 3>::Identity();
//   // tf::poseMsgToEigen(object.primitive_poses[0], pose_origin);
//   // std::cout << "-------pose_origin " << std::endl;
//   // std::cout << pose_origin.matrix() << std::endl;
//   Eigen::Affine3d pose_box;
//   tf::poseMsgToEigen(object.primitive_poses[0], pose_box);
//   // std::cout << "-------pose_handle " << std::endl;
//   std::cout << pose_box.matrix() << std::endl;
//   Eigen::Affine3d rot_mat = pose_origin * pose_box.inverse();
//   // std::cout << "-------rot_mat " << std::endl;
//   // std::cout << rot_mat.matrix() << std::endl;
//   return rot_mat;
// }

void SimpleGrasps::get_angle_diff(const world_model_msgs::Object &object, double &x_angle0, double &y_angle0, double &z_angle0,
                                 double &x_angle1, double &y_angle1, double &z_angle1){
  geometry_msgs::Pose object_pose0 = object.primitive_poses[0];
  tf::Quaternion object_q0(object_pose0.orientation.x,
                            object_pose0.orientation.y,
                            object_pose0.orientation.z,
                            object_pose0.orientation.w);
  tf::Matrix3x3 object_m0(object_q0);
  tf::Vector3 x(1, 0, 0);
  tf::Vector3 y(0, 1, 0);
  tf::Vector3 z(0, 0, 1);
  x_angle0 = 0;
  y_angle0 = 0;
  z_angle0 = 0;
  x_angle0 = x.angle(object_m0.getColumn(0));
  y_angle0 = y.angle(object_m0.getColumn(1));
  z_angle0 = z.angle(object_m0.getColumn(2));
  std::cout << "x_angle0 = " << x_angle0 << std::endl;
  std::cout << "y_angle0 = " << y_angle0 << std::endl;
  std::cout << "z_angle0 = " << z_angle0 << std::endl;
  std::cout << "-------------" << std::endl;

  if ( object.primitives.size() > 1 && (
    object.primitives[1].type == object.primitives[1].BOX || object.primitives[1].type == object.primitives[1].CYLINDER)) {
    geometry_msgs::Pose object_pose1 = object.primitive_poses[1];
    tf::Quaternion object_q1(object_pose1.orientation.x,
                              object_pose1.orientation.y,
                              object_pose1.orientation.z,
                              object_pose1.orientation.w);
    tf::Matrix3x3 object_m1(object_q1);
    x_angle1 = 0;
    y_angle1 = 0;
    z_angle1 = 0;
    x_angle1 = x.angle(object_m1.getColumn(0));
    y_angle1 = y.angle(object_m1.getColumn(1));
    z_angle1 = z.angle(object_m1.getColumn(2));
    std::cout << "x_angle1 = " << x_angle1 << std::endl;
    std::cout << "y_angle1 = " << y_angle1 << std::endl;
    std::cout << "z_angle1 = " << z_angle1 << std::endl;
    std::cout << "-------------" << std::endl;  
  }  
}

void SimpleGrasps::get_xb_yb(const world_model_msgs::Object &object, double theta_sampled, double grasp_radius, double &xb, double &yb){
  double X = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_X]);
  double Y = std::abs(object.primitives[0].dimensions[object.primitives[0].BOX_Y]);
  double theta = atan(Y/X);
  // ROS_INFO_STREAM_NAMED("DEBUG","_______________theta = " << theta);  
  if( ((theta_sampled >= 0) && (theta_sampled <= theta)) ||
     ((theta_sampled >= (2* M_PI - theta)) && theta_sampled < 2*M_PI) ) {
    xb = -grasp_radius;
    yb = -grasp_radius * tan(theta_sampled);
  }
  else if( (theta_sampled > theta) && (theta_sampled < (M_PI - theta)) ){
    xb = -grasp_radius * tan(0.5*M_PI - theta_sampled);
    yb = -grasp_radius;
  }
  else if( (theta_sampled >= (M_PI - theta)) && (theta_sampled <= (M_PI + theta)) ) {
    xb = grasp_radius;
    yb = grasp_radius* tan(theta_sampled);
  }
  else if ( (theta_sampled > (M_PI + theta)) && ( theta_sampled < (2*M_PI - theta) ) ){
    xb = grasp_radius* tan(3*M_PI/2 - theta_sampled);
    yb = grasp_radius;
  }
  // ROS_INFO_STREAM_NAMED("DEBUG","_______________theta_sampled = " << theta_sampled);  
  // ROS_INFO_STREAM_NAMED("DEBUG","_______________xb = " << xb);
  // ROS_INFO_STREAM_NAMED("DEBUG","_______________yb = " << yb);
}
 

bool SimpleGrasps::generateGraspsType0(double theta_sampled, double grasp_radius, const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps, geometry_msgs::Pose &current_pose){
  // Converts object_pose message into object_global_transform_. 
  geometry_msgs::Pose object_pose = object.primitive_poses[0];
  tf::poseMsgToEigen(object_pose, object_global_transform_);
  
  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // Create re-usable blank pose
  geometry_msgs::PoseStamped pre_grasp_pose_msg;
  pre_grasp_pose_msg.header.stamp = ros::Time::now();
  pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  grasp_generator::Grasp new_grasp;
  // A name for this grasp type
  new_grasp.grasp_type = 0;
  
  double xb, yb, zb;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    xb = grasp_radius*cos(theta_sampled + M_PI);
    yb = grasp_radius*sin(theta_sampled + M_PI);
    zb = std::max(-0.5*get_height(object) + 0.095, 0.0);
  }
  else {
    get_xb_yb(object, theta_sampled, grasp_radius, xb, yb);
    zb = std::max(-0.5*get_height(object) + 0.08, 0.0);
  }

  // Create a Grasp message
  Eigen::Affine3d grasp_pose;
  grasp_pose = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ())
              *Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())
              *Eigen::AngleAxisd(theta_sampled, Eigen::Vector3d::UnitY());
  grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);
  // std::cout << "-------grasp_pose 1 " << std::endl;
  // std::cout << grasp_pose.matrix() << std::endl;

  // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  Eigen::Affine3d pre_grasp_pose;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  }
  else {
   pre_grasp_pose = grasp_pose;
   get_xb_yb(object,theta_sampled, grasp_radius + 0.14, xb, yb);
   zb = zb + 0.02;
   pre_grasp_pose.translation() = Eigen::Vector3d(xb, yb ,zb);
  }

  // Eigen::Affine3d rot_mat = get_rotation_matrix(object);
  // grasp_pose = rot_mat * grasp_pose;
  // pre_grasp_pose = rot_mat * pre_grasp_pose;

  // ROS_INFO_STREAM_NAMED("DEBUG","object.primitive_poses[0].position.y: " <<object.primitive_poses[0].position.y);

  new_grasp.grasp_quality = get_score(theta_sampled, grasp_data.ee_group_, object, current_pose); 
  // new_grasp.grasp_quality *= 0.9;

  // A name for this grasp
  new_grasp.grasp_id = "Grasp id " + boost::lexical_cast<std::string>(grasp_id);
  // ROS_INFO_STREAM_NAMED("DEBUG","Grasp ID 01 = " << new_grasp.grasp_id);
  ++grasp_id;
  
  // // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.12)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));

  // Convert pregrasp pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * pre_grasp_pose, pre_grasp_pose_msg.pose);
  new_grasp.pre_grasp_pose = pre_grasp_pose_msg;

  // Grasp ------------------------------------------------------------------------------------------------
  // Convert pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;
  
  // Generate possible grasps msg
  possible_grasps.push_back(new_grasp);
}

bool SimpleGrasps::generateGraspsType1(double theta_sampled, double grasp_radius, const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps, geometry_msgs::Pose &current_pose){
  // Converts object_pose message into object_global_transform_. 
  geometry_msgs::Pose object_pose = object.primitive_poses[0];
  tf::poseMsgToEigen(object_pose, object_global_transform_);
  
  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // Create re-usable blank pose
  geometry_msgs::PoseStamped pre_grasp_pose_msg;
  pre_grasp_pose_msg.header.stamp = ros::Time::now();
  pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  grasp_generator::Grasp new_grasp;
  // A name for this grasp type
  new_grasp.grasp_type = 1;

  double xb, yb, zb;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    xb = grasp_radius*cos(theta_sampled + M_PI);
    yb = grasp_radius*sin(theta_sampled + M_PI);
    zb = -0.5*get_height(object) + 0.10;
  }
  else {
    get_xb_yb(object,theta_sampled, grasp_radius, xb, yb);
    zb = -0.5*get_height(object) + 0.10;
  }
  
  Eigen::Affine3d grasp_pose;
  grasp_pose = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY())
              *Eigen::AngleAxisd(-theta_sampled, Eigen::Vector3d::UnitX());
  grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

 // PreGrasp---------------------------------------------------------------------------------------------
 // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
 Eigen::Affine3d pre_grasp_pose;
 if (object.primitives[0].type == object.primitives[0].CYLINDER) {
   pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
 }
 else {
  pre_grasp_pose = grasp_pose;
  get_xb_yb(object,theta_sampled, grasp_radius + 0.14, xb, yb);
  zb = zb + 0.02;
  pre_grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
 }

  // Eigen::Affine3d rot_mat = get_rotation_matrix(object);

  // grasp_pose = rot_mat * grasp_pose;
  // pre_grasp_pose = rot_mat * pre_grasp_pose;

  new_grasp.grasp_quality = get_score(theta_sampled, grasp_data.ee_group_, object, current_pose); 

  // A name for this grasp
  new_grasp.grasp_id = "Grasp id " + boost::lexical_cast<std::string>(grasp_id);
  // ROS_INFO_STREAM_NAMED("DEBUG","Grasp ID 01 = " << new_grasp.grasp_id);
  ++grasp_id;
  
  // // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  // Convert pregrasp pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * pre_grasp_pose, pre_grasp_pose_msg.pose);
  new_grasp.pre_grasp_pose = pre_grasp_pose_msg;

  // Grasp ------------------------------------------------------------------------------------------------
  // Convert pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;
  
  // Generate possible grasps msg
  possible_grasps.push_back(new_grasp);
}


bool SimpleGrasps::generateGraspsType2(double theta_sampled, double grasp_radius, const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps, geometry_msgs::Pose &current_pose){
  // Converts object_pose message into object_global_transform_. 
  geometry_msgs::Pose object_pose = object.primitive_poses[0];
  tf::poseMsgToEigen(object_pose, object_global_transform_);
  
  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // Create re-usable blank pose
  geometry_msgs::PoseStamped pre_grasp_pose_msg;
  pre_grasp_pose_msg.header.stamp = ros::Time::now();
  pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  grasp_generator::Grasp new_grasp;
  // A name for this grasp type
  new_grasp.grasp_type = 2;
  double xb, yb, zb;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    xb = grasp_radius*cos(theta_sampled + M_PI);
    yb = grasp_radius*sin(theta_sampled + M_PI);
    zb = -0.5*get_height(object) + 0.10;
  }
  else {
    get_xb_yb(object,theta_sampled, grasp_radius, xb, yb);
    zb = -0.5*get_height(object) + 0.10;
  }

  // Create a Grasp message
  Eigen::Affine3d grasp_pose;
  grasp_pose = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY())
              *Eigen::AngleAxisd(-theta_sampled, Eigen::Vector3d::UnitX());
  grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
  
  // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  Eigen::Affine3d pre_grasp_pose;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  }
  else {
   pre_grasp_pose = grasp_pose;
   get_xb_yb(object,theta_sampled, grasp_radius + 0.14, xb, yb);
   zb = zb + 0.02;
   pre_grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
  }

  // Eigen::Affine3d rot_mat = get_rotation_matrix(object);
  // grasp_pose = rot_mat * grasp_pose;
  // pre_grasp_pose = rot_mat * pre_grasp_pose;

  new_grasp.grasp_quality = get_score(theta_sampled, grasp_data.ee_group_, object, current_pose); 

  // A name for this grasp
  new_grasp.grasp_id = "Grasp id " + boost::lexical_cast<std::string>(grasp_id);
  // ROS_INFO_STREAM_NAMED("DEBUG","Grasp ID 01 = " << new_grasp.grasp_id);
  ++grasp_id;
  

  // // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  // Convert pregrasp pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * pre_grasp_pose, pre_grasp_pose_msg.pose);
  new_grasp.pre_grasp_pose = pre_grasp_pose_msg;

  // Grasp ------------------------------------------------------------------------------------------------
  // Convert pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;
  
  // Generate possible grasps msg
  possible_grasps.push_back(new_grasp);
}


bool SimpleGrasps::generateGraspsType3(double theta_sampled, double grasp_radius, const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps, geometry_msgs::Pose &current_pose){
  // Converts object_pose message into object_global_transform_. 
  geometry_msgs::Pose object_pose = object.primitive_poses[0];
  tf::poseMsgToEigen(object_pose, object_global_transform_);
  
  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // Create re-usable blank pose
  geometry_msgs::PoseStamped pre_grasp_pose_msg;
  pre_grasp_pose_msg.header.stamp = ros::Time::now();
  pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  grasp_generator::Grasp new_grasp;
  // A name for this grasp type
  new_grasp.grasp_type = 3;
  double xb, yb, zb;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    xb = 0;
    yb = 0;
    zb = 0.5*get_height(object) + 0.12;
  }
  else{
    xb = 0;
    yb = 0;
    zb = 0.5*get_height(object) + 0.12;
  }

  // Create a Grasp message
  Eigen::Affine3d grasp_pose;
  grasp_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
              *Eigen::AngleAxisd(theta_sampled, Eigen::Vector3d::UnitZ());
  grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
  
  Eigen::Affine3d pre_grasp_pose;
  // pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));

  // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    // pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(-0.20, 0, 0)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  }
  else {
   pre_grasp_pose = grasp_pose;
   // xb = -0.20;
   xb = 0;
   yb = 0;
   zb = zb + 0.14;
   pre_grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
  }

  // Eigen::Affine3d rot_mat = get_rotation_matrix(object);
  // grasp_pose = rot_mat * grasp_pose;
  // pre_grasp_pose = rot_mat * pre_grasp_pose;

  // ROS_INFO_STREAM_NAMED("DEBUG","object.primitive_poses[0].position.y: " <<object.primitive_poses[0].position.y);
  new_grasp.grasp_quality = get_score(theta_sampled, grasp_data.ee_group_, object, current_pose); 

  // A name for this grasp
  new_grasp.grasp_id = "Grasp id " + boost::lexical_cast<std::string>(grasp_id);
  // ROS_INFO_STREAM_NAMED("DEBUG","Grasp ID 01 = " << new_grasp.grasp_id);
  ++grasp_id;
  
  // // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  // Convert pregrasp pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * pre_grasp_pose, pre_grasp_pose_msg.pose);
  new_grasp.pre_grasp_pose = pre_grasp_pose_msg;

  // Grasp ------------------------------------------------------------------------------------------------
  // Convert pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;
  
  // Generate possible grasps msg
  possible_grasps.push_back(new_grasp);
}

bool SimpleGrasps::generateGraspsType4(double theta_sampled, double grasp_radius, const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps, geometry_msgs::Pose &current_pose){
  // Converts object_pose message into object_global_transform_. 
  geometry_msgs::Pose object_pose = object.primitive_poses[0];
  tf::poseMsgToEigen(object_pose, object_global_transform_);
  
  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // Create re-usable blank pose
  geometry_msgs::PoseStamped pre_grasp_pose_msg;
  pre_grasp_pose_msg.header.stamp = ros::Time::now();
  pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  grasp_generator::Grasp new_grasp;
  // A name for this grasp type
  new_grasp.grasp_type = 4;
  double xb, yb, zb;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    xb = grasp_radius*cos(theta_sampled + M_PI);
    yb = grasp_radius*sin(theta_sampled + M_PI);
    zb = std::max(-0.5*get_height(object) + 0.1, 0.0);
  }
  else{
    get_xb_yb(object,theta_sampled, grasp_radius, xb, yb);
    zb = std::max(-0.5*get_height(object) + 0.1, 0.0);
  }

  // Create a Grasp message
  Eigen::Affine3d grasp_pose;
  grasp_pose = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
              *Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX())
              *Eigen::AngleAxisd(-theta_sampled, Eigen::Vector3d::UnitY());
  grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

  // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  Eigen::Affine3d pre_grasp_pose;
  if (object.primitives[0].type == object.primitives[0].CYLINDER) {
    pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));
  }
  else {
   pre_grasp_pose = grasp_pose;
   get_xb_yb(object,theta_sampled, grasp_radius + 0.14, xb, yb);
   zb = zb + 0.02;
   pre_grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);
  }

  // Eigen::Affine3d rot_mat = get_rotation_matrix(object);
  // grasp_pose = rot_mat * grasp_pose;
  // pre_grasp_pose = rot_mat * pre_grasp_pose;

  // ROS_INFO_STREAM_NAMED("DEBUG","object.primitive_poses[0].position.y: " <<object.primitive_poses[0].position.y);
  new_grasp.grasp_quality = get_score(theta_sampled, grasp_data.ee_group_, object, current_pose); 

  // A name for this grasp
  new_grasp.grasp_id = "Grasp id " + boost::lexical_cast<std::string>(grasp_id);
  // ROS_INFO_STREAM_NAMED("DEBUG","Grasp ID 01 = " << new_grasp.grasp_id);
  ++grasp_id;
  
  // // PreGrasp---------------------------------------------------------------------------------------------
  // Eigen::Affine3d pre_grasp_pose = Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, 0.02)) * grasp_pose * Eigen::Translation<double, 3>(Eigen::Vector3d(0, 0, -0.14));

  // Convert pregrasp pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * pre_grasp_pose, pre_grasp_pose_msg.pose);
  new_grasp.pre_grasp_pose = pre_grasp_pose_msg;

  // Grasp ------------------------------------------------------------------------------------------------
  // Convert pose to global frame (base_link)
  tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;
  
  // Generate possible grasps msg
  possible_grasps.push_back(new_grasp);
}


// Create all possible grasp positions for an object
bool SimpleGrasps::generateBlockGrasps(const world_model_msgs::Object &object, const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor){
  generateAxisGrasps( object, grasp_data, possible_grasps, planning_scene_monitor);
  return true;
}

// Create grasp positions in one axis
bool SimpleGrasps::generateAxisGrasps(
  const world_model_msgs::Object &object,
  const GraspData &grasp_data,
  std::vector<grasp_generator::Grasp> &possible_grasps,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor
  ){
  // // Converts object_pose message into object_global_transform_. 
  // geometry_msgs::Pose object_pose = object.primitive_poses[0];
  // tf::poseMsgToEigen(object_pose, object_global_transform_);

  // // Create re-usable blank pose
  // geometry_msgs::PoseStamped grasp_pose_msg;
  // grasp_pose_msg.header.stamp = ros::Time::now();
  // grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // // Create re-usable blank pose
  // geometry_msgs::PoseStamped pre_grasp_pose_msg;
  // pre_grasp_pose_msg.header.stamp = ros::Time::now();
  // pre_grasp_pose_msg.header.frame_id = grasp_data.base_link_;


  // // Create re-usable blank pose
  // geometry_msgs::PoseStamped post_grasp_pose_msg;
  // post_grasp_pose_msg.header.stamp = ros::Time::now();
  // post_grasp_pose_msg.header.frame_id = grasp_data.base_link_;
  
  double height = get_height(object);
  double radius;
  double grasp_radius;
  double theta_sampled = 0.0;

  // int grasp_type = get_grasp_type(object);
  // ROS_INFO_STREAM_NAMED("DEBUG","grasp_type: " << grasp_type);
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop
  // ---------------------------------------------------------------------------------------------
  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the object, then later convert it to the base link
   */
  // static int grasp_id = 0;

  std::string end_effector = grasp_data.ee_group_;
  geometry_msgs::Pose current_pose;

  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);
  robot_state::RobotState robot_state = planning_scene->getCurrentState();

  Eigen::Affine3d pose;
  pose.setIdentity();
  if (end_effector.empty()) {
    ROS_ERROR("No end-effector specified.");
  }
  else {
    const moveit::core::LinkModel *lm = robot_state.getLinkModel(end_effector);
    if (lm) {
      pose = robot_state.getGlobalLinkTransform(lm);
    }
  }
  tf::poseEigenToMsg(pose, current_pose);

  for(int i = 0; i <= grasp_data.angle_resolution_; ++i)
  {
    // get grasp radius
    radius = get_radius(object, theta_sampled);
    // ROS_INFO_STREAM_NAMED("DEBUG"," ++++++++radius = "<<radius);
    // ROS_INFO_STREAM_NAMED("DEBUG"," ++++++++height = "<<height);

    if (object.primitives[0].type == object.primitives[0].BOX) {
      grasp_radius = radius + 0.07;
    }
    else {
      grasp_radius = radius + 0.07;

    }

    //generate grasp candidates

    if(height < 0.05){
      generateGraspsType3(theta_sampled, grasp_radius, object, grasp_data, possible_grasps, current_pose);
    }
    else{
      if (height < 0.12){
        if (radius > 0.05){
          generateGraspsType1(theta_sampled, grasp_radius, object, grasp_data, possible_grasps, current_pose);
        }
        else{
          generateGraspsType2(theta_sampled, grasp_radius, object, grasp_data, possible_grasps, current_pose);
        }
      }
      else{
        // generateGraspsType0(theta_sampled, grasp_radius, object, grasp_data, possible_grasps, current_pose);
        if(height > 0.13){
          generateGraspsType4(theta_sampled, grasp_radius, object, grasp_data, possible_grasps, current_pose);
        }
      }
      //add power grasps for all grasp types
      // generateGraspsType3(theta_sampled, grasp_radius, object, grasp_data, possible_grasps);
    }
    theta_sampled += 2*M_PI / grasp_data.angle_resolution_;
  }
  // visualization
  // for(int i = 0; i < possible_grasps.size(); ++i){
  //   //Debug visualization
  //   ROS_INFO_STREAM_NAMED("simple_grasp", "grasp poses display in rviz!");
  //   visual_tools_->publishArrow(possible_grasps[i].grasp_pose, rviz_visual_tools::RED);
  //   ros::Duration(0.015).sleep();
  //   ROS_INFO_STREAM_NAMED("simple_grasp", "pregrasp poses display in rviz!");
  //   visual_tools_->publishArrow(possible_grasps[i].pre_grasp_pose, rviz_visual_tools::BLUE);
  //   ros::Duration(0.015).sleep();

  //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps TYPE: " << (int) possible_grasps[i].grasp_type);
  //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps QUALITY: " << possible_grasps[i].grasp_quality);
  //   ROS_INFO_STREAM_NAMED("intermediate value", "possible_grasps ID: " << possible_grasps[i].grasp_id);
  // }
  ROS_INFO_STREAM_NAMED("grasp", "Totally Generated " << possible_grasps.size() << " grasps." );
  // double x_angle0, y_angle0, z_angle0, x_angle1, y_angle1, z_angle1;
  // get_angle_diff(object, x_angle0, y_angle0, z_angle0, x_angle1, y_angle1, z_angle1);
  return true;
}

} // namespace
  
