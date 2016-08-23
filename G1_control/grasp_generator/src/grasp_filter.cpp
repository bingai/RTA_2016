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

#include <grasp_generator/grasp_filter.h>
#include <moveit/transforms/transforms.h>

// Conversions
#include <eigen_conversions/eigen_msg.h> // add to pkg TODO
#include <algorithm>

namespace grasp_generator
{

// Constructor
GraspFilter::GraspFilter( robot_state::RobotState robot_state,
  moveit_visual_tools::MoveItVisualToolsPtr& visual_tools ):
  robot_state_(robot_state),
  visual_tools_(visual_tools),
  verbose_(false)
{
  ROS_DEBUG_STREAM_NAMED("filter","Loaded simple grasp filter");
}

GraspFilter::~GraspFilter()
{
}

inline bool score_comparator(const grasp_generator::Grasp& lhs, const grasp_generator::Grasp& rhs){
  return lhs.grasp_quality > rhs.grasp_quality;
}

bool GraspFilter::chooseBestGrasp( std::vector<grasp_generator::Grasp> &possible_grasps, grasp_generator::Grasp &best_grasp )
{
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","There are no grasps to choose from");
    return false;
  }
  else {
    std::sort(possible_grasps.begin(), possible_grasps.end(), score_comparator);
    best_grasp = possible_grasps[0]; // just choose first one
    ROS_INFO_STREAM_NAMED("test","+++++Best Grasp QUALITY: " << best_grasp.grasp_quality);
    ROS_INFO_STREAM_NAMED("test","+++++Best Grasp TYPE: " << (int) best_grasp.grasp_type);
    ROS_INFO_STREAM_NAMED("test","+++++Best Grasp ID: " << best_grasp.grasp_id);
    return true;
  }
}  

// Return grasps that are kinematically feasible
bool GraspFilter::filterGrasps(std::vector<grasp_generator::Grasp>& possible_grasps,
  std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions, 
  std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions_pre,
  bool filter_pregrasp,
  const std::string &ee_parent_link, const std::string& planning_group,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
{
  // -----------------------------------------------------------------------------------------------
  // Error check
  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","Unable to filter grasps because vector is empty");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // how many cores does this computer have and how many do we need?
  int num_threads = boost::thread::hardware_concurrency();
  if( num_threads > possible_grasps.size() )
    num_threads = possible_grasps.size();

  if(false)
  {
    num_threads = 1;
    ROS_WARN_STREAM_NAMED("grasp_filter","Using only " << num_threads << " threads");
  }

  // -----------------------------------------------------------------------------------------------
  // Get the solver timeout from kinematics.yaml
  double timeout = robot_state_.getRobotModel()->getJointModelGroup( planning_group )->getDefaultIKTimeout();
  // the default timeout 0.005 is too small to use
  timeout = 0.05;
  ROS_DEBUG_STREAM_NAMED("grasp_filter","Grasp filter IK timeout " << timeout);

  // -----------------------------------------------------------------------------------------------
  // Load kinematic solvers if not already loaded
  if( kin_solvers_[planning_group].size() != num_threads )
  {
    kin_solvers_[planning_group].clear();

    const robot_model::JointModelGroup* jmg = robot_state_.getRobotModel()->getJointModelGroup(planning_group);

    // Create an ik solver for every thread
    for (int i = 0; i < num_threads; ++i)
    {
      //ROS_INFO_STREAM_NAMED("filter","Creating ik solver " << i);

      kin_solvers_[planning_group].push_back(jmg->getSolverInstance());

      // Test to make sure we have a valid kinematics solver
      if( !kin_solvers_[planning_group][i] )
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter","No kinematic solver found");
        return false;
      }
    }
  }

  // Transform poses -------------------------------------------------------------------------------
  // bring the pose to the frame of the IK solver
  const std::string &ik_frame = kin_solvers_[planning_group][0]->getBaseFrame();
  Eigen::Affine3d link_transform;
  if (!moveit::core::Transforms::sameFrame(ik_frame, robot_state_.getRobotModel()->getModelFrame()))
  {
    const robot_model::LinkModel *lm = robot_state_.getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!lm)
      return false;
    //pose = getGlobalLinkTransform(lm).inverse() * pose;
    link_transform = robot_state_.getGlobalLinkTransform(lm).inverse();
  }

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // -----------------------------------------------------------------------------------------------
  // Loop through poses and find those that are kinematically feasible
  std::vector<grasp_generator::Grasp> filtered_grasps;

  boost::thread_group bgroup; // create a group of threads
  boost::mutex lock; // used for sharing the same data structures

  ROS_INFO_STREAM_NAMED("filter", "Filtering possible grasps with " << num_threads << " threads");

  // split up the work between threads
  double num_grasps_per_thread = double(possible_grasps.size()) / num_threads;
  //ROS_INFO_STREAM("total grasps " << possible_grasps.size() << " per thead: " << num_grasps_per_thread);

  int grasps_id_start;
  int grasps_id_end = 0;

  for(int i = 0; i < num_threads; ++i)
  {
    grasps_id_start = grasps_id_end;
    grasps_id_end = ceil(num_grasps_per_thread*(i+1));
    if( grasps_id_end >= possible_grasps.size() )
      grasps_id_end = possible_grasps.size();
    //ROS_INFO_STREAM_NAMED("filter","low " << grasps_id_start << " high " << grasps_id_end);

    // IkThreadStruct tc(possible_grasps, filtered_grasps, ik_solutions, ik_solutions_pre, link_transform, grasps_id_start,
    //   grasps_id_end, kin_solvers_[planning_group][i], filter_pregrasp, ee_parent_link, timeout, &lock, i);
    // bgroup.create_thread( boost::bind( &GraspFilter::filterGraspThread, this, tc ) );

    IkThreadStruct tc(possible_grasps, filtered_grasps, ik_solutions, ik_solutions_pre, link_transform, grasps_id_start,
                      grasps_id_end, kin_solvers_[planning_group][i], filter_pregrasp, ee_parent_link, timeout, &lock, i);
    bgroup.create_thread( boost::bind( &GraspFilter::filterGraspThread, this, tc, planning_scene_monitor, planning_group) );
  }

  ROS_DEBUG_STREAM_NAMED("filter","Waiting to join " << num_threads << " ik threads...");
  bgroup.join_all(); // wait for all threads to finish

  ROS_INFO_STREAM_NAMED("filter", "Grasp filter complete, found " << filtered_grasps.size() << " IK solutions out of " <<
    possible_grasps.size() );
  
  // std::string grasp_ids= "";
  //   for (int i = 0; i< filtered_grasps.size(); ++i){
  //   grasp_ids += (filtered_grasps[i].grasp_id).substr(8) + " "; 
  // }
  // ROS_INFO_STREAM_NAMED("filter", "Grasp ids " << grasp_ids);

  possible_grasps = filtered_grasps;

  if (verbose_)
  {
    // End Benchmark time
    double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
    ROS_INFO_STREAM_NAMED("filter","Grasp generator IK grasp filtering benchmark time:");
    std::cout << duration << "\t" << possible_grasps.size() << "\n";

    ROS_INFO_STREAM_NAMED("filter","Possible grasps filtered to " << possible_grasps.size() << " options.");
  }

  return true;
}

//***********************added directly from last year********************************************
void GraspFilter::collisionFilterGrasps(std::vector<grasp_generator::Grasp>& possible_grasps,
  std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions,
  std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions_pre,
  // std::set<std::string> ignore_objs,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
  const std::string& planning_group) {

  if( possible_grasps.empty() )
  {
    ROS_ERROR_NAMED("filter","Unable to filter grasps because IK solution is empty");
    return;
  }

  // Collision filtering
  std::vector<grasp_generator::Grasp> collision_filtered_grasps;
  std::vector<trajectory_msgs::JointTrajectoryPoint> filtered_ik_solutions;

  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);

  // robot_state::RobotState robot_state(planning_scene->getRobotModel());
  robot_state::RobotState robot_state = planning_scene->getCurrentState();

  for (int i = 0; i < possible_grasps.size(); i++) {
    // convert ik_solution to robot state
    std::vector<double> joint_states = ik_solutions[i].positions;
    robot_state.setJointGroupPositions(planning_group, joint_states);

    // check if robot state is in collision and add to possible_grasps if not
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 9e9;
    req.max_contacts_per_pair = 9e9;
    req.cost = true;
    req.distance = true;
    req.group_name = planning_group;
    req.verbose = false;

    collision_detection::CollisionResult res;

    planning_scene->checkCollision(req, res, robot_state, planning_scene->getAllowedCollisionMatrix());
    // collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();

    // acm.print(std::cout);
    bool object_collision = false;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin(); it != res.contacts.end(); ++it) {
      if (it->first.second.find("distal")==0 || it->first.first.find("distal")==0){
        // dont do anything if find collision from finger to table
        ROS_INFO_STREAM("------------------------------- aaa");
        // object_collision = true;
      }
      else if (it->first.second.find("proximal")==0 || it->first.first.find("proximal")==0){
        // dont do anything if find collision from finger to table
        ROS_INFO_STREAM("------------------------------- bbb");
        // object_collision = true;
      }
      else if (it->first.first.find("visual") !=std::string::npos || it->first.second.find("visual") !=std::string::npos ){
        // dont do anything if the collision is visual part
        ROS_INFO_STREAM("------------------------------- ccc");
        // object_collision = true;
      }
      else {
        ROS_WARN("Counting collision %s with %s", it->first.first.c_str(), it->first.second.c_str());
        object_collision = true;
      }

      // bool already_found = ignore_objs.find(it->first.second) != ignore_objs.end();

      // if (it->first.second != possible_grasps[i].object_id && it->first.second.find("rvs") != std::string::npos && !already_found) {
      //   ROS_INFO("Counting collision with %s", it->first.second.c_str());
      //   table_object_collision = true;
      //   break;
      //   // ROS_WARN("Collision with %s!", it->first.second.c_str());
      // } else if (already_found) {
      //   ROS_INFO("NOT counting collision with %s", it->first.second.c_str());
      //   possible_grasps[i].collision_objects.push_back(it->first.second);
      // }
    }

    if (!object_collision){
      // Check pre grasp collision
      // convert ik_solution_pre to robot state
      std::vector<double> joint_states_pre = ik_solutions_pre[i].positions;
      robot_state.setJointGroupPositions(planning_group, joint_states_pre);

      // check if robot state is in collision and add to possible_graps if not
      collision_detection::CollisionRequest req_pre;
      req_pre.contacts = true;
      req_pre.max_contacts = 9e9;
      req_pre.max_contacts_per_pair = 9e9;
      req_pre.cost = true;
      req_pre.distance = true;
      req_pre.group_name = planning_group;
      req_pre.verbose = false;

      collision_detection::CollisionResult res_pre;

      planning_scene->checkCollision(req_pre, res_pre, robot_state, planning_scene->getAllowedCollisionMatrix());

      bool object_collision_pre = false;
      for (collision_detection::CollisionResult::ContactMap::const_iterator it = res_pre.contacts.begin(); it != res_pre.contacts.end(); ++it) {
        if (it->first.second.find("distal")==0 || it->first.first.find("distal")==0){
          // dont do anything if find collision from finger to table
          ROS_INFO_STREAM("--------------------------- PRE aaa");
          // object_collision_pre = true;
        }
        else if (it->first.second.find("proximal")==0 || it->first.first.find("proximal")==0){
        // dont do anything if find collision from finger to table
          ROS_INFO_STREAM("--------------------------- PRE bbb");
          // object_collision_pre = true;
        }
        else if (it->first.first.find("visual") !=std::string::npos || it->first.second.find("visual") !=std::string::npos ){
          // dont do anything if the collision is visual part
          ROS_INFO_STREAM("---------------------------- PRE ccc");
          // object_collision_pre = true;
        }
        else {
          ROS_WARN("Counting collision %s with %s", it->first.first.c_str(), it->first.second.c_str());
          object_collision_pre = true;
        }

        // bool already_found = ignore_objs.find(it->first.second) != ignore_objs.end();

        // if (it->first.second != possible_grasps[i].object_id && it->first.second.find("rvs") != std::string::npos && !already_found) {
        //   ROS_INFO("Counting collision with %s", it->first.second.c_str());
        //   table_object_collision = true;
        //   break;
        //   // ROS_WARN("Collision with %s!", it->first.second.c_str());
        // } else if (already_found) {
        //   ROS_INFO("NOT counting collision with %s", it->first.second.c_str());
        //   possible_grasps[i].collision_objects.push_back(it->first.second);
        // }
      }

      if (!object_collision_pre) { 
        collision_filtered_grasps.push_back(possible_grasps[i]);
        filtered_ik_solutions.push_back(ik_solutions[i]);
      }
    }
  }

  possible_grasps = collision_filtered_grasps;
  ik_solutions = filtered_ik_solutions;

  ROS_INFO_STREAM("Collision filter complete, found " << possible_grasps.size() << " collision free grasps");
}

void GraspFilter::filterGraspThread(IkThreadStruct ik_thread_struct,
 planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
 const std::string& planning_group)
// void GraspFilter::filterGraspThread(IkThreadStruct ik_thread_struct,
//  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
//  const std::string& planning_group)
{
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);

  // robot_state::RobotState robot_state(planning_scene->getRobotModel());
  robot_state::RobotState robot_state = planning_scene->getCurrentState();

  // Seed state - start at zero
  // TODO do not assume 7 dof
  std::vector<double> ik_seed_state; // fill with zeros

  robot_state.copyJointGroupPositions(planning_group, ik_seed_state);  

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::PoseStamped ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    //ROS_DEBUG_STREAM_NAMED("filter", "Checking grasp #" << i);

    // Clear out previous solution just in case - not sure if this is needed
    solution.clear();

    // Transform current pose to frame of planning group
    ik_pose = ik_thread_struct.possible_grasps_[i].grasp_pose;
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(ik_pose.pose, eigen_pose);
    eigen_pose = ik_thread_struct.link_transform_ * eigen_pose;
    tf::poseEigenToMsg(eigen_pose, ik_pose.pose);

    // Test it with IK
    ik_thread_struct.kin_solver_->
      searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, solution, error_code);

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      //ROS_INFO_STREAM_NAMED("filter","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = solution;

      // Start pre-grasp section ----------------------------------------------------------
      if (ik_thread_struct.filter_pregrasp_)       // optionally check the pregrasp
      {
        // Convert to a pre-grasp
        ik_pose = ik_thread_struct.possible_grasps_[i].pre_grasp_pose;
        // ik_pose = SimpleGrasps::getPreGraspPose(ik_thread_struct.possible_grasps_[i], ik_thread_struct.ee_parent_link_);

        // Transform current pose to frame of planning group
        Eigen::Affine3d eigen_pose;
        tf::poseMsgToEigen(ik_pose.pose, eigen_pose);
        eigen_pose = ik_thread_struct.link_transform_ * eigen_pose;
        tf::poseEigenToMsg(eigen_pose, ik_pose.pose);

        // Test it with IK
        ik_thread_struct.kin_solver_->
          searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, solution, error_code);

        // Results
        if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
        {
          ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose.");
          continue;
        }
        else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
        {
          //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose: Timed Out.");
          continue;
        }
        else if( error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS )
        {
          ROS_INFO_STREAM_NAMED("filter","IK solution error for pre-grasp: MoveItErrorCodes.msg = " << error_code);
          continue;
        }
      }
      // Both grasp and pre-grasp have passed
      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps_.push_back( ik_thread_struct.possible_grasps_[i] );

        trajectory_msgs::JointTrajectoryPoint point;
        //point.positions = ik_seed_state; // show the grasp solution
        point.positions = ik_seed_state; // show the pre-grasp solution

        // Copy solution so that we can optionally use it later
        ik_thread_struct.ik_solutions_.push_back(point);

        trajectory_msgs::JointTrajectoryPoint point_pre;
        point_pre.positions = solution;

        ik_thread_struct.ik_solutions_pre_.push_back(point_pre);
      }

      // End pre-grasp section -------------------------------------------------------
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
      ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pose.");
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pose: Timed Out.");
    }
    else
      ROS_INFO_STREAM_NAMED("filter","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }

  //ROS_DEBUG_STREAM_NAMED("filter","Thread " << ik_thread_struct.thread_id_ << " finished");
}

} // namespace
