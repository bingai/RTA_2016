#include <g1_control/planning_scene_interface.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <ros/ros.h>

namespace g1
{
namespace control
{

class PlanningSceneInterface::PlanningSceneInterfaceImpl
{
public:

  PlanningSceneInterfaceImpl()
  {
    planning_scene_service_ = node_handle_.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  }

  std::vector<std::string> getKnownObjectNames(bool with_type)
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    std::vector<std::string> result;
    request.components.components = request.components.WORLD_OBJECT_NAMES;
    if (!planning_scene_service_.call(request, response))
      return result;
    if (with_type)
    {
      for (std::size_t i = 0 ; i < response.scene.world.collision_objects.size() ; ++i)
        if (!response.scene.world.collision_objects[i].type.key.empty())
          result.push_back(response.scene.world.collision_objects[i].id);
    }
    else
    {
      for (std::size_t i = 0 ; i < response.scene.world.collision_objects.size() ; ++i)
        result.push_back(response.scene.world.collision_objects[i].id);
    }
    return result;
  }

  std::map<std::string, geometry_msgs::Pose> getObjectPoses(const std::vector<std::string> &object_ids)
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    std::map<std::string, geometry_msgs::Pose> result;
    request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
    if (!planning_scene_service_.call(request, response))
    {
      ROS_WARN("Could not call planning scene service to get object names");
      return result;
    }

    for(std::size_t i=0; i < object_ids.size(); ++i)
    {
      for (std::size_t j=0; j < response.scene.world.collision_objects.size() ; ++j)
      {
        if(response.scene.world.collision_objects[j].id == object_ids[i])
        {
          if (response.scene.world.collision_objects[j].mesh_poses.empty() &&
              response.scene.world.collision_objects[j].primitive_poses.empty())
            continue;
          if(!response.scene.world.collision_objects[j].mesh_poses.empty())
            result[response.scene.world.collision_objects[j].id] = response.scene.world.collision_objects[j].mesh_poses[0];
          else
            result[response.scene.world.collision_objects[j].id] = response.scene.world.collision_objects[j].primitive_poses[0];
        }
      }
    }
    return result;
  }   

  void addCollisionObjects(const std::vector<moveit_msgs::CollisionObject> &collision_objects) const
  {
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects = collision_objects;
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_.publish(planning_scene);
  }

  void removeCollisionObjects(const std::vector<std::string> &object_ids) const
  {
    moveit_msgs::PlanningScene planning_scene;
    moveit_msgs::CollisionObject object;
    for(std::size_t i=0; i < object_ids.size(); ++i)
    {
      object.id = object_ids[i];
      object.operation = object.REMOVE;
      planning_scene.world.collision_objects.push_back(object);
    }
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_.publish(planning_scene);
  }

  void addAllowedCollision(const std::vector<std::string> &object_ids)
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
    if (!planning_scene_service_.call(request, response)) {
      ROS_ERROR("Failed to call service /get_planning_scene");
      return;
    }

    moveit_msgs::AllowedCollisionMatrix currentACM;
    currentACM = response.scene.allowed_collision_matrix;

    // ROS_INFO_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
    // ROS_INFO_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());

    for (int i = 0; i < object_ids.size(); i++) {
      currentACM.default_entry_names.push_back(object_ids[i]);
      currentACM.default_entry_values.push_back(true);
    }

   
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher_.publish(planning_scene);
}

  void removeAllowedCollision(const std::vector<std::string> &object_ids)
  {
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
    if (!planning_scene_service_.call(request, response)) {
      ROS_ERROR("Failed to call service /get_planning_scene");
      return;
    }

    moveit_msgs::AllowedCollisionMatrix currentACM = response.scene.allowed_collision_matrix;

    // ROS_INFO_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
    // ROS_INFO_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());

    for (int i = 0; i < object_ids.size(); i++) {
      int j = 0;
      while (j < currentACM.default_entry_names.size()) {
        if (currentACM.default_entry_names[j] == object_ids[i]) {
          currentACM.default_entry_names.erase(currentACM.default_entry_names.begin() + j);
          currentACM.default_entry_values.erase(currentACM.default_entry_values.begin() + j);
        } 
        else {
          j++;
        }
      }
    }


    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher_.publish(planning_scene);
  }

private:

  ros::NodeHandle node_handle_;
  ros::ServiceClient planning_scene_service_;
  ros::Publisher planning_scene_diff_publisher_;
  robot_model::RobotModelConstPtr robot_model_;
};

PlanningSceneInterface::PlanningSceneInterface()
{
  impl_ = new PlanningSceneInterfaceImpl();
}

PlanningSceneInterface::~PlanningSceneInterface()
{
  delete impl_;
}

std::vector<std::string> PlanningSceneInterface::getKnownObjectNames(bool with_type)
{
  return impl_->getKnownObjectNames(with_type);
}

std::map<std::string, geometry_msgs::Pose> PlanningSceneInterface::getObjectPoses(const std::vector<std::string> &object_ids)
{
  return impl_->getObjectPoses(object_ids);
}

void PlanningSceneInterface::addCollisionObjects(const std::vector<moveit_msgs::CollisionObject> &collision_objects) const
{
  return impl_->addCollisionObjects(collision_objects);
}

void PlanningSceneInterface::removeCollisionObjects(const std::vector<std::string> &object_ids) const
{
  return impl_->removeCollisionObjects(object_ids);
}

void PlanningSceneInterface::addAllowedCollision(const std::vector<std::string> &object_ids) 
{
  return impl_->addAllowedCollision(object_ids);
}

void PlanningSceneInterface::removeAllowedCollision(const std::vector<std::string> &object_ids) 
{
  return impl_->removeAllowedCollision(object_ids);
}

}
}
