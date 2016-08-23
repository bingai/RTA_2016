#ifndef G1_CONTROL_PLANNING_SCENE_INTERFACE_
#define G1_CONTROL_PLANNING_SCENE_INTERFACE_

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace g1
{
namespace control
{

class PlanningSceneInterface
{
public:
  PlanningSceneInterface();
  ~PlanningSceneInterface();

  std::vector<std::string> getKnownObjectNames(bool with_type = false);

  std::map<std::string, geometry_msgs::Pose> getObjectPoses(const std::vector<std::string> &object_ids);

  void addCollisionObjects(const std::vector<moveit_msgs::CollisionObject> &collision_objects) const;

  void removeCollisionObjects(const std::vector<std::string> &object_ids) const;

  void addAllowedCollision(const std::vector<std::string> &object_ids);

  void removeAllowedCollision(const std::vector<std::string> &object_ids);

private:

  class PlanningSceneInterfaceImpl;
  PlanningSceneInterfaceImpl *impl_;

};

}
}

#endif
