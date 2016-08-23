#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

#include <iostream>
using namespace std;

visualization_msgs::Marker get_corner(visualization_msgs::Marker table, int wf, int hf) {

  visualization_msgs::Marker table_corner = table;
  table_corner.type = visualization_msgs::Marker::ARROW;
  table_corner.id = 1+((wf<<1)|hf);

  table_corner.color.r = 1.0f;
  table_corner.color.g = 0.0f;
  table_corner.color.b = 0.0f;
  table_corner.color.a = 0.5f;

  table_corner.pose.position.x = table.pose.position.x + table.scale.x / 2.0 * (wf?1:-1);
  table_corner.pose.position.y = table.pose.position.y + table.scale.y / 2.0 * (hf?1:-1);
  table_corner.pose.position.z = table.pose.position.z + table.scale.z / 2.0;
  table_corner.pose.orientation.y = -1.0;

  table_corner.scale.x = 0.10;
  table_corner.scale.y = 0.02;
  table_corner.scale.z = 0.02;

  return table_corner;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "table_marker");
  ros::NodeHandle nh;
  ros::Rate r(1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("table", 1);
  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("table_corner", 1);

  while (ros::ok())
  {
    visualization_msgs::Marker table_marker;
    visualization_msgs::MarkerArray corner_marker;

    table_marker.header.frame_id = "/base";
    table_marker.header.stamp = ros::Time::now();

    table_marker.ns = "table";
    table_marker.id = 0;
    table_marker.type = visualization_msgs::Marker::CUBE;
    table_marker.action = visualization_msgs::Marker::ADD;

    table_marker.pose.position.x = 0.72;
    table_marker.pose.position.y = 0.00;
    table_marker.pose.position.z = -0.69;
    table_marker.pose.orientation.x = 0.0;
    table_marker.pose.orientation.y = 0.0;
    table_marker.pose.orientation.z = 0.0;
    table_marker.pose.orientation.w = 1.0;

    table_marker.scale.x = 0.8;
    table_marker.scale.y = 1.6;
    table_marker.scale.z = 1.0;

    table_marker.color.r = 0.0f;
    table_marker.color.g = 1.0f;
    table_marker.color.b = 0.0f;
    table_marker.color.a = 1.0;

    table_marker.lifetime = ros::Duration();
    marker_pub.publish(table_marker);


    corner_marker.markers.push_back(get_corner(table_marker, 0, 0));
    corner_marker.markers.push_back(get_corner(table_marker, 0, 1));
    corner_marker.markers.push_back(get_corner(table_marker, 1, 1));
    corner_marker.markers.push_back(get_corner(table_marker, 1, 0));

    marker_array_pub.publish(corner_marker);

    r.sleep();
  }
}
