/*
 * Translates nav_msgs/Odometry into sensor_msgs/NavSat{Fix,Status}
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;

static ros::Publisher fix_pub;
std::string frame_id, child_frame_id;
double rot_cov;

void callback(const nav_msgs::OdometryConstPtr& odom) {

  if (odom->header.stamp == ros::Time(0)) {
    return;
  }

  if (!fix_pub) {
    return;
  }

  double northing, easting, latitude, longitude;
  std::string zone;

  northing = odom->pose.pose.position.y;
  easting = odom->pose.pose.position.x;

  std::size_t pos = odom->header.frame_id.find("/utm_");
  if(pos==std::string::npos) {
    ROS_WARN("UTM zone not found in frame_id");
    return;
  }
  zone = odom->header.frame_id.substr(pos + 5, 3);

  ROS_INFO("%s", zone.c_str());

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = odom->header.frame_id.substr(0, pos);
  fix.header.stamp = odom->header.stamp;

  UTMtoLL(northing, easting, zone, latitude, longitude);

  fix.latitude = latitude;
  fix.longitude = longitude; 

  fix_pub.publish(fix);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "reverse_utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");

  fix_pub = node.advertise<sensor_msgs::NavSatFix>("fix", 10);

  ros::Subscriber odom_sub = node.subscribe("odom", 10, callback);

  ros::spin();
}

