#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf::Vector3 translation(0.3, 0.4, 0.6);

  tf::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(q);
  tf::StampedTransform stampedTransform(transform, ros::Time::now(), "base_link", "base_laser");
  
  while(n.ok()){
    broadcaster.sendTransform(stampedTransform);
    r.sleep();
  }
}