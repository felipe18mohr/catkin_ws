#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "turtle1";
  transformStamped.child_frame_id = "carrot1";
  transformStamped.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.x = q.y(); 
  transformStamped.transform.rotation.x = q.z(); 
  transformStamped.transform.rotation.x = q.w(); 

  ros::Rate rate(10.0);
  while(nh.ok()){
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
    transformStamped.transform.translation.y = 2.0*cos(ros::Time::now().toSec());
    
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

  return 0;
}