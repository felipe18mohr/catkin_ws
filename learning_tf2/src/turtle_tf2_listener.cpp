#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle nh;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    nh.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name =  "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
    nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while(nh.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      ros::Time now = ros::Time::now();
      ros::Time past = now - ros::Duration(5.0);
      transformStamped = tfBuffer.lookupTransform("turtle2", now, 
                                                  "turtle1", past, 
                                                  "world", ros::Duration(1.0));
    } catch(tf2::TransformException &ex){
      ROS_WARN("Could NOT transform turtle2 to turtle1: %s", ex.what());
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;    
}
