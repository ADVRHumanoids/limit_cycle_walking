#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv){
    
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;

  tf::TransformListener listener;
  
  listener.waitForTransform("ci/com", "ci/l_ankle", ros::Time::now(), ros::Duration(3.0));
  
  ros::Rate rate(10.0);
  
  while (node.ok()){
      
  tf::StampedTransform transform;
  

  listener.lookupTransform("ci/com", "ci/l_ankle", ros::Time(0), transform);

  ROS_INFO("x: %f",  transform.getOrigin().z()); /*get distance from com to l_ankle*/
    
   rate.sleep();
  }
  return 0;
};
