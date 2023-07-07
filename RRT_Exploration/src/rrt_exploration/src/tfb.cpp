#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

int main(int argc, char** argv){

ros::init(argc,argv, "tfb_node");

ros::NodeHandle n;
  ros::Rate r(100);

while (ros::ok()){

static tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(0, -0.8, 0.0));
tf::Quaternion q;
q.setRPY(0,0,0);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_1/map", "/robot_2/map"));
   r.sleep();

}

}

