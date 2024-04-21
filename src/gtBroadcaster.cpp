#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <dynamic_reconfigure/server.h>
#include <homework1/parametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class gtBroadcaster{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
    public:
        gtBroadcaster(){
            this->sub = n.subscribe("/robot/pose", 1, &gtBroadcaster::publishingGtTf, this);
        }

        void publishingGtTf(const geometry_msgs::PoseStamped::ConstPtr& msg){
            transformStamped.header.stamp = msg->header.stamp;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "GT";

            transformStamped.transform.translation.x = msg->pose.position.x;
            transformStamped.transform.translation.y = msg->pose.position.y;
            transformStamped.transform.translation.z = msg->pose.position.z;
                
            transformStamped.transform.rotation.x = msg->pose.orientation.x;
            transformStamped.transform.rotation.y = msg->pose.orientation.y;
            transformStamped.transform.rotation.z = msg->pose.orientation.z;
            transformStamped.transform.rotation.w = msg->pose.orientation.w;
            broadcaster.sendTransform(transformStamped);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gtBroadcaster");
    gtBroadcaster gtBroadcaster;
    ros::spin();
    return 0;
}
