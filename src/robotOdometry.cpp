#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <dynamic_reconfigure/server.h>
#include <homework1/parametersConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "homework1/givePose.h"

enum integrationTypes {EULER, RK};

class robotOdometry{
    private:

        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Time tPrevious;
        ros::ServiceServer position_service;

        double xPrevious;
		double yPrevious;
		double thetaPrevious;
		

        dynamic_reconfigure::Server<homework1::parametersConfig> server;
        integrationTypes chosenMethod; 
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;

        int count = 143997;

    public :
        robotOdometry(){
            this->sub = n.subscribe("/cmd_vel", 1, &robotOdometry::calculateRobotOdometry, this);
            this->pub = n.advertise<nav_msgs::Odometry>("/odom",1);
            this->position_service = n.advertiseService("givePose", &robotOdometry::assignPosition, this);

            dynamic_reconfigure::Server<homework1::parametersConfig>::CallbackType callbackObject;
            callbackObject = boost::bind(&robotOdometry::integrationMethodHandler, this, _1, _2);
            this->server.setCallback(callbackObject);

            chosenMethod = EULER;

            this->n.getParam("/x_initial_pose", this->xPrevious);
            this->n.getParam("/y_initial_pose", this->yPrevious);
            this->n.getParam("/theta_initial_pose", this->thetaPrevious);

        }

        void calculateRobotOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
            double xNext, yNext, thetaNext, deltaT;

            deltaT = (msg->header.stamp.operator-(this->tPrevious)).toSec();

            if(this->chosenMethod == EULER){
                xNext = this->xPrevious + (msg->twist.linear.x*cos(thetaPrevious)-msg->twist.linear.y*sin(thetaPrevious))*deltaT;
                yNext = this->yPrevious + (msg->twist.linear.x*sin(thetaPrevious)+msg->twist.linear.y*cos(thetaPrevious))*deltaT;
            }else{
                xNext = this->xPrevious + (msg->twist.linear.x*cos(thetaPrevious + msg->twist.angular.z*deltaT*0.5)-msg->twist.linear.y*sin(thetaPrevious + msg->twist.angular.z*deltaT*0.5))*deltaT;
                yNext = this->yPrevious + (msg->twist.linear.x*sin(thetaPrevious + msg->twist.angular.z*deltaT*0.5)+msg->twist.linear.y*cos(thetaPrevious + msg->twist.angular.z*deltaT*0.5))*deltaT;
            }

            
            thetaNext = this->thetaPrevious + msg->twist.angular.z*deltaT;

            nav_msgs::Odometry odom_msg;

            odom_msg.header.seq = msg->header.seq;
            odom_msg.header.stamp = msg->header.stamp;

            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = xNext;
            odom_msg.pose.pose.position.y = yNext;
            odom_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion thetaQuaternion;
            thetaQuaternion.setRPY(0, 0, thetaNext);
            odom_msg.pose.pose.orientation.x = thetaQuaternion.x();
            odom_msg.pose.pose.orientation.y = thetaQuaternion.y();
            odom_msg.pose.pose.orientation.z = thetaQuaternion.z();
            odom_msg.pose.pose.orientation.w = thetaQuaternion.w();
            
            odom_msg.twist.twist = msg->twist;

            for(int i=0; i<16; i++){
                odom_msg.twist.covariance[i] = 0.0;
            }

            for(int i=0; i<16; i++){
                odom_msg.pose.covariance[i] = 0.0;
            }

            this->pub.publish(odom_msg);
            
            transformStamped.header.stamp = msg->header.stamp;
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_link";

            transformStamped.transform.translation.x = xNext;
            transformStamped.transform.translation.y = yNext;
            transformStamped.transform.translation.z = 0.0;
                
            transformStamped.transform.rotation.x = thetaQuaternion.x();
            transformStamped.transform.rotation.y = thetaQuaternion.y();
            transformStamped.transform.rotation.z = thetaQuaternion.z();
            transformStamped.transform.rotation.w = thetaQuaternion.w();
            broadcaster.sendTransform(transformStamped);

            this->xPrevious = xNext;
            this->yPrevious = yNext;
            this->thetaPrevious = thetaNext;
            this->tPrevious = msg->header.stamp;
        }

        void integrationMethodHandler(homework1::parametersConfig &config, uint32_t level){
            if(config.integrationMethod == 0){
                this->chosenMethod = EULER;
            }else{
                this->chosenMethod = RK;
            }
        }

        bool assignPosition(homework1::givePose::Request  &req, homework1::givePose::Response &res){
             this->xPrevious = req.givenX;
             this->yPrevious = req.givenY;
             this->thetaPrevious = req.givenTheta;

            return true;

        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    robotOdometry robotOdometry;
    ros::spin();
    return 0;
}