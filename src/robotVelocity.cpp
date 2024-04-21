#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "homework1/wheelSpeed.h"

class robotVelocity{
    private:

        ros::Time t_previous;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub_wheels;

        double radius = 0.07; //meters
        double distanceFromCenterX = 0.2; //meters
        double distanceFromCenterY = 0.169; //meters
        double encoderResolution = 42; //counts per revolution
        double gearRatio = 0.2; //meters
        double shape = 0.369;

        double vx;
        double vy;
        double wz;
        
        float wheelTickPrevious[4];
        float wheelTick[4];
        float wheelRpmFromTick[4];
        
    public:

         robotVelocity(){
            this->sub = this->n.subscribe("/wheel_states", 1, &robotVelocity::calculateRobotVelocity, this);
            this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
            
            this->n.getParam("rCalibrated", this->radius);
            this->n.getParam("shapeCalibrated", this->shape);
            this->n.getParam("nCalibrated", this->encoderResolution);

            //publisher of wheel custom message on wheels_rpm topic
            this->pub_wheels = this->n.advertise<homework1::wheelSpeed>("/wheels_rpm", 1);

            for(int i=0; i<4; i++){
                wheelTickPrevious[i] = 0;
                wheelTick[i] = 0;
                wheelRpmFromTick[i] = 0;
            }
            vx = 0;
            vy = 0;
            wz = 0;
        }

        void calculateRobotVelocity(const sensor_msgs::JointState::ConstPtr& msg){
            double delta_t;
           
            delta_t = (msg->header.stamp.operator-(this->t_previous)).toSec();
            
            for(int i=0; i<4; i++){
                wheelTick[i] = msg->position[i];
                if(wheelTickPrevious[0] != 0){
                    wheelRpmFromTick[i] = (wheelTick[i]-wheelTickPrevious[i])*2*3.14*gearRatio/(encoderResolution*delta_t);
                }
            }

            vx = radius*(wheelRpmFromTick[0]+wheelRpmFromTick[1])/2;
            vy = radius*(wheelRpmFromTick[1]-wheelRpmFromTick[3])/2;
            wz = radius*(wheelRpmFromTick[3]-wheelRpmFromTick[0])/(2*(shape));

            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.frame_id = "base_link";
            vel_msg.header.seq = msg->header.seq;
            vel_msg.header.stamp = msg->header.stamp;

            vel_msg.twist.linear.x = vx;
            vel_msg.twist.linear.y = vy;
            vel_msg.twist.linear.z = 0;

            vel_msg.twist.angular.z = wz;
            vel_msg.twist.angular.y = 0;
            vel_msg.twist.angular.x = 0;

            pub.publish(vel_msg);

            //inverts formulae and recalculates wheel speeds
            homework1::wheelSpeed wheel_msg;
            wheel_msg.header = vel_msg.header;
            wheel_msg.rpm_fl=(vx-vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_fr=(vx+vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rr=(vx-vy+(distanceFromCenterX+distanceFromCenterY)*wz)/radius;
            wheel_msg.rpm_rl=(vx+vy-(distanceFromCenterX+distanceFromCenterY)*wz)/radius;

            //publishes wheel speeds
            this->pub_wheels.publish(wheel_msg);

            this->t_previous = msg->header.stamp;
            for(int j=0; j<4; j++){
                wheelTickPrevious[j] = wheelTick[j];           
            }


        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity");
    robotVelocity robotVelocity;
    ros::spin();
    return 0;
}

