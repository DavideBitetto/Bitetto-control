#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

using namespace std;

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "topic_CT"); //The name of the node
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/test/ct_controller/command", 1);    
    ros::Rate loop_rate(10);
    
    // message declarations
    sensor_msgs::JointState joint_state;
    
    while (ros::ok()) {
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.velocity.resize(2);
        joint_state.effort.resize(2);
        
        joint_state.name[0]="joint_1";
        joint_state.position[0] = 0.0;
        joint_state.velocity[0] = 0.0;
        joint_state.effort[0] = 0.0;
        joint_state.name[1] ="joint_2";
        joint_state.position[1] = 1.5;
        joint_state.velocity[1] = 0.0;
        joint_state.effort[0] = 0.0;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}