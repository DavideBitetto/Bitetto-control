#include<iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

using namespace std;

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "topic_PD"); //The name of the node
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/test/pd_controller/command", 1);    
    ros::Rate loop_rate(10);
    
    // message declarations
    sensor_msgs::JointState joint_state;

    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);
    
    double t_f = 0.05;
    double k = 0.0;


    double sec = ros::Time::now().toSec();    
    double t = t_f + sec;
    
    while (ros::ok()) {

        while((ros::Time::now().toSec() - t < 0) && (k < 6.3)) {
        ros::spinOnce();

        cout << "sono nel while:   " << ros::Time::now().toSec() <<  endl;
        joint_state.name[0]="joint_1";
        joint_state.position[0] = k;
        joint_state.velocity[0] = 0.0;
        joint_state.effort[0] = 0.0;
        joint_state.name[1] ="joint_2";
        joint_state.position[1] = k;
        joint_state.velocity[1] = 0.0;
        joint_state.effort[0] = 0.0;
        joint_pub.publish(joint_state);
        cout << "posizione:   " <<joint_state.position[0] << endl;
        loop_rate.sleep();
        }
        
        cout << "sono uscito dal while " << ros::Time::now().toSec() << endl;
        
        k+= 0.05;
        
        sec = ros::Time::now().toSec(); 
        
        t = t_f + sec;
        
    
    }

    return 0;
}
