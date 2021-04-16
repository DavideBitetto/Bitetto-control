#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> 
#include <vector>
#include <math.h>
#include <stdlib.h>


using namespace std;

int main(int argc, char **argv)
{

ros::init(argc, argv, "joint");

ros::NodeHandle n;

ros::Publisher joint_1_pub = n.advertise<std_msgs::Float64>("/test/joint_1_position_controller/command", 10);
ros::Publisher joint_2_pub = n.advertise<std_msgs::Float64>("/test/joint_2_position_controller/command", 10);

ros::Rate loop_rate(5);
string joint1, joint2;
vector<float>joint_1;
vector<float>joint_2;

    int k=0;
ifstream coeff("/home/david/catkin_ws/src/test_control/scripts/q_joint.txt"); //opening the file.
	if (coeff.is_open()) //if the file is open
	{
		string line;
		cout <<"sono entrato"<<endl;
		while (getline(coeff,line)) 
		{
			stringstream ss(line);
			getline(ss, joint1, ',');
			joint_1.push_back(stof(joint1));
			getline(ss, joint2);
			joint_2.push_back(stof(joint2));
			k++;
		}
        coeff.close(); //closing the file
    }else cout << "non è stato aperto il file" <<endl;
int i = 0;
int j = 0;
//while (ros::ok())
//{
for (i=0, j=0 ; i < k, j < k ; i++, j++){
std_msgs::Float64 position;
position.data = joint_1[i];

std_msgs::Float64 position1;
position1.data = joint_2[j];
ROS_INFO("%f", position.data);
ROS_INFO("%f", position1.data);
joint_1_pub.publish(position);
joint_2_pub.publish(position1);

ros::spinOnce();


loop_rate.sleep();

}
//}
return 0;
}