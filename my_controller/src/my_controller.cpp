#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <urdf/model.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h> 
 
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Constraints.h>

#include <sensor_msgs/JointState.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;
namespace my_controller_ns
{

class PdController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:

bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{
  
  string path;

  if (!n.getParam("/path", path)) 
  {
    ROS_ERROR("Specify the path of model.urdf");
    return false;
  }

  const char *path_char = path.c_str();

std::cout << "Inizio il caricamento del file URDF"<<std::endl;

if (!Addons::URDFReadFromFile (path_char, model, false, false))
  {
    std::cout << "Error loading model ./onearm_ego.xacro" << std::endl;
    //abort();
  }
 std::cout << "Il file URDF è stato caricato con successo"<<std::endl;
 std::cout << "Degree of freedom overview:" << std::endl;
 std::cout << Utils::GetModelDOFOverview(*model);

    //Controlla ID del robot
    string rr_id; 
    if (!n.getParam("rr_id", rr_id)) {
        ROS_ERROR("PdController: Could not get parameter arm_id!");
        return false;
    }

    //Controlla se ci sono i guadagni
    if (!n.getParam("kp", kp) || !n.getParam("kv", kv)) 
    {
      ROS_ERROR("Non e' stato possibile trovare kp e kv");
      return false;
    }
    vector<string> joint_names;
    if (!n.getParam("joint_names", joint_names) || joint_names.size() != 2)
    {
      ROS_ERROR("Non e' stato possibile trovare tutti i giunti");
      return false;
    }
      
     for (size_t i = 0; i < 2; ++i)
     {
        joint_handle.push_back(hw->getHandle(joint_names[i]));
        command_q_d[i] = joint_handle[i].getPosition();
        command_dot_q_d[i] = joint_handle[i].getVelocity();
     }

    sub_command_ = n.subscribe<sensor_msgs::JointState>("command", 1, &PdController::setCommandCB, this);
    this->pub_err_ = n.advertise<sensor_msgs::JointState> ("tracking_error", 1);

    return true;
}    

void update(const ros::Time& time, const ros::Duration& period)
  {
    for(size_t i = 0; i < 2; ++i)
    {
      q_curr[i] = joint_handle[i].getPosition();
      dot_q_curr[i] = joint_handle[i].getVelocity();
    }


   NonlinearEffects(*model, q_curr, dot_Q, g_);

   cout << g_ <<endl;


    //Legge di controllo PD
    err = command_q_d - q_curr;
    dot_err = command_dot_q_d - dot_q_curr;

    sensor_msgs::JointState error_msg;
    vector<double> err_vec(err.data(), err.data() + err.rows()*err.cols());
    vector<double> dot_err_vec(dot_err.data(), dot_err.data() + dot_err.rows()*dot_err.cols());
    error_msg.header.stamp = ros::Time::now();
    error_msg.position = err_vec;
    error_msg.velocity = dot_err_vec;
    this->pub_err_.publish(error_msg);

    tau_cmd = kp*err + kv*dot_err + g_;

    for(size_t i= 0; i<2; ++i)
    {
      joint_handle[i].setCommand(tau_cmd[i]);
    }
  }

  void setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
  {
    command_q_d = Eigen::Map<const Eigen::Matrix<double, 2, 1>>((msg->position).data());
    command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 2, 1>>((msg->velocity).data());

  }
void starting(const ros::Time& time) { }
void stopping(const ros::Time& time) { }

private:

Model* model = new Model();

VectorNd q_curr = VectorNd::Zero (2);
VectorNd dot_Q = VectorNd::Zero (2);

Eigen::MatrixXd M_ =  Eigen::MatrixXd::Zero(2,2);									

VectorNd g_ = VectorNd::Zero (2);

double kp, kv;
//Eigen::Matrix<double, 2, 1> q_curr;
Eigen::Matrix<double, 2, 1> dot_q_curr;
Eigen::Matrix<double, 2, 1> tau_cmd;

Eigen::Matrix<double, 2, 1> err;
Eigen::Matrix<double, 2, 1> dot_err;

Eigen::Matrix<double, 2, 1> command_q_d;
Eigen::Matrix<double, 2, 1> command_dot_q_d;

ros::Subscriber sub_command_;
ros::Publisher pub_err_;

vector<hardware_interface::JointHandle> joint_handle;

};

PLUGINLIB_EXPORT_CLASS(my_controller_ns::PdController, controller_interface::ControllerBase);

}

