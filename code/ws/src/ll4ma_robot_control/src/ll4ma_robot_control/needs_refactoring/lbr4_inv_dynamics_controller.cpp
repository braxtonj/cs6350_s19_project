// This file computes gravity compensation from kdl lib and send to gazebo
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#define MAX_DEG_PER_SEC 50
#define GOAL_THRESH 10
using namespace std;
bool got_new_cmd=false;
bool got_robot_state=false;
Eigen::VectorXd q;// current joint position
Eigen::VectorXd q_dot,q_acc;// current joint velocity
vector<double> q_des,q_dot_des;// desired joint position
vector<string> j_names;

// Subscribe to joint states
void stateCallback(sensor_msgs::JointState joint_msg)
{
  if(got_robot_state==false)
    {
      j_names=joint_msg.name;
    }
  q.resize(7);
  q_dot.resize(7);
  for(int i=0;i<7;++i)
    {
      q[i]=joint_msg.position[i];
      q_dot[i]=joint_msg.velocity[i];
    }
  got_robot_state=true;
}

// Subscribe to joint command
void cmdCallback(sensor_msgs::JointState joint_msg)
{
  q_des.resize(7);
  q_dot_des.resize(7);
  q_acc.resize(7);
  for(int i=0;i<7;++i)
  {
    q_des[i]=joint_msg.position[i];
    q_dot_des[i]=joint_msg.velocity[i];
    q_acc[i]=joint_msg.effort[i];
  }
  got_new_cmd=true;
}




int main(int argc, char** argv)
{
  ros::init(argc,argv,"lbr4_computed_torque_controller");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000);
  double max_delta_q=MAX_DEG_PER_SEC*3.14/(180*100);
  // urdf file location:
  string robot_desc="robot_description";

  vector<string> ee_names={"lbr4_7_link"};
  vector<string> base_names={"base_link"};
  vector<double> g_vec={0.0,0.0,-9.8};
  while(!n.hasParam(robot_desc))
  {
    loop_rate.sleep();
  }
  
  manipulator_kdl::robotKDL lbr4_(robot_desc,n,base_names,ee_names,g_vec);


  
  
  // initialize subscribers:
  ros::Subscriber sub=n.subscribe("/lbr4/joint_states",1,stateCallback);
  ros::Subscriber cmd_sub=n.subscribe("/lbr4/controller/joint_cmd",1,cmdCallback);


  // publish torques to robot
  ros::Publisher jnt_pub=n.advertise<sensor_msgs::JointState>("/lbr4/joint_cmd",1);
  
  Eigen::VectorXd tau_g,tau_c;
  vector<double> tau_cmd;
 

  


  tau_g.resize(7);
  tau_c.resize(7);

  tau_cmd.resize(7);
  sensor_msgs::JointState j_cmd;
  int DOF=7;
  std::cerr<<max_delta_q<<endl;
  vector<float> k_p={300.0,300.0,300.0,300.0,300.0,300.0,300.0};
  vector<float> k_d={0.2,0.2,0.2,0.2,0.2,0.2,0.2};  
  q_des.resize(7);

  double goal_thresh=GOAL_THRESH*3.14/(180);

  double q_diff=0.0;
  double max_tau=20.0;

  Eigen::MatrixXd H;
  Eigen::VectorXd tau_pd;
  tau_pd.setZero(7);

  //Eigen::VectorXd q_acc;
  Eigen::VectorXd tau_ff;
  tau_ff.setZero(7);
  q_acc.setConstant(7,0.0);
  double tau_control=0.0;

  
  while(n.ok())
    {
      // Publish to gazebo
      if(got_robot_state)
	{
	  //compute grav compensation:
	  lbr4_.getGtau(0,q,tau_g);
	  lbr4_.getCtau(0,q,q_dot,tau_c);
          lbr4_.getM(0,q,H);

	  if(got_new_cmd)
	  {
	      // pd on desired pose:
            for(int i=0;i<DOF;++i)
            {
              //q_diff=;
              q_acc[i]=0.0;
              q_dot_des[i]=0.0;
              tau_control=k_p[i]*(q_des[i]-q[i])+k_d[i]*(q_dot_des[i]-q_dot[i]);
              
              tau_pd[i]=tau_control;
            }
	  
            tau_ff=H*(q_acc)+tau_pd;
          }
          
          for(int i=0;i<DOF;++i)
          {
            tau_cmd[i]=tau_g[i]+tau_ff[i];
          }
          j_cmd.name=j_names;
	  j_cmd.effort=tau_cmd;
	  jnt_pub.publish(j_cmd);
	 
	}
 
      ros::spinOnce();
      loop_rate.sleep();
    }

  
  
  return 1;
}

