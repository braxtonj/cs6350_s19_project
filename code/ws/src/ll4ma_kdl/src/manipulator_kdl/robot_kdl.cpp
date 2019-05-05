/*
 * Copyright (C) 2018 LL4MA lab, University of Utah
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions 
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
 * conditions and the following disclaimer in the documentation and/or other materials provided with 
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to 
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// This function generates a kdl chain and has functions from KDL

#include "robot_kdl.h"



namespace manipulator_kdl
{


robotKDL::robotKDL()
{
  cerr << "Dummy robotKDL constructor initialized!!" << endl;
}


void robotKDL::manipulator_init(string &base_name, string &EE_name)
{
  // Generate chain
  kdl_chains_.resize(1);
  if(!robot_tree_.getChain(base_name, EE_name, kdl_chains_[0]))
    ROS_ERROR("Failed to construct kdl chain from %s to %s", base_name.c_str(), EE_name.c_str());

  // Forward kinematics position solver
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver;
  fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chains_[0]));
  fk_pos_solvers_.push_back(fk_pos_solver);

  // Forward kinematics velocity solver
  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver;
  fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chains_[0]));
  fk_vel_solvers_.push_back(fk_vel_solver);

  // Dynamics solver
  boost::shared_ptr<KDL::ChainDynParam> dyn_solver;
  dyn_solver.reset(new KDL::ChainDynParam(kdl_chains_[0], g_vec_));
  dyn_solvers_.push_back(dyn_solver);

  // Jacobian solver
  boost::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
  jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chains_[0]));
  jacobian_solvers_.push_back(jacobian_solver);

  // initialize data for use in real-time loop
  jacobians_.push_back(new KDL::Jacobian(kdl_chains_[0].getNrOfJoints()));
  H_mat_.push_back(new KDL::JntSpaceInertiaMatrix(kdl_chains_[0].getNrOfJoints()));

  #ifdef DEBUG
    cerr << "Created kinematic and Dynamic chains" << endl;
  #endif
}


void robotKDL::robot_init(vector<string> &base_names, vector<string> &EE_names)
{
  // Assuming kdl tree is already stored in robot_tree_.
  // TODO: check if robot_tree_ is empty.

  H_mat_.resize(EE_names.size());
  jacobians_.resize(EE_names.size());

  for(int i = 0; i < EE_names.size(); ++i)
  {
    // Initialize KDL chain
    string ee_name = EE_names[i];
    KDL::Chain chain;
    if(!robot_tree_.getChain(base_names[i], ee_name, chain))
    {
      ROS_ERROR("Failed to construct kdl chain from %s to %s", base_names[i].c_str(), ee_name.c_str());
    }
    kdl_chains_.push_back(chain);

    // Read off joint names from chain
    vector<string> names;
    for (int i = 0; i < chain.getNrOfSegments(); ++i)
      if (chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
        names.push_back(chain.getSegment(i).getJoint().getName());
    jnt_names_.push_back(names);
    
    // Forward kinematics position solver
    boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver;
    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chains_[i]));
    fk_pos_solvers_.push_back(fk_pos_solver);

    // Forward kinematics velocity solver
    boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver;
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chains_[i]));
    fk_vel_solvers_.push_back(fk_vel_solver);

    // Dynamics solver
    boost::shared_ptr<KDL::ChainDynParam> dyn_solver;
    dyn_solver.reset(new KDL::ChainDynParam(kdl_chains_[i], g_vec_));
    dyn_solvers_.push_back(dyn_solver);
    
    // Jacobian solver
    boost::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
    jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chains_[i]));
    jacobian_solvers_.push_back(jacobian_solver);

    // initialize data (allow use in real-time?)
    jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
  }
  #ifdef DEBUG
    cerr << "Created kinematic and Dynamic chains" << endl;
  #endif    
}


void robotKDL::read_tree_param(string robot_desc_param, ros::NodeHandle &n)
{
  std::string robot_desc_string;
  n.param(robot_desc_param, robot_desc_string, std::string());
  // Build kdl tree:
  
  if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_)){
    ROS_ERROR("Failed to construct kdl tree from ros parameter %s", robot_desc_param.c_str());
  }
  dof = robot_tree_.getNrOfJoints();
  urdf_string_ = robot_desc_string;
}


void robotKDL::read_tree_file(string robot_file)
{
  if (!kdl_parser::treeFromFile(robot_file, robot_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf file: %s", robot_file.c_str());
  }
  dof = robot_tree_.getNrOfJoints();
  urdf_string_ = get_file_contents(robot_file.c_str());
 
}


void robotKDL::update_g_vec(vector<double> &grav_vector)
{
  KDL::Vector grav_vec(grav_vector[0], grav_vector[1], grav_vector[2]);
  g_vec_ = grav_vec;
}


void robotKDL::add_link_chains(vector <string> base_names, vector<string> link_names)
{
  // Generate link chains
  link_kdl_chains_.resize(link_names.size());
  for(int i = 0; i < link_names.size(); ++i)
  {
    string link_name = link_names[i];
    if(!robot_tree_.getChain(base_names[0], link_name, link_kdl_chains_[i]))
    {
      cerr << "Failed to construct kdl chain from " << base_names[0].c_str()
           << " to " << link_name.c_str() << endl;
    }

    // Forward kinematics position solver
    boost::shared_ptr<KDL::ChainFkSolverPos> link_fk_solver;
    link_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(link_kdl_chains_[i]));
    link_fk_solvers_.push_back(link_fk_solver);
    
    // Jacobian solver
    boost::shared_ptr<KDL::ChainJntToJacSolver> link_jacobian_solver;
    link_jacobian_solver.reset(new KDL::ChainJntToJacSolver(link_kdl_chains_[i]));
    link_jacobian_solvers_.push_back(link_jacobian_solver);

    // initialize data (allow use in real-time?)
    //jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    //H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());

    #ifdef DEBUG
      cerr << "Created kinematic and Dynamic chains for links " << i << " "
           << link_names.size() << endl;
    #endif
  }
}


robotKDL::robotKDL(string robot_desc_param, ros::NodeHandle &n, vector<string> base_names,
                   vector<string> EE_names, std::vector<double> grav_vector)
{
  update_g_vec(grav_vector);
  read_tree_param(robot_desc_param, n);
  robot_init(base_names, EE_names);
}


robotKDL::robotKDL(string file_name, vector<string> base_names, vector<string> EE_names,
                   std::vector<double> grav_vector)
{
  update_g_vec(grav_vector);
  read_tree_file(file_name);
  robot_init(base_names, EE_names);
}


robotKDL::robotKDL(string file_name, vector<string> base_names, vector<string> EE_names,
                   vector<string> link_names, std::vector<double> grav_vector)
{
  update_g_vec(grav_vector);
  read_tree_file(file_name);
  robot_init(base_names, EE_names);
  add_link_chains(base_names, link_names);
#ifdef DEBUG
    cerr << "initialized robotkdl!" << endl;
#endif
}  


robotKDL::robotKDL(string robot_desc_param, ros::NodeHandle &n, vector<string> base_names,
                   vector<string> EE_names, vector<string> link_names, std::vector<double> grav_vector)
{
  update_g_vec(grav_vector);
  read_tree_param(robot_desc_param, n);
  robot_init(base_names, EE_names);
  add_link_chains(base_names, link_names);
#ifdef DEBUG
    cerr << "initialized robotkdl!" << endl;
#endif
}  


robotKDL::robotKDL(string file_name, vector<string> base_names, vector<string> EE_names,
                   KDL::Vector grav_vector)
{
  read_tree_file(file_name);
  g_vec_=grav_vector;
  robot_init(base_names, EE_names);
 }


robotKDL::robotKDL(string file_name, string base_name, string EE_name, 
                   std::vector<double> grav_vector)
{
  update_g_vec(grav_vector);
  read_tree_file(file_name);
  manipulator_init(base_name, EE_name);
}


robotKDL::robotKDL(string file_name, string base_name, string EE_name, KDL::Vector grav_vector)
{
  g_vec_=grav_vector;
  read_tree_file(file_name);
  manipulator_init(base_name, EE_name);
}


void robotKDL::getFK(const int &chain_idx, const Eigen::VectorXd &joint_pos,
                     Eigen::VectorXd &out_pose, const bool &RPY) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  //Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  
  q.data = joint_pos;
  //  cerr<<q.data<<endl;
  fk_pos_solvers_[chain_idx]->JntToCart(q, p_out);

  frame_to_pose_(p_out, out_pose, RPY);
}


void robotKDL::getFK(const int &chain_idx, const Eigen::VectorXd &joint_pos,
                     Eigen::MatrixXd &frame_) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  
  q.data = joint_pos;
  //  cerr<<q.data<<endl;
  fk_pos_solvers_[chain_idx]->JntToCart(q,p_out);
  //Eigen::MatrixXd frame_(4,4);
  frame_.resize(4,4);
  frame_.setIdentity();
  // translation
  frame_(0,3) = p_out.p[0];
  frame_(1,3) = p_out.p[1];
  frame_(2,3) = p_out.p[2];

  for(int i = 0; i < 3; ++i)
  {
    frame_(i,0) = p_out.M(i,0);
    frame_(i,1) = p_out.M(i,1);
    frame_(i,2) = p_out.M(i,2);
  }
  
  //return frame_;
}


void robotKDL::getFK(const int &chain_idx, const Eigen::VectorXd &joint_pos,
                     Eigen::Affine3d &frame) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  
  q.data = joint_pos;
  fk_pos_solvers_[chain_idx]->JntToCart(q,p_out);

  // convert input eigen to KDL
  for (int i = 0; i < 3; ++i)
    frame(i,3) = p_out.p[i];
  for (int i = 0; i < 9; ++i)
    frame(i/3, i%3) = p_out.M.data[i];
}


void robotKDL::getTaskError(const int &chain_idx,
                            const Eigen::VectorXd &joint_pos, const Eigen::VectorXd &joint_vel,
                            const Eigen::Affine3d &task_pos_des, const Eigen::VectorXd &task_vel_des,
                            Eigen::VectorXd &task_pos_err, Eigen::VectorXd &task_vel_err)
{
  // TODO this is used in control and seems like too much allocation, should probably preallocate
  KDL::JntArrayVel q_qdot(joint_pos.size());
  KDL::FrameVel x_xdot;
  KDL::Frame x_des;
  KDL::Twist x_dot_des, x_err, x_dot_err;

  q_qdot.q.data = joint_pos;
  q_qdot.qdot.data = joint_vel;
  fk_vel_solvers_[chain_idx]->JntToCart(q_qdot, x_xdot);

  // convert input eigen to KDL
  for (int i = 0; i < 3; ++i)
    x_des.p[i] = task_pos_des(i, 3);
  for (int i = 0; i < 9; ++i)
    x_des.M.data[i] = task_pos_des(i/3, i%3);
  for (int i = 0; i < 6; ++i)
    x_dot_des(i) = task_vel_des[i];


  // ROS_INFO_STREAM("XDESROT:\n");
  // for (int i = 0; i < 9; ++i)
  //   ROS_INFO_STREAM(x_des.M.data[i]);
  // ROS_INFO_STREAM("ACTUALROT:\n");
  // for (int i = 0; i < 9; ++i)
  //   ROS_INFO_STREAM(x_xdot.GetFrame().M.data[i]);
  // ROS_INFO_STREAM("XDESPOS:\n");
  // for (int i = 0; i < 3; ++i)
  //   ROS_INFO_STREAM(x_des.p.data[i]);
  // ROS_INFO_STREAM("ACTUAL_POS:\n");
  // for (int i = 0; i < 3; ++i)
  //   ROS_INFO_STREAM(x_xdot.GetFrame().p.data[i]);
   

  

  // compute errors
  x_err = KDL::diff(x_xdot.GetFrame(), x_des);
  x_dot_err = KDL::diff(x_xdot.GetTwist(), x_dot_des);

  // transfer back to Eigen
  for (int i = 0; i < 6; ++i)
  {
    task_pos_err[i] = x_err[i];
    task_vel_err[i] = x_dot_err[i];
  }
}


void robotKDL::getLinkPose(const int &link_num, const Eigen::VectorXd &joint_pos,
                           Eigen::VectorXd &pose) const
{
  KDL::JntArray q(link_kdl_chains_[link_num].getNrOfJoints());
  KDL::Frame p_out;

  // map joint position array to joint names if they are the same size
  // assuming user sent the whole joint state
  // TODO: write as a seperate function
  if(joint_pos.size()==link_jnt_names_.size())
  {
    /*    for (int i=0;i<q.size();++i)
    {
      // get index
      link_kdl_chains_[link_num].
      q[i]=
      }*/
  }
  
  q.data = joint_pos.head(link_kdl_chains_[link_num].getNrOfJoints());
  link_fk_solvers_[link_num]->JntToCart(q, p_out);
  frame_to_pose_(p_out, pose, false);
  
}

void robotKDL::getLinkPose(const int &link_num, const Eigen::VectorXd &joint_pos,
                           Eigen::MatrixXd &out_frame) const
{
  KDL::JntArray q(link_kdl_chains_[link_num].getNrOfJoints());
  KDL::Frame p_out;

  q.data = joint_pos.head(link_kdl_chains_[link_num].getNrOfJoints());
  link_fk_solvers_[link_num]->JntToCart(q, p_out);

  out_frame.resize(4,4);
  out_frame.setIdentity();
  // translation
  out_frame(0,3) = p_out.p[0];
  out_frame(1,3) = p_out.p[1];
  out_frame(2,3) = p_out.p[2];

  for(int i = 0; i < 3; ++i)
  {
    out_frame(i,0) = p_out.M(i,0);
    out_frame(i,1) = p_out.M(i,1);
    out_frame(i,2) = p_out.M(i,2);
  }

}


void robotKDL::getLinkPoses(const Eigen::VectorXd &joint_pos, vector<Eigen::VectorXd> &cart_poses) const
{
  //vector<vector<double>> r_poses;
  cart_poses.resize(link_fk_solvers_.size());
  for (int i = 0; i < link_fk_solvers_.size(); ++i)
  {
    KDL::JntArray q(link_kdl_chains_[i].getNrOfJoints());
    KDL::Frame p_out;
    
    q.data = joint_pos.head(link_kdl_chains_[i].getNrOfJoints());//segment

    link_fk_solvers_[i]->JntToCart(q, p_out);
    frame_to_pose_(p_out, cart_poses[i], false);
  }
}


void robotKDL::getLinkPoses(const Eigen::VectorXd &joint_pos, vector<vector<double>> &cart_poses) const
{
  //vector<vector<double>> r_poses;
  cart_poses.resize(link_fk_solvers_.size());
  for (int i = 0; i < link_fk_solvers_.size(); ++i)
  {
    KDL::JntArray q(link_kdl_chains_[i].getNrOfJoints());
    KDL::Frame p_out;
    q.data = joint_pos.head(link_kdl_chains_[i].getNrOfJoints());//segment
    link_fk_solvers_[i]->JntToCart(q, p_out);
    frame_to_pose_(p_out, cart_poses[i]);
  }
}


void robotKDL::frame_to_pose_(KDL::Frame &T,  vector<double> &pose) const
{
  pose.resize(7);
  pose[0] = T.p[0];
  pose[1] = T.p[1];
  pose[2] = T.p[2];
    
  T.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);    
}


void robotKDL::frame_to_pose_(KDL::Frame &T, Eigen::VectorXd &pose, const bool &RPY) const
{
  if(RPY)
    pose.resize(6);
  else
    pose.resize(7);
  pose[0] = T.p[0];
  pose[1] = T.p[1];
  pose[2] = T.p[2];
  if(RPY)
    T.M.GetRPY(pose[3], pose[4], pose[5]);
  else
    T.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
}


void robotKDL::frame_to_pose_(const Eigen::MatrixXd &T, vector<double> &pose, const bool &RPY) const
{
  if(RPY)
    pose.resize(6);
  else
    pose.resize(7);
  pose[0] = T(0,3);
  pose[1] = T(1,3);
  pose[2] = T(2,3);
  // Eigen to rotation matrix:
  tf::Matrix3x3 R(T(0,0), T(0,1), T(0,2),
                  T(1,0), T(1,1), T(1,2),
                  T(2,0), T(2,1), T(2,2));
  
  if(RPY)
    R.getRPY(pose[3], pose[4], pose[5]);
  else
  {
    tf::Quaternion q;
    R.getRotation(q);
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();
  } 
}


Eigen::Vector3d robotKDL::get_RPY(Eigen::MatrixXd T)
{
  Eigen::Vector3d rpy(0.0,0.0,0.0);
  if(T.cols() == 4)
  {
  }
  else if(T.cols() == 3)
  {
    tf::Matrix3x3 R_;
    tf::matrixEigenToTF(T,R_);
    R_.getRPY(rpy[0], rpy[1], rpy[2]);
  }
  else
  {
    
  }
  return rpy;
}


vector<double> robotKDL::get_q_RPY(const double &R,const  double &P,const double &Y) const
{
  tf::Matrix3x3 R_;
  R_.setRPY(R,P,Y);
  tf::Quaternion q;
  R_.getRotation(q);
  vector<double> q_;
  q_.resize(4);
  q_[0] = q.x();
  q_[1] = q.y();
  q_[2] = q.z();
  q_[3] = q.w();
  return q_;
}


vector<double> robotKDL::get_RPY_q(const double &x, const double &y,const double &z,
                                   const double &w) const
{
  tf::Matrix3x3 R_;
  tf::Quaternion q(x, y, z, w);
  R_.setRotation(q);
  vector<double> rpy_;
  rpy_.resize(3);
  R_.getRPY(rpy_[0], rpy_[1], rpy_[2]);
  
  return rpy_;
}


void robotKDL::euler_diff(const vector<double> &r_1, const vector<double> &r_d,
                          vector<double> &orient_diff) const
{
  orient_diff.resize(3);

  KDL::Rotation R;
  KDL::Rotation R_1=R.RPY(r_1[0], r_1[1], r_1[2]);
  KDL::Rotation R_d=R.RPY(r_d[0], r_d[1], r_d[2]);
  KDL::Frame F_1(R_1);
  KDL::Frame F_d(R_d);

  KDL::Twist x_err=KDL::diff(F_1,F_d);

  Eigen::Matrix<double, 6, 1> err;
  tf::twistKDLToEigen(x_err, err);
  orient_diff[0] = err(3,0);
  orient_diff[1] = err(4,0);
  orient_diff[2] = err(5,0);
}

void robotKDL::euler_diff(const Eigen::MatrixXd &r_1, const Eigen::MatrixXd &r_d,
                          vector<double> &orient_diff) const
{
  // get pose from frames:
  vector<double> pose_1;
  frame_to_pose_(r_1, pose_1, true);
  vector<double> pose_d;
  frame_to_pose_(r_d, pose_d, true);
  
  orient_diff.resize(3);

  KDL::Rotation R;
  KDL::Rotation R_1 = R.RPY(pose_1[3], pose_1[4], pose_1[5]);
  KDL::Rotation R_d = R.RPY(pose_d[3], pose_d[4], pose_d[5]);
  KDL::Frame F_1(R_1);
  KDL::Frame F_d(R_d);

  KDL::Twist x_err=KDL::diff(F_1, F_d);

  Eigen::Matrix<double, 6, 1> err;
  tf::twistKDLToEigen(x_err, err);
  orient_diff[0] = err(3,0);
  orient_diff[1] = err(4,0);
  orient_diff[2] = err(5,0);
}


void robotKDL::pose_to_frame(const vector<double> &pose, Eigen::MatrixXd &mat) const
{
  mat.resize(4,4);
  mat.setIdentity();
  tf::Matrix3x3 R;
  if(pose.size() == 6)
  {
    R.setRPY(pose[3], pose[4], pose[5]);
  }
  else
  {
    tf::Quaternion q(pose[3], pose[4], pose[5], pose[6]);
    R.setRotation(q);
  }
  
  for (int i = 0; i < 3; i++)
  {
    mat(i,0) = R[i].x();
    mat(i,1) = R[i].y();
    mat(i,2) = R[i].z();
  }
  mat(0,3) = pose[0];
  mat(1,3) = pose[1];
  mat(2,3) = pose[2];
}


void robotKDL::pose_to_frame(const Eigen::VectorXd &pose, Eigen::MatrixXd &mat) const
{
  mat.resize(4,4);
  mat.setIdentity();
  tf::Matrix3x3 R;
  if(pose.size() == 6)
  {
    R.setRPY(pose[3], pose[4], pose[5]);
  }
  else
  {
    tf::Quaternion q(pose[3], pose[4], pose[5], pose[6]);
    R.setRotation(q);
  }
  
  for (int i = 0; i < 3; i++)
  {
    mat(i,0) = R[i].x();
    mat(i,1) = R[i].y();
    mat(i,2) = R[i].z();
  }
  mat(0,3) = pose[0];
  mat(1,3) = pose[1];
  mat(2,3) = pose[2];
}


void robotKDL::getGtau(const int &chain_idx, const Eigen::VectorXd &joint_pos,
                       Eigen::VectorXd &tau_g_) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::JntArray tau_g(joint_pos.size());
  
  q.data = joint_pos;
  dyn_solvers_[chain_idx]->JntToGravity(q, tau_g);
  tau_g_ = tau_g.data;
}


void robotKDL::getCtau(const int &chain_idx, const Eigen::VectorXd &joint_pos,
                       const Eigen::VectorXd &joint_vel, Eigen::VectorXd &tau_c_) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::JntArray q_dot(joint_vel.size());
  KDL::JntArray tau_c(joint_pos.size());
  
  q.data = joint_pos;
  q_dot.data = joint_vel;
  dyn_solvers_[chain_idx]->JntToCoriolis(q, q_dot, tau_c);
  tau_c_ = tau_c.data;
}
void robotKDL::getJointLimits(vector<string> &j_names,vector<double> &max_bounds, vector<double> &min_bounds)
{
  max_bounds.clear();
  min_bounds.clear();
  dof=0;
  urdf::Model model;
  if (!model.initString(urdf_string_))
  {
    ROS_ERROR("Failed to parse urdf file");
  }
  
  for(int j=0;j<j_names.size();++j)
  {
    urdf::Joint jnt=*model.getJoint(j_names[j]);
    min_bounds.emplace_back(jnt.limits->lower);
    max_bounds.emplace_back(jnt.limits->upper);
    dof += 1;
  }
  low_bounds=min_bounds;
  up_bounds=max_bounds;
  // store joint names in link_j_names:
  link_jnt_names_=j_names;
}

void robotKDL::getJointLimits(const int &chain_idx,vector<double> &max_bounds, vector<double> &min_bounds)
{
  dof=0;
  max_bounds.clear();
  min_bounds.clear();
  // get joint names belonging to chain:
  vector<string> j_names=getJointNames(chain_idx);
  // read urdf model:
  urdf::Model model;
  if (!model.initString(urdf_string_))
  {
    ROS_ERROR("Failed to parse urdf file");
  }
  
  for(int j=0;j<j_names.size();++j)
  {
    urdf::Joint jnt=*model.getJoint(j_names[j]);
    low_bounds.emplace_back(jnt.limits->lower);
    up_bounds.emplace_back(jnt.limits->upper);
    dof += 1;
  }
  cerr << "read joint limits, dof: " << dof << endl;
  
}

void robotKDL::urdfParser()
{
  // function updates bounds by using urdf parser:

  urdf::Model model;
  if (!model.initString(urdf_string_))
  {
    ROS_ERROR("Failed to parse urdf file");
  }
  dof = 0;
  /*
  for(int i=0;i<kdl_chains_[0].getNrOfSegments();++i)
  {
    if(kdl_chains_[0].getSegment(i).getJoint().getType()!=KDL::Joint::JointType::None)
    {
      low_bounds.emplace_back();
      up_bounds.emplace_back();
    }
      
  }
  */
  for(map<string,boost::shared_ptr< urdf::Joint >>::iterator it = model.joints_.begin();
      it != model.joints_.end(); ++it)
  {
    if (model.joints_[it->first]->type != urdf::Joint::UNKNOWN
        && model.joints_[it->first]->type != urdf::Joint::FIXED)
    {
      low_bounds.emplace_back(model.joints_[it->first]->limits->lower);
      up_bounds.emplace_back(model.joints_[it->first]->limits->upper);
      dof += 1;
    }
  }
  cerr << "read joint limits, dof: " << dof << endl;
}


void robotKDL::getJacobian(const int &idx_chain, const Eigen::VectorXd &jnt_array,
                           Eigen::MatrixXd &J_mat) const
{
  KDL::JntArray q(jnt_array.size());
  // std vec to eigen
  //Map<Eigen::VectorXd> q_arr(&jnt_array[0],jnt_array.size());

  q.data = jnt_array;

  jacobian_solvers_[idx_chain]->JntToJac(q, *jacobians_[idx_chain]);
  //cerr<<"Jacobian computed!"<<endl;
  J_mat = jacobians_[idx_chain]->data;
  //return jacobians_[idx_chain]->data;
}


void robotKDL::getLinkJacobian(const int &idx_link,const  Eigen::VectorXd &jnt_array,
                               Eigen::MatrixXd &J_mat) const
{
  KDL::JntArray q(link_kdl_chains_[idx_link].getNrOfJoints());
  // std vec to eigen
  
  //Map<Eigen::VectorXd> q_arr(&jnt_array[0],jnt_array.size());
  //cerr<<q_arr<<endl;
  q.data = jnt_array.head(link_kdl_chains_[idx_link].getNrOfJoints());
  KDL::Jacobian l_jacobians_(link_kdl_chains_[idx_link].getNrOfJoints());

  int er = link_jacobian_solvers_[idx_link]->JntToJac(q, l_jacobians_);
  J_mat = l_jacobians_.data;
}


void robotKDL::getM(const int &idx_chain, const Eigen::VectorXd &jnt_array, Eigen::MatrixXd &M) const
{
  KDL::JntArray q(jnt_array.size());
  q.data = jnt_array;
  dyn_solvers_[idx_chain]->JntToMass(q, *H_mat_[idx_chain]);
  M=H_mat_[idx_chain]->data;
}


// From http://insanecoding.blogspot.com/2011/11/how-to-read-in-file-in-c.html
std::string robotKDL::get_file_contents(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    return(std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()));
  }
  throw(errno);
}


void robotKDL::getSTDVector(const Eigen::VectorXd &in, vector<double> &out)
{
  out.resize(in.size());
  Eigen::VectorXd::Map(&out[0], in.size()) = in;
}


vector<string> robotKDL::getJointNames()
{
  vector<int> idxs;
  for (int i = 0; i < jnt_names_.size(); ++i)
    idxs.push_back(i);
  return getJointNames(idxs);
}


vector<string> robotKDL::getJointNames(int chain_idx)
{
  return jnt_names_[chain_idx];
}

void robotKDL::getRootName(int chain_idx, std::string& root_name)
{
  root_name = jnt_names_[0][0];
}


vector<string> robotKDL::getJointNames(vector<int> &chain_idxs)
{
  vector<string> jnt_names;
  vector<string> chain_jnt_names;
  for (int i = 0; i < chain_idxs.size(); ++i)
  {
    chain_jnt_names = getJointNames(chain_idxs[i]);
    for (int j = 0; j < chain_jnt_names.size(); ++j)
      jnt_names.push_back(chain_jnt_names[j]);
  }
  return jnt_names;
}


int robotKDL::getNumChains()
{
  return kdl_chains_.size();
}

int robotKDL::getNumLinks()
{
  return link_fk_solvers_.size();
}


} // namespace manipulator_kdl
