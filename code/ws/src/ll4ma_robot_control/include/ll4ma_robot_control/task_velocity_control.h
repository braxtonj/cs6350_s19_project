#ifndef LL4MA_JS_INV_DYN_CONTROL
#define LL4MA_JS_INV_DYN_CONTROL

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include "control.h"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <tf/transform_listener.h>

class TaskVelocityController : public Controller
{
 protected:
  std::string task_des_topic_, joint_des_topic_, base_link_;
  double pseudoinverse_tolerance_, dq_;
  bool use_redundancy_resolution_;

  // Eigen
  Eigen::VectorXd x_dot_des_, q_dot_des_, q_dot_posture_, q_prime_;
  Eigen::MatrixXd J_, J_cross_, J_null_, I_N_, JJ_trans_, J_prime_;
  tf::Vector3 out_rot_, out_vel_;
  tf::StampedTransform transform_;
  // ROS
  ros::Subscriber task_des_sub_;
  ros::Publisher joint_des_pub_;
  ros::ServiceServer reload_srv_;
  sensor_msgs::JointState q_dot_des_ros_;
  shared_ptr<tf::TransformListener> tf_;

  void taskDesCallback(geometry_msgs::TwistStamped cmd);
  bool reloadControlParameters(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
  void log(std::string msg);
  void log(std::string msg, LogLevel level);
  template<typename Derived>
  bool getGainsFromParamServer(std::string param, ros::NodeHandle &nh, Eigen::MatrixBase<Derived> &v)
  {
    std::vector<double> temp;
    bool success = nh.getParam(param, temp);
    for (int i = 0; i < temp.size(); i++)
      v(i, i) = temp[i];
    return success;
  }

  // TODO: Move these functions into a utility library?
  /*
   * Compute the pseudoinverse for the input matrix m, store to input m_pinv.
   * Values less than tolerance are set to zero to avoid control blowups.
   */
  // TODO: do you still need templates now that matrices are dynamic?
  template<typename Derived1, typename Derived2>
  void getPseudoInverse(const Eigen::MatrixBase<Derived1> &m,
                        Eigen::MatrixBase<Derived2> &m_pinv,
                        double tolerance)
  {
    using namespace Eigen;

    JacobiSVD<typename Derived1::PlainObject> svd(m, ComputeFullU | ComputeFullV);
    typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType sing_vals = svd.singularValues();
    // set values within tolerance to zero
    for (int idx = 0; idx < sing_vals.size(); idx++)
    {
      if (tolerance > 0.0 && sing_vals(idx) > tolerance)
        sing_vals(idx) = 1.0 / sing_vals(idx);
      else
        sing_vals(idx) = 0.0;
    }

    m_pinv = svd.matrixV().leftCols(sing_vals.size()) * sing_vals.asDiagonal() * svd.matrixU().leftCols(sing_vals.size()).transpose();
  }

  /**
   * Project the input q_dot_posture_ onto the null space of the jacobian J
   * Values less than tolerance are set to zero to avoid control blowups.
   * 
   * @param q_dot Output of the projection is added to this vector
   * @param q_dot_posture_null  Input desired velocity to be mapped to the null space
   * @param J manipulator Jacobian
   * @param J_cross_ pseudoinverse of the manipulator Jacobian
   */
  template<typename Derived1, typename Derived2, typename Derived3>
  void nullSpaceProjection(Eigen::MatrixBase<Derived1> &q_dot,
                           const Eigen::MatrixBase<Derived1> &q_dot_null,
                           const Eigen::MatrixBase<Derived2> &J,
                           const Eigen::MatrixBase<Derived3> &J_cross)
  {
    q_dot += (I_N_ - J_cross * J)*q_dot_null;
  }

  void computeManipulabilityRedundancyResolution(Eigen::VectorXd &q_dot_posture_,
                                                 Eigen::MatrixXd &J_);

  void computeJointLimitRedundancyResolution(Eigen::VectorXd &q_dot_posture_,
                                             Eigen::MatrixXd &J_);
  double computeManipulabilityScore(Eigen::MatrixXd& JJ_trans_);
 public:
  TaskVelocityController(std::string ns, robot_interface::RobotInterface *robot_interface, double rate, int robot_chain_idx=0)
      : Controller(ns, robot_interface, rate, robot_chain_idx) {}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};

#endif
