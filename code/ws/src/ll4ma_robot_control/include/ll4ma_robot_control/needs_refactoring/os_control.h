#ifndef LL4MA_OS_CONTROL
#define LL4MA_OS_CONTROL

#include "base_control.h"
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "chainjnttojacdotsolver.hpp"
#include <ll4ma_robot_control/RobotState.h>
#include <ll4ma_robot_control/OpSpaceCommand.h>


namespace controller_interface
{
  class OpSpaceController: public BaseController
  {
  protected:
    bool use_posture_, use_ft_sensor_, transform_wrench_, wrench_state_received_, forces_zeroed_;
    double num_logged_, time_sec_;

    std::string ft_topic_, filtered_ft_topic_, os_cmd_topic_, ft_link_;
    std::vector<double> force_lims_;

    // KDL
    KDL::Frame x_kdl_, x_ft_kdl_, x_des_kdl_;
    KDL::Twist x_err_kdl_, xdot_kdl_, xdot_des_kdl_, xdot_err_kdl_, xdotdot_des_kdl_, jdot_qdot_kdl_;
    KDL::FrameVel x_xdot_kdl_;
    KDL::Jacobian J_kdl_;
    KDL::JntSpaceInertiaMatrix Mq_kdl_;
    KDL::Wrench w_raw_, w_filt_, w_trans_;
    boost::scoped_ptr<KDL::ChainFkSolverVel> fk_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;

    // Eigen
    Eigen::VectorXd x_err_, x_err_accum_, xdot_err_, xdotdot_des_, Jdot_qdot_, ux_;
    Eigen::VectorXd q_, qdot_, q_null_des_, tau_, tau_posture_;
    Eigen::MatrixXd J_, Mq_, I_N_, Kp_null_, Kd_null_, Mx_, I_6_, Kp_, Kd_, I_3_;
    Eigen::Affine3d I_4_;                          

    // ROS
    ros::Subscriber os_cmd_sub_, wrench_state_sub_, filtered_wrench_sub_;
    ros::Publisher robot_state_pub_;

    ll4ma_robot_control::RobotState robot_state_; // reporting state to other nodes
    void opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg);
    void jointStateCallback(sensor_msgs::JointState);
    void wrenchStateCallback(geometry_msgs::WrenchStamped wrench_stmp);
    void filteredWrenchStateCallback(geometry_msgs::WrenchStamped wrench_stmp);

    // FT Sensor
    int ft_index_;
            
    /*
     * Compute the pseudoinverse for the input matrix m, store to input m_pinv.
     * Values less than tolerance are set to zero to avoid control blowups.
     */
    // TODO do you still need templates now that matrices are dynamic?
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

    /* void getPseudoInverse(Eigen::MatrixXd &m, Eigen::MatrixXd &m_pinv, double tolerance) */
    /* { */
    /*   Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV); */
    /*   Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues(); */
    /*   // set values within tolerance to zero */
    /*   for (int idx = 0; idx < sing_vals.size(); idx++) */
    /*   { */
    /*     if (tolerance > 0.0 && sing_vals(idx) > tolerance) */
    /*       sing_vals(idx) = 1.0 / sing_vals(idx); */
    /*     else */
    /*       sing_vals(idx) = 0.0; */
    /*   } */
      
    /*   m_pinv = svd.matrixV().leftCols(sing_vals.size()) */
    /* 	* sing_vals.asDiagonal() */
    /* 	* svd.matrixU().leftCols(sing_vals.size()).transpose(); */
    /* } */

    /*
     * Compute the pseudoinverse for the input matrix m, store to input m_pinv.
     * Values less than tolerance are set to zero to avoid control blowups.
     */
    template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    void nullSpaceProjection(Eigen::MatrixBase<Derived1> &tau,
			     const Eigen::MatrixBase<Derived1> &tau_null,
			     const Eigen::MatrixBase<Derived2> &J,
			     const Eigen::MatrixBase<Derived3> &Mx,
			     const Eigen::MatrixBase<Derived4> &Mq)
    {
      tau += (I_N_ - ((Mq.inverse() * J.transpose() * Mx) * J).transpose()) * tau_null;
    }

    void blockTensorMatrix(Eigen::MatrixXd &M, const Eigen::MatrixXd &R, const Eigen::MatrixXd &S);

    
  public:
    OpSpaceController(double rate, ros::NodeHandle &nh) : BaseController(rate, nh) {}

    bool configureHook();
    void setCurrentConfigAsDesired();
    void publishCurrentRobotState();
  };
  
} // namespace controller_interface

#endif
