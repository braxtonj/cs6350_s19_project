#ifndef LL4MA_ROBOT_INTERFACE
#define LL4MA_ROBOT_INTERFACE

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
#include <ll4ma_robot_interface/types.h>
#include <ll4ma_robot_control_msgs/RobotState.h>


namespace robot_interface
{

  enum LogLevel
  {
    INFO, WARN, ERROR
  };
  
  class RobotInterface
  {
  protected:
    int num_jnts_;
    std::string jnt_cmd_topic_, robot_state_topic_;
    std::vector<std::string> jnt_names_, tip_links_, root_links_;
    std::vector<double> torque_lims_;
    std::vector<double> gravity_ = {0.0,0.0,-9.8};

    // Eigen
    Eigen::VectorXd tau_, tau_g_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher jnt_cmd_pub_, robot_state_pub_;
    sensor_msgs::JointState jnt_cmd_;
    ll4ma_robot_control_msgs::RobotState robot_state_;

    void log(std::string msg);
    void log(std::string msg, LogLevel level);
    void saturateTorques(Eigen::VectorXd &torques);
    void initJointState(sensor_msgs::JointState &jnt_state);

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
      
      m_pinv = svd.matrixV().leftCols(sing_vals.size()) * sing_vals.asDiagonal()
               * svd.matrixU().leftCols(sing_vals.size()).transpose();
    }

    
  public:
    // KDL
    boost::shared_ptr<manipulator_kdl::robotKDL> kdl_;

    // Constructors
    RobotInterface(std::string ns) : nh_(ns) {}

    // Public functions
    virtual bool init();
    virtual void publishTorqueCommand(Eigen::VectorXd &torques);
    
    /**
     * @brief Publishes joint state command to the real robot if it has its own low level controller. 
     * 
     * [Optional, use only if your robot has its own low level controller]
     * @param cmd the joint command values
     * @param cm enum to switch between [position,velocity, effort]
     */
    virtual void publishRealRobot(const Eigen::VectorXd &cmd,ControlMode &cm);
    virtual void publishRealRobot(const Eigen::VectorXd &cmd, const Eigen::VectorXd &q_des_cmd);

    virtual void publishRobotState(Eigen::VectorXd &q, Eigen::VectorXd &q_dot);

    virtual void publishRobotState(Eigen::VectorXd &q, Eigen::VectorXd &q_dot,
                                   Eigen::Affine3d &x, Eigen::VectorXd &x_dot);
    
    virtual void compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &gravity);

    double getNumJoints();
    std::vector<std::string> getRootNames();
    std::vector<std::string> getTipNames();

    void getFK(Eigen::VectorXd &q, Eigen::Affine3d &x, const int &ch_idx=0);
    void getJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &J,const int &ch_idx=0);
    void getJointMass(Eigen::VectorXd &q, Eigen::MatrixXd &Mq);
    void getTaskMass(Eigen::MatrixXd &Mq, Eigen::MatrixXd &J, Eigen::MatrixXd &Mx);
    void getTaskError(Eigen::VectorXd &q, Eigen::VectorXd &q_dot,
                      Eigen::Affine3d &x_des, Eigen::VectorXd &x_dot_des,
                      Eigen::VectorXd &x_err, Eigen::VectorXd &x_dot_err);
  };
}

#endif
