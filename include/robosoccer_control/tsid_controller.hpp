/*
header file for the TSID controller.
responsible for:
- initializing the TSID controller
- subscribing to the joint states
- publishing the joint commands
- publishing the computed torques


uses op3 for configuration - is that correct???
*/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

// tsi library includes (need to confirmm if this is the correct library)
#include <tsid/robot/robot-wrapper.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-am-equality.hpp>
#include <tsid/tasks/task-cop-equality.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/math/utils.hpp>

namespace robosoccer_control
{

struct TSIDConfig
{
  // robot parameters (from op3)
  double dt = 0.002;
  double g = 9.81;
  double z0 = 0.4;
  
  // Task weights (from op3)
  double w_com = 1.0;
  double w_am = 1e-3;        // Angular momentum
  double w_foot = 1e-1;      // Foot motion
  double w_contact = -1.0;   // Contact (negative = infinite weight)
  double w_posture = 1e-1;   // Joint posture
  double w_forceRef = 1e-5;  // Force regularization
  double w_cop = 0.0;        // Center of Pressure
  double w_torque_bounds = 1e-1;
  double w_joint_bounds = 0.0;
  
  // Foot parameters
  double lxp = 0.0275;  // foot length in positive y direction
  double lxn = 0.0275;  // foot length in negative y direction  
  double lyp = 0.055;   // foot length in positive x direction
  double lyn = 0.055;   // foot length in negative x direction
  double lz = 0.0;      // foot sole height
  double mu = 0.5;      // friction coefficient
  double fMin = 0.0;    // minimum normal force
  double fMax = 1000.0; // maximum normal force
  
  // Control gains
  double kp_contact = 10.0;
  double kp_foot = 10.0;
  double kp_com = 10.0;
  double kp_am = 10.0;
  double kp_posture = 1.0;
  
  // Bounds scaling
  double tau_max_scaling = 3.0;
  double v_max_scaling = 10.0;
  
  // Frame names (matching reference)
  std::string rf_frame_name = "feet_2";  // Right foot frame
  std::string lf_frame_name = "feet";    // Left foot frame
  
  // Contact normal
  Eigen::Vector3d contactNormal = Eigen::Vector3d(0.0, 0.0, 1.0);
};

class TSIDController
{
public:
  TSIDController();
  ~TSIDController() = default;

  // Initialization
  bool initialize(const std::string& urdf_path, const TSIDConfig& config = TSIDConfig{});
  
  // state update
  void updateRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
  
  // Main control computation
  bool computeControlTorques(Eigen::VectorXd& tau_out);
  
  // Task references (matching reference Python API)
  void setCoMReference(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc);
  void setFootReference(const std::string& foot_name, const Eigen::Matrix4d& H_ref);
  void setPostureReference(const Eigen::VectorXd& q_ref);
  
  // Contact management
  void addFootContact(const std::string& foot_name);
  void removeFootContact(const std::string& foot_name);
  bool isFootInContact(const std::string& foot_name) const;
  
  // Getters
  Eigen::Vector3d getCurrentCoM() const;
  Eigen::Matrix4d getFootPosition(const std::string& foot_name) const;
  const Eigen::VectorXd& getCurrentConfiguration() const { return q_; }
  const Eigen::VectorXd& getCurrentVelocity() const { return v_; }

private:
  // TSID components (matching reference biped.py structure)
  std::shared_ptr<tsid::robots::RobotWrapper> robot_;
  std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> formulation_;
  std::shared_ptr<tsid::solvers::SolverHQPBase> solver_;
  
  // Tasks (matching reference)
  std::shared_ptr<tsid::tasks::TaskComEquality> comTask_;
  std::shared_ptr<tsid::tasks::TaskAMEquality> amTask_;
  std::shared_ptr<tsid::tasks::TaskCopEquality> copTask_;
  std::shared_ptr<tsid::tasks::TaskJointPosture> postureTask_;
  std::shared_ptr<tsid::tasks::TaskSE3Equality> leftFootTask_;
  std::shared_ptr<tsid::tasks::TaskSE3Equality> rightFootTask_;
  std::shared_ptr<tsid::tasks::TaskActuationBounds> actuationBoundsTask_;
  std::shared_ptr<tsid::tasks::TaskJointBounds> jointBoundsTask_;
  
  // Contacts
  std::shared_ptr<tsid::contacts::Contact6d> contactRF_;
  std::shared_ptr<tsid::contacts::Contact6d> contactLF_;
  
  // Trajectories
  std::shared_ptr<tsid::trajectories::TrajectorySE3> trajLF_;
  std::shared_ptr<tsid::trajectories::TrajectorySE3> trajRF_;
  std::shared_ptr<tsid::trajectories::TrajectoryEuclidian> trajCom_;
  std::shared_ptr<tsid::trajectories::TrajectoryEuclidian> trajPosture_;
  
  // State
  Eigen::VectorXd q_, v_;
  Eigen::VectorXd q_ref_, v_ref_;
  
  // Configuration
  TSIDConfig config_;
  
  // Contact state tracking
  bool contact_LF_active_;
  bool contact_RF_active_;
  
  // Frame IDs
  pinocchio::FrameIndex LF_frame_id_;
  pinocchio::FrameIndex RF_frame_id_;
  
  // Helper methods
  void initializeTasks();
  void initializeContacts();
  void initializeTrajectories();
  void updateTaskReferences();
  Eigen::MatrixXd getContactPoints() const;
};

} // namespace robosoccer_control 