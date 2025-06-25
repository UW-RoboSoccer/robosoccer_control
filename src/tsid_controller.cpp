/*
implementation of the TSID controller.
responsible for:
- initializing the TSID controller
- subscribing to the joint states
- publishing the joint commands
- publishing the computed torques

matches ethan's tsid_control/biped.py for now
*/

#include "robosoccer_control/tsid_controller.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>

namespace robosoccer_control
{

TSIDController::TSIDController()
: contact_LF_active_(true)
, contact_RF_active_(true)
{
}

bool TSIDController::initialize(const std::string& urdf_path, const TSIDConfig& config)
{
  try {
    config_ = config;
    
    // Initialize robot model using TSID RobotWrapper (matching reference biped.py)
    robot_ = std::make_shared<tsid::robots::RobotWrapper>(
      urdf_path,
      std::vector<std::string>{},
      pinocchio::JointModelFreeFlyer(),
      false
    );
    
    // Get frame IDs
    LF_frame_id_ = robot_->model().getFrameId(config_.lf_frame_name);
    RF_frame_id_ = robot_->model().getFrameId(config_.rf_frame_name);
    
    std::cout << "Left foot frame ID: " << LF_frame_id_ << std::endl;
    std::cout << "Right foot frame ID: " << RF_frame_id_ << std::endl;
    
    // Initialize configuration to neutral pose
    q_ = pinocchio::neutral(robot_->model());
    v_ = Eigen::VectorXd::Zero(robot_->nv());
    
    // Initialize TSID formulation (matching reference biped.py)
    formulation_ = std::make_shared<tsid::InverseDynamicsFormulationAccForce>(
      "tsid", *robot_, false);
    formulation_->computeProblemData(0.0, q_, v_);
    
    // Initialize tasks and contacts
    initializeTasks();
    initializeContacts();
    initializeTrajectories();
    
    // Initialize solver (matching reference)
    solver_ = tsid::solvers::SolverHQPFactory::createNewSolver(
      tsid::solvers::SOLVER_HQP_EIQUADPROG, "eiquadprog");
    solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());
    
    std::cout << "TSID Controller initialized successfully!" << std::endl;
    std::cout << "nVar: " << formulation_->nVar() << ", nEq: " << formulation_->nEq() 
              << ", nIn: " << formulation_->nIn() << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Error initializing TSID controller: " << e.what() << std::endl;
    return false;
  }
}

void TSIDController::initializeTasks()
{
  // Initialize all tasks matching reference biped.py structure
  
  // CoM Task
  comTask_ = std::make_shared<tsid::tasks::TaskComEquality>("task-com", *robot_);
  comTask_->setKp(config_.kp_com * Eigen::Vector3d::Ones());
  comTask_->setKd(2.0 * std::sqrt(config_.kp_com) * Eigen::Vector3d::Ones());
  formulation_->addMotionTask(comTask_, config_.w_com, 1, 0.0);
  
  // Angular Momentum Task
  amTask_ = std::make_shared<tsid::tasks::TaskAMEquality>("task-am", *robot_);
  Eigen::Vector3d kp_am_gains(config_.kp_am, config_.kp_am, 0.0);  // No control in yaw
  amTask_->setKp(kp_am_gains);
  amTask_->setKd(2.0 * kp_am_gains.cwiseSqrt());
  formulation_->addMotionTask(amTask_, config_.w_am, 1, 0.0);
  
  // Center of Pressure Task
  copTask_ = std::make_shared<tsid::tasks::TaskCopEquality>("task-cop", *robot_);
  formulation_->addForceTask(copTask_, config_.w_cop, 1, 0.0);
  
  // Joint Posture Task
  postureTask_ = std::make_shared<tsid::tasks::TaskJointPosture>("task-posture", *robot_);
  
  // Create gain vector (matching reference op3_conf.py gain_vector)
  Eigen::VectorXd gain_vector(robot_->nv() - 6);  // Excluding free-flyer
  gain_vector << 
    100.0, 100.0,  // head
    10.0, 5.0, 5.0, 1.0, 1.0,  // left leg
    10.0, 10.0, 10.0,  // left arm
    10.0, 5.0, 5.0, 1.0, 1.0,  // right leg
    10.0, 10.0, 10.0;  // right arm
  
  postureTask_->setKp(config_.kp_posture * gain_vector);
  postureTask_->setKd(2.0 * (config_.kp_posture * gain_vector).cwiseSqrt());
  
  // Set posture mask (all joints active)
  Eigen::VectorXd mask = Eigen::VectorXd::Ones(robot_->nv() - 6);
  postureTask_->setMask(mask);
  formulation_->addMotionTask(postureTask_, config_.w_posture, 1, 0.0);
  
  // Foot Tasks
  leftFootTask_ = std::make_shared<tsid::tasks::TaskSE3Equality>(
    "task-left-foot", *robot_, config_.lf_frame_name);
  leftFootTask_->setKp(config_.kp_foot * Eigen::Vector6d::Ones());
  leftFootTask_->setKd(2.0 * std::sqrt(config_.kp_foot) * Eigen::Vector6d::Ones());
  formulation_->addMotionTask(leftFootTask_, config_.w_foot, 1, 0.0);
  
  rightFootTask_ = std::make_shared<tsid::tasks::TaskSE3Equality>(
    "task-right-foot", *robot_, config_.rf_frame_name);
  rightFootTask_->setKp(config_.kp_foot * Eigen::Vector6d::Ones());
  rightFootTask_->setKd(2.0 * std::sqrt(config_.kp_foot) * Eigen::Vector6d::Ones());
  formulation_->addMotionTask(rightFootTask_, config_.w_foot, 1, 0.0);
  
  // Actuation bounds
  actuationBoundsTask_ = std::make_shared<tsid::tasks::TaskActuationBounds>(
    "task-actuation-bounds", *robot_);
  Eigen::VectorXd tau_max = config_.tau_max_scaling * robot_->model().effortLimit.tail(robot_->na());
  Eigen::VectorXd tau_min = -tau_max;
  actuationBoundsTask_->setBounds(tau_min, tau_max);
  if (config_.w_torque_bounds > 0.0) {
    formulation_->addActuationTask(actuationBoundsTask_, config_.w_torque_bounds, 0, 0.0);
  }
  
  // Joint bounds
  jointBoundsTask_ = std::make_shared<tsid::tasks::TaskJointBounds>(
    "task-joint-bounds", *robot_, config_.dt);
  Eigen::VectorXd v_max = config_.v_max_scaling * robot_->model().velocityLimit.tail(robot_->na());
  Eigen::VectorXd v_min = -v_max;
  jointBoundsTask_->setVelocityBounds(v_min, v_max);
  if (config_.w_joint_bounds > 0.0) {
    formulation_->addMotionTask(jointBoundsTask_, config_.w_joint_bounds, 0, 0.0);
  }
}

void TSIDController::initializeContacts()
{
  // Contact points (matching reference biped.py)
  Eigen::Matrix3Xd contact_points = getContactPoints();
  
  // Right foot contact
  contactRF_ = std::make_shared<tsid::contacts::Contact6d>(
    "contact_rfoot", *robot_, config_.rf_frame_name, contact_points,
    config_.contactNormal, config_.mu, config_.fMin, config_.fMax);
  contactRF_->setKp(config_.kp_contact * Eigen::Vector6d::Ones());
  contactRF_->setKd(2.0 * std::sqrt(config_.kp_contact) * Eigen::Vector6d::Ones());
  
  // Left foot contact  
  contactLF_ = std::make_shared<tsid::contacts::Contact6d>(
    "contact_lfoot", *robot_, config_.lf_frame_name, contact_points,
    config_.contactNormal, config_.mu, config_.fMin, config_.fMax);
  contactLF_->setKp(config_.kp_contact * Eigen::Vector6d::Ones());
  contactLF_->setKd(2.0 * std::sqrt(config_.kp_contact) * Eigen::Vector6d::Ones());
  
  // Add contacts to formulation
  pinocchio::Data& data = formulation_->data();
  
  pinocchio::SE3 H_rf_ref = robot_->framePosition(data, RF_frame_id_);
  contactRF_->setReference(H_rf_ref);
  if (config_.w_contact >= 0.0) {
    formulation_->addRigidContact(contactRF_, config_.w_forceRef, config_.w_contact, 1);
  } else {
    formulation_->addRigidContact(contactRF_, config_.w_forceRef);
  }
  
  pinocchio::SE3 H_lf_ref = robot_->framePosition(data, LF_frame_id_);
  contactLF_->setReference(H_lf_ref);
  if (config_.w_contact >= 0.0) {
    formulation_->addRigidContact(contactLF_, config_.w_forceRef, config_.w_contact, 1);
  } else {
    formulation_->addRigidContact(contactLF_, config_.w_forceRef);
  }
}

void TSIDController::initializeTrajectories()
{
  pinocchio::Data& data = formulation_->data();
  
  // CoM trajectory
  Eigen::Vector3d com_ref = robot_->com(data);
  trajCom_ = std::make_shared<tsid::trajectories::TrajectoryEuclidianConstant>("traj_com", com_ref);
  
  // Posture trajectory
  q_ref_ = q_.tail(robot_->nv() - 6);  // Exclude free-flyer
  trajPosture_ = std::make_shared<tsid::trajectories::TrajectoryEuclidianConstant>("traj_joint", q_ref_);
  
  // Foot trajectories
  pinocchio::SE3 H_lf_ref = robot_->framePosition(data, LF_frame_id_);
  pinocchio::SE3 H_rf_ref = robot_->framePosition(data, RF_frame_id_);
  
  trajLF_ = std::make_shared<tsid::trajectories::TrajectorySE3Constant>("traj-left-foot", H_lf_ref);
  trajRF_ = std::make_shared<tsid::trajectories::TrajectorySE3Constant>("traj-right-foot", H_rf_ref);
}

Eigen::MatrixXd TSIDController::getContactPoints() const
{
  // Contact points matrix (matching reference biped.py)
  Eigen::Matrix<double, 3, 4> contact_points;
  contact_points.row(0) << -config_.lxn, -config_.lxn, config_.lxp, config_.lxp;
  contact_points.row(1) << -config_.lyn, config_.lyp, -config_.lyn, config_.lyp;
  contact_points.row(2) << -config_.lz, -config_.lz, -config_.lz, -config_.lz;
  
  return contact_points;
}

void TSIDController::updateRobotState(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
  q_ = q;
  v_ = v;
  
  // Update formulation with current state
  formulation_->computeProblemData(0.0, q_, v_);
}

void TSIDController::setCoMReference(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc)
{
  // Update CoM trajectory reference
  tsid::trajectories::TrajectorySample sample(3);
  sample.value(pos);
  sample.derivative(vel);
  sample.second_derivative(acc);
  
  comTask_->setReference(sample);
}

void TSIDController::setFootReference(const std::string& foot_name, const Eigen::Matrix4d& H_ref)
{
  // Convert Matrix4d to SE3
  pinocchio::SE3 se3_ref;
  se3_ref.translation() = H_ref.block<3,1>(0,3);
  se3_ref.rotation() = H_ref.block<3,3>(0,0);
  
  tsid::trajectories::TrajectorySample sample(12);
  tsid::math::SE3ToVector(se3_ref, sample.value());
  sample.derivative().setZero();
  sample.second_derivative().setZero();
  
  if (foot_name == config_.lf_frame_name || foot_name == "left") {
    leftFootTask_->setReference(sample);
  } else if (foot_name == config_.rf_frame_name || foot_name == "right") {
    rightFootTask_->setReference(sample);
  }
}

void TSIDController::setPostureReference(const Eigen::VectorXd& q_ref)
{
  if (q_ref.size() == robot_->nv() - 6) {  // Exclude free-flyer
    q_ref_ = q_ref;
    
    tsid::trajectories::TrajectorySample sample(q_ref.size());
    sample.value(q_ref);
    sample.derivative().setZero();
    sample.second_derivative().setZero();
    
    postureTask_->setReference(sample);
  }
}

void TSIDController::addFootContact(const std::string& foot_name)
{
  pinocchio::Data& data = formulation_->data();
  
  if (foot_name == config_.lf_frame_name || foot_name == "left") {
    if (!contact_LF_active_) {
      pinocchio::SE3 H_lf_ref = robot_->framePosition(data, LF_frame_id_);
      contactLF_->setReference(H_lf_ref);
      
      if (config_.w_contact >= 0.0) {
        formulation_->addRigidContact(contactLF_, config_.w_forceRef, config_.w_contact, 1);
      } else {
        formulation_->addRigidContact(contactLF_, config_.w_forceRef);
      }
      contact_LF_active_ = true;
    }
  } else if (foot_name == config_.rf_frame_name || foot_name == "right") {
    if (!contact_RF_active_) {
      pinocchio::SE3 H_rf_ref = robot_->framePosition(data, RF_frame_id_);
      contactRF_->setReference(H_rf_ref);
      
      if (config_.w_contact >= 0.0) {
        formulation_->addRigidContact(contactRF_, config_.w_forceRef, config_.w_contact, 1);
      } else {
        formulation_->addRigidContact(contactRF_, config_.w_forceRef);
      }
      contact_RF_active_ = true;
    }
  }
}

void TSIDController::removeFootContact(const std::string& foot_name)
{
  pinocchio::Data& data = formulation_->data();
  
  if (foot_name == config_.lf_frame_name || foot_name == "left") {
    if (contact_LF_active_) {
      pinocchio::SE3 H_lf_ref = robot_->framePosition(data, LF_frame_id_);
      trajLF_->setReference(H_lf_ref);
      
      tsid::trajectories::TrajectorySample sample = trajLF_->computeNext();
      leftFootTask_->setReference(sample);
      
      formulation_->removeRigidContact(contactLF_->name());
      contact_LF_active_ = false;
    }
  } else if (foot_name == config_.rf_frame_name || foot_name == "right") {
    if (contact_RF_active_) {
      pinocchio::SE3 H_rf_ref = robot_->framePosition(data, RF_frame_id_);
      trajRF_->setReference(H_rf_ref);
      
      tsid::trajectories::TrajectorySample sample = trajRF_->computeNext();
      rightFootTask_->setReference(sample);
      
      formulation_->removeRigidContact(contactRF_->name());
      contact_RF_active_ = false;
    }
  }
}

bool TSIDController::isFootInContact(const std::string& foot_name) const
{
  if (foot_name == config_.lf_frame_name || foot_name == "left") {
    return contact_LF_active_;
  } else if (foot_name == config_.rf_frame_name || foot_name == "right") {
    return contact_RF_active_;
  }
  return false;
}

bool TSIDController::computeControlTorques(Eigen::VectorXd& tau_out)
{
  try {
    // Update task references
    updateTaskReferences();
    
    // Compute HQP problem
    auto HQPData = formulation_->computeProblemData(0.0, q_, v_);
    
    // Solve QP
    auto sol = solver_->solve(HQPData);
    
    if (sol.status != tsid::solvers::HQP_STATUS_OPTIMAL) {
      std::cerr << "QP solver failed with status: " << sol.status << std::endl;
      return false;
    }
    
    // Extract joint torques (excluding free-flyer)
    tau_out = formulation_->getActuatorForces(sol);
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Error in computeControlTorques: " << e.what() << std::endl;
    return false;
  }
}

void TSIDController::updateTaskReferences()
{
  // Set angular momentum reference to zero
  tsid::trajectories::TrajectorySample sampleAM(3);
  sampleAM.value().setZero();
  sampleAM.derivative().setZero();
  sampleAM.second_derivative().setZero();
  amTask_->setReference(sampleAM);
}

Eigen::Vector3d TSIDController::getCurrentCoM() const
{
  pinocchio::Data data_copy = formulation_->data();
  return robot_->com(data_copy);
}

Eigen::Matrix4d TSIDController::getFootPosition(const std::string& foot_name) const
{
  pinocchio::Data data_copy = formulation_->data();
  pinocchio::SE3 se3_pos;
  
  if (foot_name == config_.lf_frame_name || foot_name == "left") {
    se3_pos = robot_->framePosition(data_copy, LF_frame_id_);
  } else if (foot_name == config_.rf_frame_name || foot_name == "right") {
    se3_pos = robot_->framePosition(data_copy, RF_frame_id_);
  } else {
    return Eigen::Matrix4d::Identity();
  }
  
  Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
  H.block<3,3>(0,0) = se3_pos.rotation();
  H.block<3,1>(0,3) = se3_pos.translation();
  
  return H;
}

} // namespace robosoccer_control 