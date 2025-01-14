// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cartesian_impedance_control/cartesian_impedance_controller.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <Eigen/Eigen>
#include <chrono>

using namespace std::chrono;

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace cartesian_impedance_control {

void CartesianImpedanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  q_d_ = 0.005 * q_desired_ + 0.995 * q_d_;
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
    
}


void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}


CallbackReturn CartesianImpedanceController::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                                               
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&CartesianImpedanceController::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }


  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  //Initialize tau_gravity_publisher
  tau_gravity_publisher_ = this->get_node()->create_publisher<messages_fr3::msg::TauGravity>("tau_gravity", 10);
  // Create the subscriber in the on_activate method
  desired_pose_sub = get_node()->create_subscription<geometry_msgs::msg::Pose>(
        "cartesian_impedance_controller/reference_pose", 
        10,  // Queue size
        std::bind(&CartesianImpedanceController::reference_pose_callback, this, std::placeholders::_1)
    );

  //create publisher for Jacobian
  jacobian_publisher_ = this->get_node()->create_publisher<messages_fr3::msg::Jacobian>("jacobian", 10);

  desired_joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        "/franka_robot_state_broadcaster/desired_joint_state", 10,
        std::bind(&CartesianImpedanceController::desiredJointStateCallback, this, std::placeholders::_1)
        );
  //create publisher for gravity force vector
  gravity_force_vector_publisher_ = this->get_node()->create_publisher<messages_fr3::msg::GravityForceVector>("gravity_force_vector", 10);
  //create publisher for acceleration
  acceleration_publisher_ = this->get_node()->create_publisher<messages_fr3::msg::Acceleration>("acceleration", 10);
  //create publisher for transformation matrix
  transformation_publisher_ = this->get_node()->create_publisher<messages_fr3::msg::TransformationMatrix>("transformation_matrix", 10);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  std::array<double, 7> gravity_force_vector_array = franka_robot_model_->getGravityForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_force_vector(gravity_force_vector_array.data());
  tau_gravity = gravity_force_vector;
  return CallbackReturn::SUCCESS;
  
}


controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

void CartesianImpedanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  // Existing handling of external forces
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}


void CartesianImpedanceController::desiredJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Handle the received desired joint positions
        if (msg->position.size() != 7) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 7 joint positions, but got %lu", msg->position.size());
            return;
        }
        for (size_t i = 0; i < 7; ++i) {
            q_desired_(i) = msg->position[i];
        }
    }



void CartesianImpedanceController::reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Handle the incoming pose message
    std::cout << "received reference posistion as " <<  msg->position.x << ", " << msg->position.y << ", " << msg->position.z << std::endl;
    position_d_target_ << msg->position.x, msg->position.y,msg->position.z;
    orientation_d_target_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    // You can add more processing logic here
}


void CartesianImpedanceController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Check if the size of the effort vector is correct (should match the number of joints, e.g., 7 for Franka)
    if (msg->effort.size() == 7) {
        // Convert std::vector from the message into an Eigen matrix for tau_J
        for (size_t i = 0; i < 7; ++i) {
            tau_J(i) = msg->effort[i];  // Extract the measured joint torques
        }
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "JointState message has incorrect effort size");
    }
}


void CartesianImpedanceController::updateJointStates() {
  dq_prev_ = dq_;
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}


controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  // if (outcounter == 0){
  // std::cout << "Enter 1 if you want to track a desired position or 2 if you want to use free floating with optionally shaped inertia" << std::endl;
  // std::cin >> mode_;
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // std::cout << "Mode selected" << std::endl;
  // while (mode_ != 1 && mode_ != 2){
  //   std::cout << "Invalid mode, try again" << std::endl;
  //   std::cin >> mode_;
  // }
  // }
  jacobian_prev_ = jacobian_endeffector;
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity_force_vector_array = franka_robot_model_->getGravityForceVector();
  //publish gravity force vector
  messages_fr3::msg::GravityForceVector gravity_force_vector_msg;
  gravity_force_vector_msg.gravity = gravity_force_vector_array;
  gravity_force_vector_publisher_->publish(gravity_force_vector_msg);

  jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  jacobian_endeffector = franka_robot_model_->getBodyJacobian(franka::Frame::kFlange);
  
  for (std::size_t i = 0; i < jacobian_endeffector.size(); ++i) {
      dJ[i] = (jacobian_endeffector[i] - jacobian_prev_[i]) / dt;
  }

  
  //calcuate accelerations
  ddq_ = (dq_ - dq_prev_)/dt;  //calculate acceleration
  //publish acceleration
  messages_fr3::msg::Acceleration acceleration_msg;
  for (int i = 0; i < 7; ++i) {
    acceleration_msg.acceleration[i] = ddq_(i);
  }
  acceleration_publisher_->publish(acceleration_msg);

  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  std::array<double, 16> pose_pub = franka_robot_model_->getPoseMatrix(franka::Frame::kFlange);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_force_vector(gravity_force_vector_array.data());
  jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>> (jacobian_array.data());
  
  //Publish Jacobian here
  messages_fr3::msg::Jacobian jacobian_msg;
  jacobian_msg.jacobian = jacobian_endeffector;
  jacobian_msg.d_jacobian = dJ;
  jacobian_publisher_->publish(jacobian_msg);

  //publish transformation matrix
  messages_fr3::msg::TransformationMatrix transformation_msg;
  transformation_msg.transformation_matrix = pose_pub;
  transformation_publisher_->publish(transformation_msg);
  
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  pseudoInverse(jacobian, jacobian_pinv);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  updateJointStates(); 

  
  error.head(3) << position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);
  I_error += Sm * dt * integrator_weights.cwiseProduct(error);
  for (int i = 0; i < 6; i++){
    I_error(i,0) = std::min(std::max(-max_I(i,0),  I_error(i,0)), max_I(i,0)); 
  }

  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
  // Theta = T*Lambda;
  // F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
  //Inertia of the robot
   switch (mode_)
  {
  case 1: 
    Theta = Lambda;
    F_impedance = -1 * (D * (jacobian * dq_) + K * error /*+ I_error*/);
     break; 

  case 2:
    Theta = T*Lambda;
    F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    break;
  
  default:
    break;
  }

  F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M; //Filtering 
  I_F_error += dt * Sf* (F_contact_des - F_ext);
  F_cmd = Sf*(0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des);

  //Calculate friction forces
  N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian);
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_nullspace <<  N * (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  

  calculate_tau_friction();
  
  calculate_tau_gravity(coriolis, gravity_force_vector, jacobian);
  
  //calculate_gravity_torques();
  //calculate_gravity_torques_ana(jacobian);
  tau_gravity_error = tau_gravity - gravity_force_vector;
  tau_impedance = jacobian.transpose() * Sm * (F_impedance /*+ F_repulsion + F_potential*/) + jacobian.transpose() * Sf * F_cmd;
  kp.diagonal() << 300,300,300,300, 100, 100, 80;  // kp gains per joint
  kd.diagonal() << 34.64, 34.64, 34.64, 34.64, 20, 20 , 17.88;  // kd gains per joint  
  auto tau_d_placeholder = kp * (q_d_ - q_) - kd * dq_ + coriolis;  //use this controller for inertia estimation
   //set impedance to zero for gravity torque test, also remove coriolis from control law
  //set friction to zero for gravity torque test
  tau_impedance.setZero();
  tau_friction.setZero();
  coriolis.setZero();
  tau_nullspace.setZero();
  //auto tau_d_placeholder =  tau_impedance + tau_nullspace + tau_friction + coriolis - tau_gravity_error; //add nullspace, friction, gravity and coriolis components to desired torque
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  // Step 5: Implement a logger that logs every 5 seconds
  static steady_clock::time_point last_log_time = steady_clock::now();
  steady_clock::time_point current_time = steady_clock::now();

  // Check if 5 seconds have passed since the last log
  if (duration_cast<seconds>(current_time - last_log_time).count() >= 0.1) {
    last_log_time = current_time;  // Reset the last log time
        
  // Log gravity_force_vector
    RCLCPP_INFO(get_node()->get_logger(), "Gravity Vector (Franka): [%f, %f, %f, %f, %f, %f, %f]",
                    gravity_force_vector[0], gravity_force_vector[1], gravity_force_vector[2],
                    gravity_force_vector[3], gravity_force_vector[4], gravity_force_vector[5], gravity_force_vector[6]);

    RCLCPP_INFO(get_node()->get_logger(), "Modelled Gravity Torque (Observer): [%f, %f, %f, %f, %f, %f, %f]",
                    tau_gravity[0], tau_gravity[1], tau_gravity[2],
                    tau_gravity[3], tau_gravity[4], tau_gravity[5], tau_gravity[6]);

    RCLCPP_INFO(get_node()->get_logger(), "I_tau: [%f, %f, %f, %f, %f, %f, %f]",
                    I_tau[0], I_tau[1], I_tau[2],
                    I_tau[3], I_tau[4], I_tau[5], I_tau[6]);
    RCLCPP_INFO(get_node()->get_logger(), "residual: [%f, %f, %f, %f, %f, %f, %f]",
                    residual[0], residual[1], residual[2],
                    residual[3], residual[4], residual[5], residual[6]);
    RCLCPP_INFO(get_node()->get_logger(), "dq_: [%f, %f, %f, %f, %f, %f, %f]",
                    dq_[0], dq_[1], dq_[2],
                    dq_[3], dq_[4], dq_[5], dq_[6]);


  }


  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  if (outcounter % 1000/update_frequency == 0){
    std::cout << "F_ext_robot [N]" << std::endl;
    std::cout << O_F_ext_hat_K << std::endl;
    std::cout << O_F_ext_hat_K_M << std::endl;
    std::cout << "Lambda  Thetha.inv(): " << std::endl;
    std::cout << Lambda*Theta.inverse() << std::endl;
    std::cout << "tau_d" << std::endl;
    std::cout << tau_d << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << tau_nullspace << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << tau_impedance << std::endl;
    std::cout << "--------" << std::endl;
    std::cout << coriolis << std::endl;
    std::cout << "Inertia scaling [m]: " << std::endl;
    std::cout << T << std::endl;
    std::cout << "F_impedance: " << std::endl;
    std::cout << F_impedance << std::endl;
    std::cout << "q: " << std::endl;
    std::cout << q_ << std::endl;
  }
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace cartesian_impedance_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_control::CartesianImpedanceController,
                       controller_interface::ControllerInterface)