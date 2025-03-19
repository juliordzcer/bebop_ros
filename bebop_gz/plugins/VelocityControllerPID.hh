/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

 #ifndef GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_VELOCITYCONTROLLERPID_HH_
 #define GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_VELOCITYCONTROLLERPID_HH_
 
 #include <Eigen/Geometry>
 #include <memory>
 #include "gz/sim/config.hh"
 #include "Common.hh"
 
 namespace gz
 {
 namespace sim
 {
 inline namespace GZ_SIM_VERSION_NAMESPACE {
 namespace systems
 {
 namespace multicopter_control
 {
   /// \brief Data structure containing various parameters for the velocity controller
   struct VelocityControllerPIDParameters
   {
     Eigen::Vector3d Kp_pos;  // Proportional gain for position
     Eigen::Vector3d Ki_pos;  // Integral gain for position
     Eigen::Vector3d Kd_pos;  // Derivative gain for position
     Eigen::Vector3d Kp_att;  // Proportional gain for attitude
     Eigen::Vector3d Ki_att;  // Integral gain for attitude
     Eigen::Vector3d Kd_att;  // Derivative gain for attitude
   };
 
   /// \brief Velocity controller for multicopters using PID control
   class VelocityControllerPID
   {
     public:
       /// \brief Factory function to create a VelocityControllerPID instance
       static std::unique_ptr<VelocityControllerPID> MakeController(
           const VelocityControllerPIDParameters &_controllerParams,
           const VehicleParameters &_vehicleParams);
 
       /// \brief Calculate rotor velocities based on current state and commanded velocity
       void CalculateRotorVelocities(
           const FrameData &_frameData,
           const EigenTwist &_cmdVel,
           Eigen::VectorXd &_rotorVelocities) const;
 
     private:
       /// \brief Private constructor
       VelocityControllerPID() = default;
 
       /// \brief Compute thrust and attitude angles based on current state and commanded velocity
       Eigen::Vector3d ComputeThrustandAttitude(
           const FrameData &_frameData, const EigenTwist &_cmdVel) const;
 
       /// \brief Initialize controller parameters
       bool InitializeParameters();
 
       /// \brief Controller parameters
       VelocityControllerPIDParameters controllerParameters;
 
       /// \brief Vehicle parameters
       VehicleParameters vehicleParameters;
 
       /// \brief Matrix to map angular acceleration and thrust to rotor velocities
       Eigen::MatrixX4d angularAccToRotorVelocities;
 
       /// \brief Integral error for position control
       Eigen::Vector3d ie_xi{Eigen::Vector3d::Zero()};
 
       /// \brief Integral error for attitude control
       Eigen::Vector3d ie_eta{Eigen::Vector3d::Zero()};
 
       /// \brief Previous error for position control (for derivative term)
       Eigen::Vector3d e_xi_prev{Eigen::Vector3d::Zero()};
 
       /// \brief Previous error for attitude control (for derivative term)
       Eigen::Vector3d e_eta_prev{Eigen::Vector3d::Zero()};
   };
 }  // namespace multicopter_control
 }  // namespace systems
 }  // namespace GZ_SIM_VERSION_NAMESPACE
 }  // namespace sim
 }  // namespace gz
 
 #endif  // GZ_SIM_SYSTEMS_MULTICOPTER_CONTROL_VELOCITYCONTROLLERPID_HH_