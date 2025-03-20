#include "VelocityControllerPID.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace multicopter_control
{
  //////////////////////////////////////////////////
  std::unique_ptr<VelocityControllerPID> VelocityControllerPID::MakeController(
      const VelocityControllerPIDParameters &_controllerParams,
      const VehicleParameters &_vehicleParams)
  {
    std::unique_ptr<VelocityControllerPID> controller(new VelocityControllerPID());
    controller->controllerParameters = _controllerParams;
    controller->vehicleParameters = _vehicleParams;
    if (controller->InitializeParameters())
    {
      return controller;
    }
    else
    {
      return nullptr;
    }
  }

  //////////////////////////////////////////////////
  bool VelocityControllerPID::InitializeParameters()
  {
    auto allocationMatrix = calculateAllocationMatrix(this->vehicleParameters.rotorConfiguration);
    if (!allocationMatrix.has_value())
    {
      gzerr << "Failed to calculate allocation matrix.\n";
      return false;
    }

    Eigen::Matrix4d moi;
    moi.setZero();
    moi.block<3, 3>(0, 0) = this->vehicleParameters.inertia;
    moi(3, 3) = 1;

    this->angularAccToRotorVelocities.resize(
        this->vehicleParameters.rotorConfiguration.size(), 4);
    const auto &aMat = *allocationMatrix;
    this->angularAccToRotorVelocities =
        aMat.transpose() * (aMat * aMat.transpose()).inverse() * moi;

    return true;
  }

  //////////////////////////////////////////////////
  void VelocityControllerPID::CalculateRotorVelocities(
      const FrameData &_frameData, const EigenTwist &_cmdVel,
      Eigen::VectorXd &_rotorVelocities) const
  {
    Eigen::Vector3d thrustAndAttitude = ComputeThrustandAttitude(_frameData, _cmdVel);

    double thrust = thrustAndAttitude(0);
    double phid = thrustAndAttitude(1);   // Desired roll
    double thetad = thrustAndAttitude(2); // Desired pitch
    double psid = _cmdVel.angular.z();    // Desired yaw

    // Current orientation (rotation matrix)
    Eigen::Matrix3d R = _frameData.pose.linear().matrix().block<3, 3>(0, 0);

    // Convert rotation matrix to Euler angles (roll, pitch, yaw)
    Eigen::Vector3d eta = R.eulerAngles(0, 1, 2);

    // Attitude error
    Eigen::Vector3d etad(phid, thetad, psid);
    Eigen::Vector3d e_eta = eta - etad;

    // Integral and derivative terms for attitude control
    Eigen::Vector3d ie_eta = this->ie_eta + e_eta * 0.01; // dt = 0.01
    Eigen::Vector3d ep_eta = (e_eta - this->e_eta_prev) / 0.01;

    // Attitude control law
    Eigen::Vector3d TauRPY =
        this->controllerParameters.Kp_att.cwiseProduct(e_eta) +
        this->controllerParameters.Ki_att.cwiseProduct(ie_eta) +
        this->controllerParameters.Kd_att.cwiseProduct(ep_eta);

    // Map torques and thrust to rotor velocities
    Eigen::Vector4d angularAccelerationThrust;
    angularAccelerationThrust << TauRPY, thrust;

    _rotorVelocities =
        this->angularAccToRotorVelocities * angularAccelerationThrust;

    // Ensure rotor velocities are non-negative
    _rotorVelocities = _rotorVelocities.cwiseMax(Eigen::VectorXd::Zero(_rotorVelocities.rows()));
    _rotorVelocities = _rotorVelocities.cwiseSqrt();
  }

  //////////////////////////////////////////////////
  Eigen::Vector3d VelocityControllerPID::ComputeThrustandAttitude(
      const FrameData &_frameData, const EigenTwist &_cmdVel) const
  {
    double dt = 0.01;

    // Desired velocity in body frame
    Eigen::Vector3d xipd = _frameData.pose.linear() * _cmdVel.linear;
    Eigen::Vector3d xip = _frameData.linearVelocityWorld;

    // Current orientation (rotation matrix)
    Eigen::Matrix3d R = _frameData.pose.linear().matrix().block<3, 3>(0, 0);

    // Convert rotation matrix to Euler angles (roll, pitch, yaw)
    Eigen::Vector3d euler = R.eulerAngles(0, 1, 2);
    double yaw = euler.z();

    // Transform velocity to body frame
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);
    double state_body_vx = xip.x() * cosyaw + xip.y() * sinyaw;
    double state_body_vy = -xip.x() * sinyaw + xip.y() * cosyaw;

    Eigen::Vector3d xipm(state_body_vx, state_body_vy, xipd.z());

    // Position error
    Eigen::Vector3d e_xi = xipm - xipd;

    // Integral and derivative terms for position control
    Eigen::Vector3d ie_xi = this->ie_xi + e_xi * dt;
    Eigen::Vector3d ep_xi = (e_xi - this->e_xi_prev) / dt;

    // Position control law
    Eigen::Vector3d RPT =
        this->controllerParameters.Kp_pos.cwiseProduct(e_xi) +
        this->controllerParameters.Ki_pos.cwiseProduct(ie_xi) +
        this->controllerParameters.Kd_pos.cwiseProduct(ep_xi);

    double thrust = RPT.z();
    double roll = RPT.x();
    double pitch = RPT.y();

    // Return thrust and attitude angles
    Eigen::Vector3d thrustAndAttitude(thrust, roll, pitch);
    return thrustAndAttitude;
  }
}  // namespace multicopter_control
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz