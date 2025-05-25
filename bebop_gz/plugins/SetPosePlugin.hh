// Copyright 2025 Julio César Rodríguez
// Licensed under the Apache License, Version 2.0
// https://www.apache.org/licenses/LICENSE-2.0

#ifndef GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_
#define GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/common/Console.hh>
#include <mutex>
#include <optional>

namespace gz
{
namespace sim
{
namespace systems
{

class SetPosePlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  // Constructor and destructor
  SetPosePlugin();
  virtual ~SetPosePlugin();

  // Configure method to initialize the plugin
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm, EventManager &eventMgr) override;

  // PreUpdate method to update the model's pose before each simulation step
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;

private:
  // Callback method to handle incoming pose messages
  void OnPoseMsg(const gz::msgs::Pose &msg);

  // Member variables
  Model model; // The model associated with this plugin
  std::string topic; // The topic to subscribe to for pose messages
  std::unique_ptr<gz::transport::Node> node; // The transport node for communication
  std::optional<gz::math::Pose3d> newPose; // The new pose to set for the model
  std::mutex mutex; // Mutex to protect access to newPose
};

} // namespace systems
} // namespace sim
} // namespace gz

#endif // GZ_SIM_SYSTEMS_SETPOSEPLUGIN_HH_