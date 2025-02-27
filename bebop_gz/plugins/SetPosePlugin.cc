// Include necessary Gazebo and Ignition libraries
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

// Define the SetPosePlugin class inheriting from System, ISystemConfigure, and ISystemPreUpdate
class SetPosePlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  // Configure method to initialize the plugin
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm, EventManager & /*eventMgr*/) override
  {
    // Initialize the model with the given entity
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "SetPosePlugin debe estar asociado a un modelo válido." << std::endl;
      return;
    }

    // Check if the topic element is provided in the SDF
    if (sdf->HasElement("topic"))
    {
      this->topic = sdf->Get<std::string>("topic");
    }
    else
    {
      gzerr << "SetPosePlugin requiere el elemento <topic>." << std::endl;
      return;
    }

    // Create a new transport node
    this->node = std::make_unique<gz::transport::Node>();

    // Subscribe to the topic to receive pose messages
    if (!this->node->Subscribe(this->topic, &SetPosePlugin::OnPoseMsg, this))
    {
      gzerr << "Error al suscribirse al tópico [" << this->topic << "]." << std::endl;
      return;
    }

    gzmsg << "SetPosePlugin suscrito al tópico [" << this->topic << "]" << std::endl;
  }

  // PreUpdate method to update the model's pose before each simulation step
  void PreUpdate(const UpdateInfo & /*info*/, EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->newPose)
    {
      // Set the new pose for the model
      this->model.SetWorldPoseCmd(ecm, *this->newPose);
      gzmsg << "Posición actualizada a: " << this->newPose->Pos() << std::endl;
      this->newPose.reset();
    }
  }

private:
  // Callback method to handle incoming pose messages
  void OnPoseMsg(const gz::msgs::Pose &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    // Convert the message to a Pose3d object
    this->newPose = gz::math::Pose3d(
      gz::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
      gz::math::Quaterniond(msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z())
    );
    gzmsg << "Mensaje recibido: posición (" << msg.position().x() << ", "
          << msg.position().y() << ", " << msg.position().z() << ")" << std::endl;
  }

  // Member variables
  Model model; // The model associated with this plugin
  std::string topic; // The topic to subscribe to for pose messages
  std::unique_ptr<gz::transport::Node> node; // The transport node for communication
  std::optional<gz::math::Pose3d> newPose; // The new pose to set for the model
  std::mutex mutex; // Mutex to protect access to newPose
};

// Register the plugin with Gazebo
GZ_ADD_PLUGIN(SetPosePlugin, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(SetPosePlugin, "gz::sim::systems::SetPosePlugin")

} // namespace systems
} // namespace sim
} // namespace gz
