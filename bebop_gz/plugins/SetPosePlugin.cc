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
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm, EventManager & /*eventMgr*/) override
  {
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "SetPosePlugin debe estar asociado a un modelo válido." << std::endl;
      return;
    }

    if (sdf->HasElement("topic"))
    {
      this->topic = sdf->Get<std::string>("topic");
    }
    else
    {
      gzerr << "SetPosePlugin requiere el elemento <topic>." << std::endl;
      return;
    }

    this->node = std::make_unique<gz::transport::Node>();

    if (!this->node->Subscribe(this->topic, &SetPosePlugin::OnPoseMsg, this))
    {
      gzerr << "Error al suscribirse al tópico [" << this->topic << "]." << std::endl;
      return;
    }

    gzmsg << "SetPosePlugin suscrito al tópico [" << this->topic << "]" << std::endl;
  }

  void PreUpdate(const UpdateInfo & /*info*/, EntityComponentManager &ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->newPose)
    {
      this->model.SetWorldPoseCmd(ecm, *this->newPose);
      gzmsg << "Posición actualizada a: " << this->newPose->Pos() << std::endl;
      this->newPose.reset();
    }
  }

private:
  void OnPoseMsg(const gz::msgs::Pose &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->newPose = gz::math::Pose3d(
      gz::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
      gz::math::Quaterniond(msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z())
    );
    gzmsg << "Mensaje recibido: posición (" << msg.position().x() << ", "
          << msg.position().y() << ", " << msg.position().z() << ")" << std::endl;
  }

  Model model;
  std::string topic;
  std::unique_ptr<gz::transport::Node> node;
  std::optional<gz::math::Pose3d> newPose;
  std::mutex mutex;
};

GZ_ADD_PLUGIN(SetPosePlugin, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(SetPosePlugin, "gz::sim::systems::SetPosePlugin")

} // namespace systems
} // namespace sim
} // namespace gz
