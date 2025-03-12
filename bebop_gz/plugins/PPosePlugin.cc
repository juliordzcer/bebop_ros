#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/common/Console.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>  // Asegúrate de incluir esta línea

namespace gz
{
namespace sim
{
namespace systems
{

class PosePlugin : public System, public ISystemPostUpdate
{
public:
  void Configure(const Entity &entity,
                const std::shared_ptr<const sdf::Element> &sdf,
                EntityComponentManager &ecm,
                EventManager & /*eventMgr*/)
  {
    this->model = Model(entity);
    if (!this->model.Valid(ecm)) {
      gzerr << "Plugin debe asociarse a un modelo válido.\n";
      return;
    }

    if (!sdf->HasElement("topic")) {
      gzerr << "Falta elemento <topic> en el SDF.\n";
      return;
    }
    this->topic = sdf->Get<std::string>("topic");

    // Crear un nodo de transporte
    this->node = std::make_unique<gz::transport::Node>();

    // Anunciar el tópico para publicar la pose
    this->publisher = this->node->Advertise<gz::msgs::Pose>(this->topic);
    if (!this->publisher.Valid()) {  // Usar Valid() en lugar de comparación con nullptr
      gzerr << "Error al anunciar el tópico [" << this->topic << "].\n";
      return;
    }

    gzmsg << "Plugin publicando en el tópico [" << this->topic << "]\n";
  }

  void PostUpdate(const UpdateInfo &, const EntityComponentManager &ecm) override
  {
    // Obtener el componente de Pose del modelo utilizando el EntityComponentManager
    auto pose_component = ecm.Component<gz::sim::components::Pose>(this->model.Entity());
    if (!pose_component) {
      gzwarn << "No se pudo obtener la pose del modelo.\n";
      return;
    }

    // Obtener la pose del componente
    const auto &pose = pose_component->Data();

    // Crear un mensaje de pose
    gz::msgs::Pose msg;
    msg.mutable_position()->set_x(pose.Pos().X());
    msg.mutable_position()->set_y(pose.Pos().Y());
    msg.mutable_position()->set_z(pose.Pos().Z());
    msg.mutable_orientation()->set_w(pose.Rot().W());
    msg.mutable_orientation()->set_x(pose.Rot().X());
    msg.mutable_orientation()->set_y(pose.Rot().Y());
    msg.mutable_orientation()->set_z(pose.Rot().Z());

    // Publicar el mensaje
    if (!this->publisher.Publish(msg)) {
      gzerr << "Error al publicar la pose en el tópico [" << this->topic << "].\n";
    }
  }

private:
  Model model; // El modelo asociado al plugin
  std::string topic; // El tópico para publicar la pose
  std::unique_ptr<gz::transport::Node> node; // Nodo de transporte
  gz::transport::Node::Publisher publisher; // Publicador
};

GZ_ADD_PLUGIN(PosePlugin, System, ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(PosePlugin, "gz::sim::systems::PosePlugin")

} // namespace systems
} // namespace sim
} // namespace gz 
