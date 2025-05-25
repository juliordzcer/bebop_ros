#include "SetPosePlugin.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// Constructor por defecto
SetPosePlugin::SetPosePlugin() = default;

// Destructor por defecto
SetPosePlugin::~SetPosePlugin() = default;

// Método Configure: inicializa el plugin
void SetPosePlugin::Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                              EntityComponentManager &ecm, EventManager & /*eventMgr*/)
{
  // Inicializa el modelo con la entidad proporcionada
  this->model = Model(entity);
  if (!this->model.Valid(ecm))
  {
    gzerr << "SetPosePlugin debe estar asociado a un modelo válido." << std::endl;
    return;
  }

  // Verifica si el elemento <topic> está presente en el SDF
  if (sdf->HasElement("topic"))
  {
    this->topic = sdf->Get<std::string>("topic");
  }
  else
  {
    gzerr << "SetPosePlugin requiere el elemento <topic>." << std::endl;
    return;
  }

  // Crea un nuevo nodo de transporte
  this->node = std::make_unique<gz::transport::Node>();

  // Suscribe el nodo al tópico para recibir mensajes de pose
  if (!this->node->Subscribe(this->topic, &SetPosePlugin::OnPoseMsg, this))
  {
    gzerr << "Error al suscribirse al tópico [" << this->topic << "]." << std::endl;
    return;
  }

  gzmsg << "SetPosePlugin suscrito al tópico [" << this->topic << "]" << std::endl;
}

// Método PreUpdate: actualiza la pose del modelo antes de cada paso de simulación
void SetPosePlugin::PreUpdate(const UpdateInfo & /*info*/, EntityComponentManager &ecm)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->newPose)
  {
    // Establece la nueva pose para el modelo
    this->model.SetWorldPoseCmd(ecm, *this->newPose);
    gzmsg << "Posición actualizada a: " << this->newPose->Pos() << std::endl;
    this->newPose.reset();
  }
}

// Método OnPoseMsg: callback para manejar mensajes de pose recibidos
void SetPosePlugin::OnPoseMsg(const gz::msgs::Pose &msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  // Convierte el mensaje a un objeto Pose3d
  this->newPose = gz::math::Pose3d(
    gz::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
    gz::math::Quaterniond(msg.orientation().w(), msg.orientation().x(), msg.orientation().y(), msg.orientation().z())
  );
  gzmsg << "Mensaje recibido: posición (" << msg.position().x() << ", "
        << msg.position().y() << ", " << msg.position().z() << ")" << std::endl;
}

// Registra el plugin en Gazebo
GZ_ADD_PLUGIN(SetPosePlugin, System, ISystemConfigure, ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(SetPosePlugin, "gz::sim::systems::SetPosePlugin")