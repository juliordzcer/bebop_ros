#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/systems/CameraPublisher.hh>
#include <gz/plugin/Register.hh>

namespace gz {
namespace sim {
namespace systems {

class CameraPublisherPlugin : public System, public ISystemConfigure
{
public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf) override
  {
    // Inicializar el sistema de la cámara
    this->cameraEntity = entity;
    
    // Crear el publicador de la cámara
    this->cameraPublisher = std::make_unique<CameraPublisher>();

    // Aquí puedes ajustar cualquier parámetro de la cámara si es necesario
    this->cameraPublisher->Configure(entity);
  }

  // Función para actualizar el publicador de la cámara
  void Update(const UpdateInfo &info) override
  {
    if (this->cameraEntity == kNullEntity) return;
    
    // Publicar las imágenes de la cámara
    this->cameraPublisher->OnUpdate(info);
  }

private:
  Entity cameraEntity{kNullEntity};
  std::unique_ptr<CameraPublisher> cameraPublisher;
};

GZ_REGISTER_SYSTEM_PLUGIN(CameraPublisherPlugin)

}  // namespace systems
}  // namespace sim
}  // namespace gz
