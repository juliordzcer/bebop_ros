#include "RobotPosePublisher.hh"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>

#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdf/Joint.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::RobotPosePublisherPrivate
{
  public: void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);

  public: void FillPoses(const EntityComponentManager &_ecm,
      std::vector<std::pair<Entity, math::Pose3d>> &_poses,
      bool _static);

  public: void PublishPoses(
      std::vector<std::pair<Entity, math::Pose3d>> &_poses,
      const msgs::Time &_stampMsg,
      transport::Node::Publisher &_publisher);

  public: transport::Node node;
  public: transport::Node::Publisher posePub;
  public: bool publishModelPose = true;
  public: double updateFrequency = -1;
  public: std::chrono::steady_clock::duration lastPosePubTime{0};
  public: std::chrono::steady_clock::duration updatePeriod{0};
  public: std::unordered_map<Entity, std::pair<std::string, std::string>> entitiesToPublish;
  public: std::vector<std::pair<Entity, math::Pose3d>> poses;
  public: msgs::Pose poseMsg;
  public: msgs::Pose_V poseVMsg;
  public: bool usePoseV = false;
  public: bool initialized{false};

  // Add the model variable
  public: Model model{kNullEntity};

  // Add a variable to store the custom topic
  public: std::string customTopic;
};

//////////////////////////////////////////////////
RobotPosePublisher::RobotPosePublisher()
  : dataPtr(std::make_unique<RobotPosePublisherPrivate>())
{
  // Initialize the model variable
  this->dataPtr->model = Model();
}

//////////////////////////////////////////////////
void RobotPosePublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Initialize the model with the entity
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "RobotPosePublisher plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // Parse optional params
  this->dataPtr->usePoseV =
    _sdf->Get<bool>("use_pose_vector_msg", this->dataPtr->usePoseV).first;

  double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;

  if (updateFrequency > 0)
  {
    std::chrono::duration<double> period{1 / updateFrequency};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  // Read the custom topic from the SDF
  if (_sdf->HasElement("topic"))
  {
    this->dataPtr->customTopic = _sdf->Get<std::string>("topic");
  }
  else
  {
    // Default topic if not specified
    this->dataPtr->customTopic = topicFromScopedName(_entity, _ecm, true) + "/pose";
    if (this->dataPtr->customTopic.empty())
    {
      this->dataPtr->customTopic = "/pose";
      gzerr << "Empty pose topic generated for pose_publisher system. "
             << "Setting to " << this->dataPtr->customTopic << std::endl;
    }
  }

  // Create publisher
  if (this->dataPtr->usePoseV)
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<msgs::Pose_V>(this->dataPtr->customTopic);
  }
  else
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<msgs::Pose>(this->dataPtr->customTopic);
  }
}

//////////////////////////////////////////////////
void RobotPosePublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("RobotPosePublisher::PostUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (_info.paused)
    return;

  bool publish = true;
  auto diff = _info.simTime - this->dataPtr->lastPosePubTime;
  if ((diff > std::chrono::steady_clock::duration::zero()) &&
      (diff < this->dataPtr->updatePeriod))
  {
    publish = false;
  }

  if (!publish)
    return;

  if (!this->dataPtr->initialized)
  {
    this->dataPtr->InitializeEntitiesToPublish(_ecm);
    this->dataPtr->initialized = true;
  }

  this->dataPtr->poses.clear();
  this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, true);
  this->dataPtr->PublishPoses(this->dataPtr->poses,
      convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);
  this->dataPtr->lastPosePubTime = _info.simTime;
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::InitializeEntitiesToPublish(
    const EntityComponentManager &_ecm)
{
  std::stack<Entity> toCheck;
  toCheck.push(this->model.Entity());
  std::vector<Entity> visited;
  while (!toCheck.empty())
  {
    Entity entity = toCheck.top();
    toCheck.pop();
    visited.push_back(entity);

    auto isModel = _ecm.Component<components::Model>(entity);
    auto parent = _ecm.Component<components::ParentEntity>(entity);

    if (isModel)
    {
      std::string frame;
      std::string childFrame;
      auto entityName = _ecm.Component<components::Name>(entity);
      if (!entityName)
        continue;
      childFrame =
        removeParentScope(scopedName(entity, _ecm, "::", false), "::");

      if (parent)
      {
        auto parentName = _ecm.Component<components::Name>(parent->Data());
        if (parentName)
        {
          frame = removeParentScope(
              scopedName(parent->Data(), _ecm, "::", false), "::");
        }
      }
      this->entitiesToPublish[entity] = std::make_pair(frame, childFrame);
    }

    // Recursively check child entities
    auto childEntities =
        _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

    for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend();
         ++childIt)
    {
      auto it = std::find(visited.begin(), visited.end(), *childIt);
      if (it == visited.end())
      {
        toCheck.push(*childIt);
      }
    }
  }

  this->poses.reserve(this->entitiesToPublish.size());
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::FillPoses(const EntityComponentManager &_ecm,
    std::vector<std::pair<Entity, math::Pose3d>> &_poses, bool _static)
{
  GZ_PROFILE("RobotPosePublisher::FillPose");

  for (const auto &entity : this->entitiesToPublish)
  {
    auto pose = _ecm.Component<components::Pose>(entity.first);
    if (!pose)
      continue;

    _poses.emplace_back(entity.first, pose->Data());
  }
}

//////////////////////////////////////////////////
void RobotPosePublisherPrivate::PublishPoses(
    std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    const msgs::Time &_stampMsg,
    transport::Node::Publisher &_publisher)
{
  GZ_PROFILE("RobotPosePublisher::PublishPoses");

  msgs::Pose *msg = nullptr;
  if (this->usePoseV)
    this->poseVMsg.Clear();

  for (const auto &[entity, pose] : _poses)
  {
    auto entityIt = this->entitiesToPublish.find(entity);
    if (entityIt == this->entitiesToPublish.end())
      continue;

    if (this->usePoseV)
    {
      msg = this->poseVMsg.add_pose();
    }
    else
    {
      this->poseMsg.Clear();
      msg = &this->poseMsg;
    }

    auto header = msg->mutable_header();
    header->mutable_stamp()->CopyFrom(_stampMsg);
    const std::string &frameId = entityIt->second.first;
    const std::string &childFrameId = entityIt->second.second;
    const math::Pose3d &transform = pose;
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    auto childFrame = header->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(childFrameId);

    msg->set_name(childFrameId);
    msgs::Set(msg, transform);

    if (!this->usePoseV)
      _publisher.Publish(this->poseMsg);
  }

  if (this->usePoseV)
    _publisher.Publish(this->poseVMsg);
}

GZ_ADD_PLUGIN(RobotPosePublisher,
                    System,
                    RobotPosePublisher::ISystemConfigure,
                    RobotPosePublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(RobotPosePublisher,
                          "gz::sim::systems::RobotPosePublisher")