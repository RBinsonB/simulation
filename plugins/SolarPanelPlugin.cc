#include <ignition/msgs/double.pb.h>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "SolarPanelPlugin.hh"

using namespace simulation;

ignition::common::EventT<void(const ignition::rendering::ScenePtr &)>
SolarPanelPlugin::sceneEvent;

void SolarPanelPlugin::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  // Store the pointer to the model the solar panel is under
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Solar panel plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->model = model;
  this->modelName = model.Name(_ecm);

  // Load params
  if (_sdf->HasElement("link_name"))
  {
    this->linkName = _sdf->Get<std::string>("link_name");
    this->topicName = this->modelName + "/" + this->linkName + "/solar_panel_output";
    // Advertise topic where data will be published
    this->pub = this->node.Advertise<ignition::msgs::Float>(this->topicName);
  }
  else
  {
    ignerr << "Solar panel plugin should have a <link_name> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    ignerr << "Solar panel plugin should have a <nominal_power> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->sceneChangeConnection = this->sceneEvent.Connect(std::bind(&SolarPanelPlugin::SetScene, this, std::placeholders::_1));
}

bool SolarPanelPlugin::FindScene()
{
  auto loadedEngNames = ignition::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    ignerr << "No rendering engine is loaded yet" << std::endl;
    return false;
  }
 
  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    ignerr << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = ignition::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return false;
  }
 
  if (engine->SceneCount() == 0)
  {
    igndbg << "No scene has been created yet" << std::endl;
    return false;
  }
 
  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return false;
  }
 
  if (engine->SceneCount() > 1)
  {
    igndbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }
 
  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return false;
  }
 
  this->scene = scenePtr;
  return true;
}


std::vector<std::string> SolarPanelPlugin::GetVisualChildren(
      const ignition::gazebo::EntityComponentManager &_ecm) 
  {
  // Build the prefix for the scoped name
  std::string scopedPrefix = this->modelName + "::" + this->linkName + "::";

  // Find all visual entities that are children of this link
  std::vector<std::string> scopedVisualChildren;
  _ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Name, ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::Visual *,
        const ignition::gazebo::components::Name *_name,
        const ignition::gazebo::components::ParentEntity *_parent) -> bool
    {
      if (_parent->Data() == linkEntity)
      {
        std::string scopedName = scopedPrefix + _name->Data();
        scopedVisualChildren.push_back(scopedName);
      }
      return true;
    });

  return scopedVisualChildren;
}

void SolarPanelPlugin::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->scene != _scene)
  {
    this->scene = _scene;
  }
}

void SolarPanelPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    if (!this->scene)
    {
      if (!FindScene())
      {
        ignerr << "Rendering scene not available yet" << std::endl;
        return;
      }
    }

    std::shared_ptr<ignition::rendering::RayQuery> rayQuery = this->scene->CreateRayQuery();
    if (!rayQuery)
    {
      ignerr << "Failed to create RayQuery" << std::endl;
      return;
    }

    if (this->scopedVisualChildren.empty())
    {
      this->scopedVisualChildren = GetVisualChildren(_ecm);
    }

    // Get sun entity
    ignition::gazebo::Entity sunEntity;
    ignition::math::Pose3d sunPose;
    _ecm.Each<ignition::gazebo::components::Name, ignition::gazebo::components::Pose>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::Pose *_pose)->bool
      {
        if (_name->Data() == "sun")
        {
          sunEntity = _entity;
          sunPose = _pose->Data();
          return false;  // Stop iteration
        }
        return true;
      });

    if (sunEntity == ignition::gazebo::v6::kNullEntity)
    {
      ignerr << "Sun entity not found" << std::endl;
      return;
    }

    // Check if sun entity is of type "light" and has a "direction" element
    const auto *lightComp = _ecm.Component<ignition::gazebo::components::Light>(sunEntity);
    ignition::math::Vector3d direction;
    if (lightComp)
    {
      const auto &light = lightComp->Data();
      direction = light.Direction();
    }
    else
    {
      ignerr << "Sun entity is not a light!" << std::endl;
      return;
    }

    // Rotate sun direction according to sun pose orientation
    ignition::math::Vector3d sunDirection = sunPose.Rot().RotateVector(direction);

    if (this->linkEntity == ignition::gazebo::v6::kNullEntity)
    {
      this->linkEntity =
          this->model.LinkByName(_ecm, this->linkName);
    }

    ignition::math::Pose3d linkPose = ignition::gazebo::worldPose(linkEntity, _ecm);
    // Perform ray cast from link to sun
    ignition::math::Vector3d start = linkPose.Pos();
    ignition::math::Vector3d end = sunPose.Pos();
    
    rayQuery->SetOrigin(end);
    rayQuery->SetDirection(start - end);

    // Check if ray intersects with any obstacles
    auto result = rayQuery->ClosestPoint();
    bool isValid = result;

    std::string objectName = "unknown";
    bool isInLOS = false;
    ignition::rendering::NodePtr node = this->scene->NodeById(result.objectId);
    if (node)
    {
      objectName = node->Name();
      if (isValid)
      {
        isInLOS = (any_of(this->scopedVisualChildren.begin(), this->scopedVisualChildren.end(), [&](const std::string& elem) { return elem == objectName; }));
      }
    }

    // Compute current power output
    float currentPower = 0.0F;

    // Compute the angle between the link normal and sun direction
    // Calculate dot product
    ignition::math::Vector3d linkNormal = linkPose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
    float dotProduct = linkNormal.Dot(-sunDirection); // Negate sunDirection because it points from sun to scene

    // Solar panel will not receive any power if angle is more than 90deg (sun rays hitting horizontally or below)
    if ((dotProduct > 0.0F) && (isInLOS))
    {
      // Calculate magnitudes
      float magnitude1 = linkNormal.Length();
      float magnitude2 = sunDirection.Length();
      // Calculate cosine of the angle
      float cosAngle;
      if (ignition::math::equal(magnitude1, 0.0F) || 
          ignition::math::equal(magnitude2, 0.0F)) 
      {
        cosAngle = 1.0F;
      }
      else
      {
        cosAngle = dotProduct / (magnitude1 * magnitude2);
      }
      
      // Compute the effective area factor (cosine of the angle)
      float effectiveAreaFactor = std::max(0.0F, cosAngle);

      // Compute current power based on the angle
      currentPower = this->nominalPower * effectiveAreaFactor;
    }

    // Publish result
    ignition::msgs::Float msg;
    msg.set_data(currentPower);
    this->pub.Publish(msg);

    igndbg << "Solar Panel Plugin:: Current power output: " << currentPower << " watts" << std::endl;
    igndbg << "Solar Panel Plugin:: In line of sight: " << (isInLOS ? "Yes" : "No") << std::endl;
  }
}

IGNITION_ADD_PLUGIN(SolarPanelPlugin, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(SolarPanelPlugin, "simulation::SolarPanelPlugin")
