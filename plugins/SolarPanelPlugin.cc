#include <ignition/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "SolarPanelPlugin.hh"

using namespace simulation;

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
    this->topicName = this->modelName + "/solar_panel/" + this->linkName;
    // Advertise topic where data will be published
    this->pub = this->node.Advertise<ignition::msgs::Boolean>(this->topicName);
  }
  else
  {
    ignerr << "Missing <link_name> element in SDF" << std::endl;
  }

  ignerr << "SOLAR PANEL: INIT" << std::endl;
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

  ignerr << "VISUAL CHILDREN OF LINK ARE:" << std::endl;
  for (auto child_name : scopedVisualChildren) {
    ignerr << child_name << std::endl;
  }

  return scopedVisualChildren;
}

void SolarPanelPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  ignerr << "SOLAR PANEL: POSTUPDATE" << std::endl;

  if (!_info.paused)
  {
    this->scene = ignition::rendering::sceneFromFirstRenderEngine();
    if (!this->scene)
    {
      ignerr << "Rendering scene not available yet" << std::endl;
      return;
    }
    // if (nullptr == this->scene)
    // {
    //   this->FindScene();
    // }

    // if (nullptr == this->scene)
    //   return;
    
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

    // Get sun position (assuming there's a sun entity with a pose component)
    ignition::math::Pose3d sunPose;
    _ecm.Each<ignition::gazebo::components::Name, ignition::gazebo::components::Pose>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::Pose *_pose)->bool
      {
        if (_name->Data() == "sun")
        {
          sunPose = _pose->Data();
          ignerr << "SOLAR PANEL: FOUND SUN" << std::endl;
          return false;  // Stop iteration
        }
        return true;
      });

    if (this->linkEntity == ignition::gazebo::v6::kNullEntity)
    {
      this->linkEntity =
          this->model.LinkByName(_ecm, this->linkName);
    }

    ignerr << "SOLAR PANEL: COMPUTE" << std::endl;


    ignition::math::Pose3d linkPose = ignition::gazebo::worldPose(linkEntity, _ecm);
    // Perform ray cast from link to sun
    ignition::math::Vector3d start = linkPose.Pos();
    ignition::math::Vector3d end = sunPose.Pos();
    
    rayQuery->SetOrigin(end);
    rayQuery->SetDirection(start - end);

    ignerr << "SOLAR PANEL: DO RAY QUERY" << std::endl;
    ignerr << "TO " << start.X() << ", " << start.Y() << ", " << start.Z() << std::endl;
    ignerr << "FROM " << end.X() << ", " << end.Y() << ", " << end.Z() << std::endl;

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
    else
    {
      // Node not found for the given ID
    }


    // Publish result
    ignerr << "SOLAR PANEL: " << isValid << std::endl;
    ignerr << "SOLAR PANEL VISIBLE: " << isInLOS << std::endl;
    // ignerr << "SOLAR PANEL INTERSECTION POINT: " << result.point << std::endl;
    // ignerr << "SOLAR PANEL INTERSECTION OBJECT ID: " << result.objectId << std::endl;
    // ignerr << "SOLAR PANEL INTERSECTION OBJECT: " << objectName << std::endl;
    // ignerr << "SOLAR PANEL LINK ID: " << this->linkEntity << std::endl;

    ignition::gazebo::Entity collisionEntity = FindEntityFromRenderingName(_ecm, objectName);
    ignerr << "SOLAR PANEL INTERSECTION ENTITY ID: " << collisionEntity << std::endl;

     // Now you have the latest scene, you can perform operations on it
    // For example, let's print the names of all visual nodes
    // for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    // {
    //   auto node = scene->NodeByIndex(i);
    //   ignerr << "Visual node: " << node->Name() << std::endl;
    // }
    ignition::msgs::Boolean msg;
    msg.set_data(isValid);
    this->pub.Publish(msg);
  }

}

ignition::gazebo::Entity SolarPanelPlugin::FindEntityFromRenderingName(
    const ignition::gazebo::EntityComponentManager &_ecm,
    const std::string &_renderingName)
{
  auto entities = _ecm.EntitiesByComponents(
    ignition::gazebo::components::Name(_renderingName),
    ignition::gazebo::components::Visual());
  
  if (!entities.empty())
  {
    return entities.front();
  }
  
  return ignition::gazebo::kNullEntity;
}

IGNITION_ADD_PLUGIN(SolarPanelPlugin, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(SolarPanelPlugin, "simulation::SolarPanelPlugin")
