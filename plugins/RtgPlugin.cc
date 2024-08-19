#include <ignition/msgs/double.pb.h>

#include <string>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "RtgPlugin.hh"

using namespace simulation;

void RtgPlugin::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  // Store the pointer to the model the RTG is under
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Radioisotope Thermal Generator plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->modelName = model.Name(_ecm);

  // Load params
  if (_sdf->HasElement("link_name"))
  {
    this->linkName = _sdf->Get<std::string>("link_name");
    this->topicName = this->modelName + "/" + this->linkName + "/radioisothope_thermal_generator_output";
    // Advertise topic where data will be published
    this->pub = this->node.Advertise<ignition::msgs::Float>(this->topicName);
  }
  else
  {
    ignerr << "Radioisotope Thermal Generator plugin should have a <link_name> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    ignerr << "Radioisotope Thermal Generator plugin should have a <nominal_power> element. "
           << "Failed to initialize." << std::endl;
    return;
  }
}

void RtgPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    // Publish result
    ignition::msgs::Float msg;
    msg.set_data(this->nominalPower);
    this->pub.Publish(msg);

    igndbg << "Radioisotope Thermal Generator Plugin:: Current power output: " << this->nominalPower << " watts" << std::endl;
  }
}

IGNITION_ADD_PLUGIN(RtgPlugin, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(RtgPlugin, "simulation::RtgPlugin")
