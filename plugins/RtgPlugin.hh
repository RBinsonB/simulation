#ifndef RTGPLUGIN_HH_
#define RTGPLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport/Node.hh>
#include <ignition/physics/Entity.hh>
#include "ignition/gazebo/Model.hh"

namespace simulation
{
  class RtgPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: void Configure(const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;

    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
        const ignition::gazebo::EntityComponentManager &_ecm) final;

    private: std::string linkName;
    private: std::string modelName;
    private: std::string topicName;
    private: double nominalPower;

    private: ignition::transport::Node node;
    public: ignition::transport::Node::Publisher pub;
  };
}
#endif