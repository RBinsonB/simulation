#ifndef SOLARPANELPLUGIN_HH_
#define SOLARPANELPLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport/Node.hh>
#include <ignition/physics/Entity.hh>
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/Model.hh"
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/World.hh>

namespace simulation
{
  class SolarPanelPlugin:
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

    public: ignition::gazebo::Entity FindEntityFromRenderingName(
        const ignition::gazebo::EntityComponentManager &_ecm,
        const std::string &_renderingName);

    public: std::vector<std::string> GetVisualChildren(
        const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Pointer to rendering scene
    private: ignition::rendering::ScenePtr scene{nullptr};


    private: std::string linkName;
    private: std::string modelName;
    private: std::string topicName;
    private: std::vector<std::string> scopedVisualChildren;
    // \brief Model interface
    public: ignition::gazebo::Model model{ignition::gazebo::v6::kNullEntity};
    public: ignition::gazebo::Entity linkEntity{ignition::gazebo::v6::kNullEntity};

    private: ignition::transport::Node node;
    public: ignition::transport::Node::Publisher pub;
  };
}
#endif