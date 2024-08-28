#ifndef SOLARPANELPLUGIN_HH_
#define SOLARPANELPLUGIN_HH_

#include <string>
#include <mutex>

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport/Node.hh>
#include <ignition/physics/Entity.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/common/Event.hh>

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

    public: std::vector<std::string> GetVisualChildren(
        const ignition::gazebo::EntityComponentManager &_ecm);

    public: void SetScene(ignition::rendering::ScenePtr _scene);

    public: bool FindScene();

    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    /// \brief Event that is used to trigger callbacks when the scene
    /// is changed
    public: static ignition::common::EventT<
            void(const ignition::rendering::ScenePtr &)> sceneEvent;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

    /// \brief Pointer to rendering scene
    private: ignition::rendering::ScenePtr scene{nullptr};
    /// \brief Connection to the Manager's scene change event.
    public: ignition::common::ConnectionPtr sceneChangeConnection;
    /// \brief Just a mutex for thread safety
    public: std::mutex mutex;

    private: std::string linkName;
    private: std::string modelName;
    private: std::string topicName;
    private: float nominalPower{0.0};
    private: std::vector<std::string> scopedVisualChildren;
    
    // \brief Model interface
    public: ignition::gazebo::Model model{ignition::gazebo::v6::kNullEntity};
    public: ignition::gazebo::Entity linkEntity{ignition::gazebo::v6::kNullEntity};

    private: ignition::transport::Node node;
    public: ignition::transport::Node::Publisher pub;
  };
}
#endif