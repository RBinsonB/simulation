#ifndef SENSOR_POWER_SYSTEM_HH_
#define SENSOR_POWER_SYSTEM_HH_


#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
#include "gz/sim/Model.hh"
#include <ignition/gazebo/System.hh>
#include <gz/common/Util.hh>
#include <gz/sensors/Manager.hh>

namespace simulation
{
    // todo add this functionality to the sensor plugin

  struct sensorType
  {
        int id{0};
      std::string name;
      bool state{false};
    std::unique_ptr<std::mutex> mutex_ptr = std::make_unique<std::mutex>();
      bool updated{false};
      float power{0.0};
      std::string batteryName;
      bool batteryExist{false};
      ignition::gazebo::Entity BatteryConsumerEntity{ignition::gazebo::kNullEntity};
      ignition::gazebo::Entity BatteryEntity{ignition::gazebo::kNullEntity};
  };


  class SensorPowerSystemPrivate
  {
      public:
            void OnActivateSensor(int _id, const ignition::msgs::Boolean &_msg);

     public:
        std::string modelName;
    public:
        gz::sim::Model model{ignition::gazebo::kNullEntity};

    /// \brief Ignition communication node
    public:
        ignition::transport::Node node;

    public:
        std::vector<sensorType> sensorsInfo;
    public: 
        std::vector<std::string> batteryNames;
    
    public: 
     bool battery_initialized{false};

    public: bool enabled{true};


    // public: std::unordered_map<ignition::gazebo::Entity, std::vector<sensorType>> sensorMap;

    public: bool HasSufficientBattery(const ignition::gazebo::EntityComponentManager &_ecm) const;
  };

class SensorPowerSystemPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
      {
            // Constructor
        public:
            SensorPowerSystemPlugin();

            // Destructor
        public:
            ~SensorPowerSystemPlugin() override;

            // Configure the plugin
        public:
            void Configure(const ignition::gazebo::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        ignition::gazebo::EntityComponentManager &_ecm,
                        ignition::gazebo::EventManager &_eventMgr) final;
        public:
            void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm) override;

            // PostUpdate
        public:
            void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm) override;
        private:
            std::unique_ptr<SensorPowerSystemPrivate> dataPtr;
      };






}

#endif // SENSOR_POWER_SYSTEM_HH_