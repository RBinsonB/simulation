#ifndef Rechargeable_BATTERY_PLUGIN_HH_
#define Rechargeable_BATTERY_PLUGIN_HH_

#include <string>
#include <map>
#include <memory>

#include <gz/common/Battery.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <ignition/msgs/battery_state.pb.h>

#include "gz/sim/Model.hh"

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace simulation
{
    struct powerSourceType
    {
        int id{0};
        double power{0.0};
        std::unique_ptr<std::mutex> mutex_ptr = std::make_unique<std::mutex>();
        bool updated{false};
    };

    // Forward declaration
    class RechargeableBatteryPluginPrivate
    {
    public:
        void Reset();
        /// \brief Name that identifies a battery.

        /// \brief Get the current state of charge of the battery.
        /// \return State of charge of the battery in range [0.0, 1.0].
    public:
        double StateOfCharge() const;

        /// \brief Callback connected to additional topics that can start battery
        /// draining.
        /// \param[in] _data Message data.
        /// \param[in] _size Message data size.
        /// \param[in] _info Information about the message.
    public:
        void OnBatteryDrainingMsg(
            const char *_data, const size_t _size,
            const ignition::transport::MessageInfo &_info);

        /// \brief Callback connected to additional topics that can stop battery
        /// draining.
        /// \param[in] _data Message data.
        /// \param[in] _size Message data size.
        /// \param[in] _info Information about the message.
    public:
        void OnBatteryStopDrainingMsg(
            const char *_data, const size_t _size,
            const ignition::transport::MessageInfo &_info);

    public:
        void OnPowerSourceMsg(int _id,
                              const ignition::msgs::Float &_msg);

        /// \brief Ignition communication node
    public:
        ignition::transport::Node node;

        /// \brief Battery state of charge message publisher
    public:
        ignition::transport::Node::Publisher statePub;

    public:
        std::string batteryName;

        /// \brief Pointer to battery contained in link.
    public:
        gz::common::BatteryPtr battery;

        /// \brief Whether warning that battery has drained has been printed once.
    public:
        bool drainPrinted{false};

        /// \brief Battery consumer identifier.
        /// Current implementation limits one consumer (Model) per battery.
    public:
        int32_t consumerId;

        /// \brief Battery entity
    public:
        ignition::gazebo::Entity batteryEntity{ignition::gazebo::kNullEntity};

    public:
        std::string modelName;

    public:
        gz::sim::Model model{ignition::gazebo::kNullEntity};

        /// \brief Open-circuit voltage.
        /// E(t) = e0 + e1 * Q(t) / c
    public:
        double e0{0.0};

    public:
        double e1{0.0};

        /// \brief Initial battery charge in Ah.
    public:
        double q0{0.0};

        /// \brief Battery capacity in Ah.
    public:
        double c{0.0};

        /// \brief Battery inner resistance in Ohm.
    public:
        double r{0.0};

        /// \brief Current low-pass filter characteristic time in seconds [0, 1].
    public:
        double tau{1.0};

        /// \brief Raw battery current in A.
    public:
        double iraw{0.0};

        /// \brief Smoothed battery current in A.
    public:
        double ismooth{0.0};

        /// \brief Instantaneous battery charge in Ah.
    public:
        double q{0.0};

        /// \brief State of charge [0, 1].
    public:
        double soc{1.0};

        /// \brief Simulation time handled during a single update.
    public:
        std::chrono::steady_clock::duration stepSize;

        /// \brief Flag on whether the battery should start draining
    public:
        bool startDraining = false;
        /// \brief The start time when battery starts draining in seconds
    public:
        int drainStartTime = -1;

        /// \brief Book keep the last time printed, so as to not pollute dbg messages
        /// in minutes
    public:
        int lastPrintTime = -1;
        /// \brief Initial power load set trough config
    public:
        double initialPowerLoad = 0.0;

    public:
        double totalPowerSupply = 0.0;
    
    public:
        bool isCharging = false;

        /// \ vector of mutexes to protect the power source
    public:
        std::vector<powerSourceType> powerSources;
    };

    class RechargeableBatteryPlugin
        : public ignition::gazebo::System,
          public ignition::gazebo::ISystemConfigure,
          public ignition::gazebo::ISystemPreUpdate,
          public ignition::gazebo::ISystemUpdate,
          public ignition::gazebo::ISystemPostUpdate
    {
        // COnstructpr
    public:
        RechargeableBatteryPlugin();

        // Destructor
    public:
        ~RechargeableBatteryPlugin() override;

        // Configure the plugin
    public:
        void Configure(const ignition::gazebo::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       ignition::gazebo::EntityComponentManager &_ecm,
                       ignition::gazebo::EventManager &_eventMgr) final;

        // PreUpdate
    public:
        void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                       ignition::gazebo::EntityComponentManager &_ecm) override;

        // Update
    public:
        void Update(const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

        // PostUpdate
    public:
        void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm) override;

        /// \brief Callback for Battery Update events.
        /// \param[in] _battery Pointer to the battery that is to be updated.
        /// \return The new voltage.
    private:
        double OnUpdateVoltage(const ignition::common::Battery *_battery);

        /// \brief Private data pointer
    private:
        std::unique_ptr<RechargeableBatteryPluginPrivate> dataPtr;
    };

}
#endif // Rechargeable_BATTERY_PLUGIN_HH_