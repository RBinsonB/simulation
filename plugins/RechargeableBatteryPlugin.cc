#include "RechargeableBatteryPlugin.hh"

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/BatteryPowerLoad.hh>
#include "ignition/gazebo/components/BatterySoC.hh"
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>


using namespace simulation;

/////////////////////////////////////////////////
RechargeableBatteryPlugin::RechargeableBatteryPlugin()
    : System(), dataPtr(std::make_unique<RechargeableBatteryPluginPrivate>())
{
}

/////////////////////////////////////////////////
RechargeableBatteryPlugin::~RechargeableBatteryPlugin()
{
    this->dataPtr->Reset();

    if(this->dataPtr->battery)
    {
        // consumer-specific
        if (this->dataPtr->consumerId !=-1)
        {
            this->dataPtr->battery->RemoveConsumer(this->dataPtr->consumerId);
        }

        // This is needed so that common::Battery stops calling the update function
    //   of this object, when this object is destroyed. Else seg fault in test,
    //   though no seg fault in actual run.
        this->dataPtr->battery->ResetUpdateFunc();
    }
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          ignition::gazebo::EntityComponentManager &_ecm,
                                          ignition::gazebo::EventManager &_eventMgr)
{
    // Store the pointer to the model this battery is under
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm))
    {
        ignerr << "Battery plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->model = model;
    this->dataPtr->modelName = model.Name(_ecm);

    if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
        this->dataPtr->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

    if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
        this->dataPtr->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

    if (_sdf->HasElement("capacity"))
        this->dataPtr->c = _sdf->Get<double>("capacity");

    if (this->dataPtr->c <= 0)
    {
        ignerr << "No <capacity> or incorrect value specified. Capacity should be "
               << "greater than 0.\n";
        return;
    }

    this->dataPtr->q0 = this->dataPtr->c;
    if (_sdf->HasElement("initial_charge"))
    {
        this->dataPtr->q0 = _sdf->Get<double>("initial_charge");
        if (this->dataPtr->q0 > this->dataPtr->c || this->dataPtr->q0 < 0)
        {
            ignerr << "<initial_charge> value should be between [0, <capacity>]."
                   << std::endl;
            this->dataPtr->q0 =
                std::max(0.0, std::min(this->dataPtr->q0, this->dataPtr->c));
            ignerr << "Setting <initial_charge> to [" << this->dataPtr->q0
                   << "] instead." << std::endl;
        }
    }

    this->dataPtr->q = this->dataPtr->q0;

    if (_sdf->HasElement("resistance"))
        this->dataPtr->r = _sdf->Get<double>("resistance");

    if (_sdf->HasElement("smooth_current_tau"))
    {
        this->dataPtr->tau = _sdf->Get<double>("smooth_current_tau");
        if (this->dataPtr->tau <= 0)
        {
            ignerr << "<smooth_current_tau> value should be positive. "
                   << "Using [1] instead." << std::endl;
            this->dataPtr->tau = 1;
        }
    }

    if (_sdf->HasElement("battery_name") && _sdf->HasElement("voltage"))
    {
        this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
        auto initVoltage = _sdf->Get<double>("voltage");
        ignerr << "Battery name: " << this->dataPtr->batteryName << std::endl;
        ignerr << "Initial voltage: " << initVoltage << std::endl;

        // Create battery entity and some components
        this->dataPtr->batteryEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(this->dataPtr->batteryEntity, ignition::gazebo::components::Name(
                                                               this->dataPtr->batteryName));
        _ecm.SetParentEntity(this->dataPtr->batteryEntity, _entity);

        // Create actual battery and assign update function
        this->dataPtr->battery = std::make_shared<ignition::common::Battery>(
            this->dataPtr->batteryName, initVoltage);
        this->dataPtr->battery->Init();
        // print battery voltage
        ignerr << "Battery voltage: " << this->dataPtr->battery->Voltage() << std::endl;
        this->dataPtr->battery->SetUpdateFunc(
            std::bind(&RechargeableBatteryPlugin::OnUpdateVoltage, this,
                      std::placeholders::_1));
    }
    else
    {
        ignerr << "No <battery_name> or <voltage> specified. Both are required.\n";
        return;
    }

    // Consumer-specific
    if (_sdf->HasElement("power_load"))
    {
        this->dataPtr->initialPowerLoad = _sdf->Get<double>("power_load");
        this->dataPtr->consumerId = this->dataPtr->battery->AddConsumer();
        bool success = this->dataPtr->battery->SetPowerLoad(
            this->dataPtr->consumerId, this->dataPtr->initialPowerLoad);
        if (!success)
            ignerr << "Failed to set consumer power load." << std::endl;
    }
    else
    {
        ignwarn << "Required attribute power_load missing "
                << "in BatteryPlugin SDF" << std::endl;
    }
    if (_sdf->HasElement("start_draining"))
        this->dataPtr->startDraining = _sdf->Get<bool>("start_draining");

    // Subscribe to power draining topics, if any.
    if (_sdf->HasElement("power_draining_topic"))
    {
        sdf::ElementConstPtr sdfElem = _sdf->FindElement("power_draining_topic");
        while (sdfElem)
        {
            const auto &topic = sdfElem->Get<std::string>();
            this->dataPtr->node.SubscribeRaw(topic,
                                             std::bind(&RechargeableBatteryPluginPrivate::OnBatteryDrainingMsg,
                                                       this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
                                                       std::placeholders::_3));
            ignmsg << "RechargeableBatteryPlugin subscribes to power draining topic ["
                   << topic << "]." << std::endl;
            sdfElem = sdfElem->GetNextElement("power_draining_topic");
        }
    }

    // Subscribe to stop power draining topics, if any.
    if (_sdf->HasElement("stop_power_draining_topic"))
    {
        sdf::ElementConstPtr sdfElem =
            _sdf->FindElement("stop_power_draining_topic");
        while (sdfElem)
        {
            const auto &topic = sdfElem->Get<std::string>();
            this->dataPtr->node.SubscribeRaw(topic,
                                             std::bind(&RechargeableBatteryPluginPrivate::OnBatteryStopDrainingMsg,
                                                       this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
                                                       std::placeholders::_3));
            ignmsg << "RechargeableBatteryPlugin subscribes to stop power draining topic ["
                   << topic << "]." << std::endl;
            sdfElem = sdfElem->GetNextElement("power_draining_topic");
        }
    }

    // subscriber to all power sources topics
    if(_sdf->HasElement("power_source"))
    {
        sdf::ElementConstPtr powerSourceElem = _sdf->FindElement("power_source");
        int id = 0;
        while(powerSourceElem)
        {
            const auto &topic = powerSourceElem->Get<std::string>();
            std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/" +
                           topic};
            auto validPowerSourceTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
            if(validPowerSourceTopic.empty())
            {
                ignerr << "Failed to create valid topic. Not valid: ["
                       << topic << "]" << std::endl;
                return;
            }
            powerSourceType powerSource;
            powerSource.id = id;
            powerSource.power = 0.0;
            powerSource.updated = false;
            std::function<void(const ignition::msgs::Float &)> callback = std::bind(&RechargeableBatteryPluginPrivate::OnPowerSourceMsg, 
                                                                                    this->dataPtr.get(), id, std::placeholders::_1);    
            this->dataPtr->node.Subscribe(validPowerSourceTopic, callback);
            ignerr << "RechargeableBatteryPlugin subscribes to power source topic ["
                   << validPowerSourceTopic << "]." << std::endl;
            this->dataPtr->powerSources.emplace_back(std::move(powerSource));
            powerSourceElem = powerSourceElem->GetNextElement("power_source");
            id++;
        }
    }
    else 
    {
        ignerr << "No power source topic specified." << std::endl;
    }
    

    ignmsg << "RechargeableBatteryPlugin configured. Battery name: "
           << this->dataPtr->battery->Name() << std::endl;
    igndbg << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage()
           << std::endl;

    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;
    // Initialize battery with initial calculated state of charge
    _ecm.CreateComponent(this->dataPtr->batteryEntity,
                         ignition::gazebo::components::BatterySoC(this->dataPtr->soc));

    // Setup battery state topic
    std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                           "/battery/" + this->dataPtr->battery->Name() + "/state"};

    auto validStateTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
    if (validStateTopic.empty())
    {
        ignerr << "Failed to create valid state topic ["
               << stateTopic << "]" << std::endl;
        return;
    }

    ignition::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(50);
    this->dataPtr->statePub = this->dataPtr->node.Advertise<ignition::msgs::BatteryState>(
        validStateTopic, opts);
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::PreUpdate");
    // Get the total power load 
    double total_power_load = this->dataPtr->initialPowerLoad;
    // ignerr << "Battery power load: " << total_power_load << std::endl;
    _ecm.Each<ignition::gazebo::components::BatteryPowerLoad>(
    [&](const ignition::gazebo::Entity & /*_entity*/,
        const ignition::gazebo::components::BatteryPowerLoad *_batteryPowerLoadInfo)->bool
    {
      // print battery power id  and the entity id
    //   ignerr << "sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss" << std::endl;
    //   ignerr << "Battery power id: " << _batteryPowerLoadInfo->Data().batteryId << std::endl;
    //   ignerr << "Entity id: " << this->dataPtr->batteryEntity << std::endl;
    //   ignerr << "Power load: " << _batteryPowerLoadInfo->Data().batteryPowerLoad << std::endl;
        // ignerr << "batteryentity: " << this->dataPtr->batteryEntity << std::endl;
      if (_batteryPowerLoadInfo->Data().batteryId ==
          this->dataPtr->batteryEntity)
      {
        total_power_load = total_power_load +
            _batteryPowerLoadInfo->Data().batteryPowerLoad;
            // ignerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
            // ignerr << "Battery power load: " << _batteryPowerLoadInfo->Data().batteryPowerLoad << std::endl;
            // // // print consumer id
            // ignerr << "Consumer id: " << _batteryPowerLoadInfo->Data().batteryId << std::endl;
            // ignerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
      }
      return true;
    });
    ignerr << "Battery power load: " << total_power_load << std::endl;

    bool success = this->dataPtr->battery->SetPowerLoad(
        this->dataPtr->consumerId, total_power_load);
    if (!success)
        ignerr << "Failed to set consumer power load." << std::endl;

    // // Update total power supply
    // for(auto &powerSource : this->dataPtr->powerSources)
    // {
    //     std::lock_guard<std::mutex> lock(*(powerSource.mutex_ptr));
    //     if(powerSource.updated)
    //     {
    //         powerSource.updated = false;
    //         ignerr << "Power source [" << powerSource.id << "] updated with power: " << powerSource.power << std::endl;
    //         this->dataPtr->totalPowerSupply += powerSource.power;
    //     }
    // }

    // start draining the battery if the robot has started moving
    if (!this->dataPtr->startDraining)
    {
        const std::vector<ignition::gazebo::Entity> &joints =
            _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), ignition::gazebo::components::Joint());

        for (ignition::gazebo::Entity jointEntity : joints)
        
        {
            const auto *jointVelocityCmd =
                _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
            if(jointVelocityCmd)
            {
                for(double jointVel : jointVelocityCmd->Data())
                {
                    if(fabs(static_cast<float>(jointVel)) > 0.01)
                    {
                        this->dataPtr->startDraining = true;
                        ignerr << "Robot is moving2------------------------------------------------------------------------------------------------------" << std::endl;
                        break;
                    }
                }
            }

            const auto *jointForceCmd =
                _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity);
            if(jointForceCmd)
            {
                for(double jointForce : jointForceCmd->Data())
                {
                    if(fabs(static_cast<float>(jointForce)) > 0.01)
                    {
                        this->dataPtr->startDraining = true;
                        ignerr << "Robot is moving1------------------------------------------------------------------------------------------------------" << std::endl; 
                        break;
                    }
                }
            }
        }
    }


}

///////////////////////////////////////////////
void RechargeableBatteryPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                                       ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::Update");
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

    if (_info.paused)
        return;

    // ignerr << "Battery update" <<  this->dataPtr->startDraining << std::endl;

    if (!this->dataPtr->startDraining)
        return;

    // Find the time at which battery starts to drain
    int simTime = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
    if (this->dataPtr->drainStartTime == -1)
        this->dataPtr->drainStartTime = simTime;

    // Print drain time in minutes
    int drainTime = (simTime - this->dataPtr->drainStartTime) / 60;
    if (drainTime != this->dataPtr->lastPrintTime)
    {
        this->dataPtr->lastPrintTime = drainTime;
        igndbg << "[Battery Plugin] Battery drain: " << drainTime <<
        " minutes passed.\n";
    }

    // update step size 
    this->dataPtr->stepSize = _info.dt;

    double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
    this->dataPtr->stepSize).count()) * 1e-9;
    if (this->dataPtr->tau < dt)
    {
        ignerr << "<smooth_current_tau> should be in the range [dt, +inf) but is "
            << "configured with [" << this->dataPtr->tau << "]. We'll be using "
            << "[" << dt << "] instead" << std::endl;
        this->dataPtr->tau = dt;
    }

    if(this->dataPtr->battery)
    {
        // Update battery state
        // print battery voltage and battery initial voltage
        // ignerr << "----------------------------------------------------------" << std::endl;
        // ignerr << "Battery voltage: " << this->dataPtr->battery->Voltage() << std::endl;
        // ignerr << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage() << std::endl;
        this->dataPtr->battery->Update();

        // print after battery update
        // ignerr << "Battery voltage: " << this->dataPtr->battery->Voltage() << std::endl;
//         ignerr << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage() << std::endl;
//  ignerr << "----------------------------------------------------------" << std::endl;
        // Update battery state of charge
        auto *batteryComp = _ecm.Component<ignition::gazebo::components::BatterySoC>(
            this->dataPtr->batteryEntity);

        batteryComp->Data() = this->dataPtr->StateOfCharge();
    }


}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                           const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::PostUpdate");
    // Nothing left to do if paused or the publisher wasn't created.
    if (_info.paused || !this->dataPtr->statePub)
        return;
    
    // Publish battery state
    ignition::msgs::BatteryState msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime));
    msg.set_voltage(this->dataPtr->battery->Voltage());
    msg.set_current(this->dataPtr->ismooth);
    msg.set_charge(this->dataPtr->q);
    msg.set_capacity(this->dataPtr->c);
    msg.set_percentage(this->dataPtr->soc * 100);

    if (this->dataPtr->isCharging)
        msg.set_power_supply_status(ignition::msgs::BatteryState::CHARGING);
    else if (this->dataPtr->startDraining)
        msg.set_power_supply_status(ignition::msgs::BatteryState::DISCHARGING);
    else if (!this->dataPtr->StateOfCharge() > 0.9)
        msg.set_power_supply_status(ignition::msgs::BatteryState::FULL);
    else 
        msg.set_power_supply_status(ignition::msgs::BatteryState::NOT_CHARGING);

    this->dataPtr->statePub.Publish(msg); 
}

/////////////////////////////////////////////////
double RechargeableBatteryPlugin::OnUpdateVoltage(const ignition::common::Battery *_battery)
{
    IGN_ASSERT(_battery != nullptr, "Battery pointer is null");

    // ignerr << "Battery update voltage" << std::endl;

    // ignerr << "Battery voltage: before if" << _battery->Voltage() << std::endl;

    if(fabs(_battery->Voltage()) < 1e-3)
    {
        ignerr << "Battery voltage is zero" << std::endl;
        return 0.0;
    }

    // ignerr << "Battery voltage: after if" << _battery->Voltage() << std::endl;

    // ignerr << "Battery voltage: after if" << _battery->Voltage() << std::endl;
    if(this->dataPtr->StateOfCharge() <= 0)
    {
        ignerr << "Battery is out of charge" << std::endl;
        return _battery->Voltage();
    }
        
    
    // ignerr << "Battery update voltage two" << std::endl;
    
    auto prevSocInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

    // seconds
    double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->dataPtr->stepSize).count()) * 1e-9;
    double totalpower = 0.0;
    double k = dt / this->dataPtr->tau;

    if(this->dataPtr->startDraining)
    {
        for (auto powerLoad : _battery->PowerLoads())
        {
            totalpower += powerLoad.second;
        }
    }

    this->dataPtr->iraw = totalpower / _battery->Voltage();

    // ignerr << "iraw: " << this->dataPtr->iraw << std::endl;
    // ignerr << "volt: " << _battery->Voltage() << std::endl;
    // ignerr << "total power: " << totalpower << std::endl;

    // current state of charge
    // Update total power supply
    for(auto &powerSource : this->dataPtr->powerSources)
    {
        std::lock_guard<std::mutex> lock(*(powerSource.mutex_ptr));
        if(powerSource.updated)
        {
            powerSource.updated = false;
            ignerr << "Power source [" << powerSource.id << "] updated with power: " << powerSource.power << std::endl;
            this->dataPtr->totalPowerSupply += powerSource.power;
        }
    }
    // if(this->dataPtr->totalPowerSupply > 100)
    // {
    //     ignerr << "-slssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssk" << std::endl;
    // }
    ignerr << "SOC: " << this->dataPtr->StateOfCharge() << std::endl;
    ignerr << "total power supply: " << this->dataPtr->totalPowerSupply << std::endl;
    ignerr << "total power: " << totalpower << std::endl;
    ignerr << "iraw: " << this->dataPtr->iraw << std::endl;

    // check if the total powersupply is not zero
    this->dataPtr->isCharging = this->dataPtr->totalPowerSupply > 1e-9 && this->dataPtr->StateOfCharge() < 0.9;

    // compute current due to power sources
    float powerSourceCurrent = 0.0;
    if(this->dataPtr->isCharging)
    {
        powerSourceCurrent = this->dataPtr->totalPowerSupply / _battery->Voltage();
        ignerr << "Charging current: " << powerSourceCurrent << std::endl;
        ignerr << "iraw: " << this->dataPtr->iraw << std::endl;
        this->dataPtr->iraw -= powerSourceCurrent;
        ignerr << "Charging current: " << powerSourceCurrent << std::endl;
        ignerr << "iraw: " << this->dataPtr->iraw << std::endl;

    }
    // reset total power supply to zero
    this->dataPtr->totalPowerSupply = 0.0;

    // if(this->dataPtr->StateOfCharge() < 0.9)
    // {
    //     this->dataPtr->iraw -= powerSourceCurrent;
    // }

    this->dataPtr->ismooth = this->dataPtr->ismooth + k *
    (this->dataPtr->iraw - this->dataPtr->ismooth);

    // Convert dt to hours
    this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) /
        3600.0);
        ignerr << "Dt: " << dt*1000 << std::endl;


    // open circuit voltage
    double voltage = this->dataPtr->e0 + this->dataPtr->e1 * (
    1 - this->dataPtr->q / this->dataPtr->c)
      - this->dataPtr->r * this->dataPtr->ismooth;

    ignerr << "e0: " << this->dataPtr->e0 << std::endl;
    ignerr << "e1: " << this->dataPtr->e1 << std::endl;
    ignerr << "q: " << this->dataPtr->q << std::endl;
    ignerr << "c: " << this->dataPtr->c << std::endl;
    ignerr << "r: " << this->dataPtr->r << std::endl;
    ignerr << "ismooth: " << this->dataPtr->ismooth << std::endl;

    ignerr << "Battery voltage: before update voltage" << voltage << std::endl;

    // stop program if the voltage is zero
    
    // print all the values
    // ignerr << "volt: " << voltage << std::endl;
    // ignerr << "e0: " << this->dataPtr->e0 << std::endl;
    // ignerr << "e1: " << this->dataPtr->e1 << std::endl;
    // ignerr << "q: " << this->dataPtr->q << std::endl;
    // ignerr << "c: " << this->dataPtr->c << std::endl;
    // ignerr << "r: " << this->dataPtr->r << std::endl;
    // ignerr << "ismooth: " << this->dataPtr->ismooth << std::endl;

    // Estimate state of charge
    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;

    // Throttle debug messages
    auto socInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

    if (socInt % 10 == 0 && socInt != prevSocInt)
    {
        igndbg << "Battery: " << this->dataPtr->battery->Name() << std::endl;
        igndbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
            << std::endl;
        igndbg << "charging current: " << powerSourceCurrent << std::endl;
        igndbg << "voltage: " << voltage << std::endl;
        igndbg << "state of charge: " << this->dataPtr->StateOfCharge()
            << " (q " << this->dataPtr->q << ")" << std::endl << std::endl;
    }
    if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->drainPrinted)
    {
        ignwarn << "Model " << this->dataPtr->modelName << " out of battery.\n";
        this->dataPtr->drainPrinted = true;
    }
    // ignerr << "**************************************************************" << std::endl;
    // ignerr << "Battery voltage: after update voltage" << voltage << std::endl;
    // ignerr << "----------------------------------------------------------" << std::endl;

    return voltage;


}

/////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::Reset()
{
    // Reset the plugin
   this->totalPowerSupply = 0.0;
   this->ismooth = 0.0;
    this->iraw = 0.0;   
    this->q = this->q0;
    this->startDraining = false;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnBatteryDrainingMsg(
  const char *, const size_t, const ignition::transport::MessageInfo &)
{
  this->startDraining = true;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnBatteryStopDrainingMsg(
  const char *, const size_t, const ignition::transport::MessageInfo &)
{
  this->startDraining = false;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnPowerSourceMsg(int _id,
  const ignition::msgs::Float &_msg)
{
    std::lock_guard<std::mutex> lock(*(this->powerSources[_id].mutex_ptr));
    this->powerSources[_id].power = _msg.data();
    this->powerSources[_id].updated = true;
    // ignerr << "Power source [" << _id << "] updated with power: " << _msg.data() << std::endl;
}


//////////////////////////////////////////////////
double RechargeableBatteryPluginPrivate::StateOfCharge() const
{
    return this->soc;
}


#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(RechargeableBatteryPlugin,
                    ignition::gazebo::System,
                    RechargeableBatteryPlugin::ISystemConfigure,
                    RechargeableBatteryPlugin::ISystemPreUpdate,
                    RechargeableBatteryPlugin::ISystemUpdate,
                    RechargeableBatteryPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RechargeableBatteryPlugin, "simulation::RechargeableBatteryPlugin")
