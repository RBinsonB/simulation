#include "SensorPowerSystemPlugin.hh"
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/Imu.hh>
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BatteryPowerLoad.hh"
#include <sdf/Sensor.hh>

// include sensor manager
#include "ignition/sensors/Sensor.hh"

using namespace simulation;


/////////////////////////////////////////////////
SensorPowerSystemPlugin::SensorPowerSystemPlugin()
    : System(), dataPtr(std::make_unique<SensorPowerSystemPrivate>())
{
}

/////////////////////////////////////////////////
SensorPowerSystemPlugin::~SensorPowerSystemPlugin()
{
    // this->dataPtr->Reset();
}


/////////////////////////////////////////////////
void SensorPowerSystemPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        ignition::gazebo::EntityComponentManager &_ecm,
                                        ignition::gazebo::EventManager &_eventMgr)
{
    // Store the pointer to the model this battery is under
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm))
    {
        ignerr << "SensorPowerSystemPlugin plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->model = model;
    this->dataPtr->modelName = model.Name(_ecm);
    ignerr << "Model name: " << this->dataPtr->modelName << std::endl;
    ignerr << "******************************************************" << std::endl;

    // get all camera components
    // todo include more sensors
    int sensorCount = 0;

    _ecm.Each<ignition::gazebo::components::Camera>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Camera *_camera)->bool
        {
            // get the camera name
            auto cameraName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
            if(cameraName)
            {
                ignerr << "Camera name: " << cameraName->Data() << std::endl;
                auto cameraPtr = _ecm.Component<ignition::gazebo::components::Camera>(_entity);
                sdf::Sensor sensor = cameraPtr->Data();
                auto camera = sensor.CameraSensor();
                // std::cout << "Update rate: " << camera.UpdateRate() << std::endl;
                auto sdf_elem = camera->Element();
                auto parent = sdf_elem->GetParent();
                if(parent->HasElement("power_load") && parent->HasElement("battery_name"))
                {
                    ignerr << "power_load:  " << parent->Get<double>("power_load") << std::endl;
                    ignerr << "battery_name:  " << parent->Get<std::string>("battery_name") << std::endl;
                    sensorType sensorInfo;
                    sensorInfo.id = sensorCount;
                    sensorInfo.name = cameraName->Data();
                    sensorInfo.power = parent->Get<double>("power_load");
                    sensorInfo.batteryName = parent->Get<std::string>("battery_name");
                    sensorInfo.state = true;
                    sensorInfo.updated = false;
                    sensorCount++;
                    this->dataPtr->sensorsInfo.emplace_back(std::move(sensorInfo));
                }
            }
            return true;
        });
    
    // imu sensors
    _ecm.Each<ignition::gazebo::components::Imu>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Imu *_imu)->bool
        {
            // get the imu name
            auto imuName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
            if(imuName)
            {
                ignerr << "Imu name: " << imuName->Data() << std::endl;
                auto imuPtr = _ecm.Component<ignition::gazebo::components::Imu>(_entity);
                sdf::Sensor sensor = imuPtr->Data();
                auto imu = sensor.ImuSensor();
                // std::cout << "Update rate: " << camera.UpdateRate() << std::endl;
                auto sdf_elem = imu->Element();
                auto parent = sdf_elem->GetParent();
                if(parent->HasElement("power_load") && parent->HasElement("battery_name"))
                {
                    ignerr << "power_load:  " << parent->Get<double>("power_load") << std::endl;
                    ignerr << "battery_name:  " << parent->Get<std::string>("battery_name") << std::endl;
                    sensorType sensorInfo;
                    sensorInfo.id = sensorCount;
                    sensorInfo.name = imuName->Data();
                    sensorInfo.power = parent->Get<double>("power_load");
                    sensorInfo.batteryName = parent->Get<std::string>("battery_name");
                    sensorInfo.state = true;
                    sensorInfo.updated = false;
                    sensorCount++;
                    this->dataPtr->sensorsInfo.emplace_back(std::move(sensorInfo));
                }
            }
            return true;
        });

    // create a subscription to the sensor topic

    for(auto &sensor : this->dataPtr->sensorsInfo)
    {
        ignerr << "Sensor name: " << sensor.name << std::endl;
        ignerr << "Sensor power: " << sensor.power << std::endl;
        ignerr << "Sensor battery name: " << sensor.batteryName << std::endl;
        std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/sensor/" + sensor.name + "/trigger"};
        auto validSensorTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
        if(validSensorTopic.empty())
        {
            ignerr << "Failed to create valid topic. Not valid: ["
                   << sensor.name << "]" << std::endl;
            return;
        }
        std::function<void(const ignition::msgs::Boolean &)> callback = std::bind(&SensorPowerSystemPrivate::OnActivateSensor, 
                                                                                this->dataPtr.get(), sensor.id, std::placeholders::_1);
        this->dataPtr->node.Subscribe(validSensorTopic, callback);
    }
}

// add a preupdate stage that checks if the sensor is active and updates the 
// flags accordingly
void SensorPowerSystemPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & _info,
                            ignition::gazebo::EntityComponentManager &_ecm)

{
    if(!this->dataPtr->enabled || _info.paused)
    {
        return;
    }
    // ignerr << "PreUpdate" << std::endl;


      if(!this->dataPtr->battery_initialized)
            {
                ignerr << "++++++++++++++++++++++++++++++++++++++++Battery not initialized" << std::endl;
            this->dataPtr->battery_initialized = true;
            _ecm.Each<ignition::gazebo::components::BatterySoC, ignition::gazebo::components::Name>(
                [&](const ignition::gazebo::Entity &_entity,
                    const ignition::gazebo::components::BatterySoC *_batterySoc,
                    const ignition::gazebo::components::Name *_name)->bool
                {
                    ignerr << "Battery name: " << _name->Data() << std::endl;

                
                // get the battery name
                // auto batteryName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
                if(_name)
                {
                    // ignerr << "Battery name: " << batteryName->Data() << std::endl;
                    // auto batteryPtr = _ecm.Component<ignition::gazebo::components::BatterySoC>(_entity);
                    // // ignerr << "Battery SOC: " << batteryPtr->Data().StateOfCharge() << std::endl;
                    // this->dataPtr->batteryNames.push_back(batteryName->Data());
                    // // check if the battery ,atch the sensor battery name
                    for(auto &sensor : this->dataPtr->sensorsInfo)
                    {
                        if(sensor.batteryName == _name->Data())
                        {
                            ignerr << "Battery found for sensor: " << sensor.name << std::endl;

                            sensor.batteryExist = true;
                            sensor.BatteryEntity = _entity;
                        }
                    }
                }
                return true;
            });
                for(auto &sensor : this->dataPtr->sensorsInfo)
                {
                    if(sensor.batteryExist)
                    {
                    sensor.BatteryConsumerEntity = _ecm.CreateEntity();
                    ignition::gazebo::components::BatteryPowerLoadInfo batteryPowerLoad{
                            sensor.BatteryEntity, sensor.power};
                        _ecm.CreateComponent(sensor.BatteryConsumerEntity,  ignition::gazebo::components::BatteryPowerLoad(batteryPowerLoad));
                        _ecm.SetParentEntity(sensor.BatteryConsumerEntity, sensor.BatteryEntity);
                    }
                    else
                    {
                        ignerr << "Sensor " << sensor.name << " battery does not exist" << std::endl;
                    }
                }
        }
        else 
        {
            for(auto &sensor : this->dataPtr->sensorsInfo)
            {
                if(sensor.batteryExist)
                {
                    if(sensor.updated)
                    {
                        float setPower = sensor.power;
                        if(!sensor.state)
                        {
                            setPower = 0.0;
                        }
                        std::lock_guard<std::mutex> lock(*sensor.mutex_ptr);
                        sensor.updated = false;
                        ignition::gazebo::v6::components::BatteryPowerLoadInfo batteryPowerLoad{
                            sensor.BatteryConsumerEntity, setPower};
                        _ecm.SetComponentData<ignition::gazebo::components::BatteryPowerLoad>(sensor.BatteryConsumerEntity, batteryPowerLoad);

                    }
                    
                }
            }
        }
}


/////////////////////////////////////////////////
void SensorPowerSystemPlugin::PostUpdate(const ignition::gazebo::UpdateInfo & _info,
                            const ignition::gazebo::EntityComponentManager &_ecm)

{
    // if(_info.paused)
    // {
    //     return;
    // }
    // nmake this per sensor
    // this->dataPtr->enabled = this->dataPtr->HasSufficientBattery(_ecm);
    
}


bool SensorPowerSystemPrivate::HasSufficientBattery(
  const ignition::gazebo::EntityComponentManager &_ecm) const
{
  bool result = true;
//   (void)_ecm;
//   _ecm.Each<ignition::gazebo::components::BatterySoC>([&](
//     const ignition::gazebo::Entity &_entity,
//     const ignition::gazebo::components::BatterySoC *_data
//   ){
//     if(_ecm.ParentEntity(_entity) == this->modelEntity)
//     {
//       if(_data->Data() <= 0)
//       {
//         result = false;
//       }
//     }

//     return true;
//   });
//   return result;
}



void SensorPowerSystemPrivate::OnActivateSensor(int _id, const ignition::msgs::Boolean &_msg)
{
    // std::lock_guard<std::mutex> lock(*this->sensors[_id].mutex_ptr);
    // ignerr << "Sensor " << this->sensors[_id].name << " with message " << _msg.data() << std::endl;
    // this->sensors[_id].state = _msg.data();
    // this->sensors[_id].updated = true;
}










#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(SensorPowerSystemPlugin,
                    ignition::gazebo::System,
                    SensorPowerSystemPlugin::ISystemConfigure,
                    SensorPowerSystemPlugin::ISystemPreUpdate,
                    SensorPowerSystemPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SensorPowerSystemPlugin, "simulation::SensorPowerSystemPlugin")