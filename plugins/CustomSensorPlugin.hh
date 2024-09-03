/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef CUSTOM_SENSOR_PLUGIN_HH_
#define CUSTOM_SENSOR_PLUGIN_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/Sensor.hh>



#include <atomic>
#include <chrono>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>

#include <gz/rendering/Scene.hh>
#include <ignition/sensors/BoundingBoxCameraSensor.hh>
#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/DepthCameraSensor.hh>
#include <ignition/sensors/GpuLidarSensor.hh>
#include <ignition/sensors/RenderingSensor.hh>
#include <ignition/sensors/RgbdCameraSensor.hh>
#include <ignition/sensors/ThermalCameraSensor.hh>
#include <ignition/sensors/SegmentationCameraSensor.hh>
#include <ignition/sensors/Manager.hh>

#include "ignition/gazebo/components/Atmosphere.hh"
#include "ignition/gazebo/components/BatterySoC.hh"
#include "ignition/gazebo/components/BoundingBoxCamera.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/RenderEngineServerHeadless.hh"
#include "ignition/gazebo/components/RenderEngineServerPlugin.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/SegmentationCamera.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/Events.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

namespace simulation
{

  // Forward declarations.
  // class CustomSensorPluginPrivate;

  // Private data class.
class CustomSensorPluginPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: ignition::sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: std::atomic<bool> initialized { false };

  /// \brief Main rendering interface
  public: ignition::gazebo::RenderUtil renderUtil;

  /// \brief Unique set of sensor ids
  public: std::set<ignition::sensors::SensorId> sensorIds;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: ignition::rendering::ScenePtr scene;

  /// \brief Temperature used by thermal camera. Defaults to temperature at
  /// sea level
  public: double ambientTemperature = 288.15;

  /// \brief Temperature gradient with respect to increasing altitude at sea
  /// level in units of K/m.
  public: double ambientTemperatureGradient = -0.0065;

  /// \brief Keep track of cameras, in case we need to handle stereo cameras.
  /// Key: Camera's parent scoped name
  /// Value: Pointer to camera
  public: std::unordered_map<std::string, ignition::sensors::CameraSensor *> cameras;

  /// \brief Maps gazebo entity to its matching sensor ID
  ///
  /// Useful for detecting when a sensor Entity has been deleted and trigger
  /// the destruction of the corresponding ignition::sensors Sensor object
  public: std::unordered_map<ignition::gazebo::Entity, ignition::sensors::SensorId> entityToIdMap;

  /// \brief Flag to indicate if worker threads are running
  public: std::atomic<bool> running { false };

  /// \brief Flag to signal if initialization should occur
  public: bool doInit { false };

  /// \brief Flag to signal if rendering update is needed
  public: std::atomic<bool> updateAvailable { false };

  /// \brief Flag to signal if a rendering update must be done
  public: std::atomic<bool> forceUpdate { false };

  /// \brief Thread that rendering will occur in
  public: std::thread renderThread;

  /// \brief Mutex to protect rendering data
  public: std::mutex renderMutex;

  /// \brief Mutex to protect renderUtil changes
  public: std::mutex renderUtilMutex;

  /// \brief Condition variable to signal rendering thread
  ///
  /// This variable is used to block/unblock operations in the rendering
  /// thread.  For a more detailed explanation on the flow refer to the
  /// documentation on RenderThread.
  public: std::condition_variable renderCv;

  /// \brief Condition variable to signal update time applied
  ///
  /// This variable is used to block/unblock operations in PostUpdate thread
  /// to make sure renderUtil's ECM updates are applied to the scene first
  /// before they are overriden by PostUpdate
  public: std::condition_variable updateTimeCv;

  /// \brief Connection to events::Stop event, used to stop thread
  public: ignition::common::ConnectionPtr stopConn;

  /// \brief Update time for the next rendering iteration
  public: std::chrono::steady_clock::duration updateTime;

  /// \brief Update time applied in the rendering thread
  public: std::chrono::steady_clock::duration updateTimeApplied;

  /// \brief Update time to be appplied in the rendering thread
  public: std::chrono::steady_clock::duration updateTimeToApply;

  /// \brief Next sensors update time
  public: std::chrono::steady_clock::duration nextUpdateTime;

  /// \brief Sensors to include in the next rendering iteration
  public: std::set<ignition::sensors::SensorId> activeSensors;

  /// \brief Sensors to be updated next
  public: std::set<ignition::sensors::SensorId> sensorsToUpdate;

  /// \brief Mutex to protect sensorMask
  public: std::mutex sensorsMutex;

  /// \brief Pointer to the event manager
  public: ignition::gazebo::EventManager *eventManager{nullptr};

  /// \brief Wait for initialization to happen
  private: void WaitForInit();

  /// \brief Run one rendering iteration
  private: void RunOnce();

  /// \brief Top level function for the rendering thread
  ///
  /// This function captures all of the behavior of the rendering thread.
  /// The behavior is captured in two phases: initialization and steady state.
  ///
  /// When the thread is first started, it waits on renderCv until the
  /// prerequisites for initialization are met, and the `doInit` flag is set.
  /// In order for initialization to proceed, rendering sensors must be
  /// available in the EntityComponentManager.
  ///
  /// When doInit is set, and renderCv is notified, initialization
  /// is performed (creating the render context and scene). During
  /// initialization, execution is blocked for the caller of PostUpdate.
  /// When initialization is complete, PostUpdate will be notified via
  /// renderCv and execution will continue.
  ///
  /// Once in steady state, a rendering operation is triggered by setting
  /// updateAvailable to true, and notifying via the renderCv.
  /// The rendering operation is done in `RunOnce`.
  ///
  /// The caller of PostUpdate will not be blocked if there is no
  /// rendering operation currently ongoing. Rendering will occur
  /// asyncronously.
  //
  /// The caller of PostUpdate will be blocked if there is a rendering
  /// operation currently ongoing, until that completes.
  private: void RenderThread();

  /// \brief Launch the rendering thread
  public: void Run();

  /// \brief Stop the rendering thread
  public: void Stop();

  /// \brief Update battery state of sensors in model
  /// \param[in] _ecm Entity component manager
  public: void UpdateBatteryState(const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Get the next closest sensor update time
  public: std::chrono::steady_clock::duration NextUpdateTime(
      std::set<ignition::sensors::SensorId> &_sensorsToUpdate,
      const std::chrono::steady_clock::duration &_currentTime);

  /// \brief Check if any of the sensors have connections
  public: bool SensorsHaveConnections();

  /// \brief Check if sensor has subscribers
  /// \param[in] _sensor Sensor to check
  /// \return True if the sensor has subscribers, false otherwise
  public: bool HasConnections(ignition::sensors::RenderingSensor *_sensor) const;

  /// \brief Use to optionally set the background color.
  public: std::optional<ignition::math::Color> backgroundColor;

  /// \brief Use to optionally set the ambient light.
  public: std::optional<ignition::math::Color> ambientLight;

  /// \brief A map between model entity ids in the ECM to its battery state
  /// True means has charge, false means drained
  public: std::unordered_map<ignition::gazebo::Entity, bool> modelBatteryState;

  /// \brief A map between model entity ids in the ECM to whether its battery
  /// state has changed.
  /// True means has charge, false means drained
  public: std::unordered_map<ignition::gazebo::Entity, bool> modelBatteryStateChanged;

  /// \brief A map of sensor ids to their active state
  public: std::unordered_map<ignition::sensors::SensorId, bool> sensorStateChanged;

  /// \brief Disable sensors if parent model's battery is drained
  /// Affects sensors that are in nested models
  public: bool disableOnDrainedBattery = false;

  /// \brief Mutex to protect access to sensorStateChanged
  public: std::mutex sensorStateMutex;
};


  /// \class CustomSensorPlugin Sensors.hh ignition/gazebo/systems/Sensors.hh
  /// \brief A system that manages sensors.
  ///
  /// ## System Parameters
  ///
  /// - `<render_engine>` Name of the render engine, such as 'ogre' or 'ogre2'.
  /// - `<background_color>` Color used for the scene's background. This
  /// will override the background color specified in a world's SDF <scene>
  /// element. This background color is used by sensors, not the GUI.
  /// - `<ambient_light>` Color used for the scene's ambient light. This
  /// will override the ambient value specified in a world's SDF <scene>
  /// element. This ambient light is used by sensors, not the GUI.
  /// - `<disable_on_drained_battery>` Disable sensors if the model's
  /// battery plugin charge reaches zero. Sensors that are in nested
  /// models are also affected.
  ///
  /// \TODO(louise) Have one system for all sensors, or one per
  /// sensor / sensor type?
  class CustomSensorPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit CustomSensorPlugin();

    /// \brief Destructor
    public: ~CustomSensorPlugin() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) final;

    // Documentation inherited
    public: void Update(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Create a rendering sensor from sdf
    /// \param[in] _entity Entity of the sensor
    /// \param[in] _sdf SDF description of the sensor
    /// \param[in] _parentName Name of parent that the sensor is attached to
    /// \return Sensor name
    private : std::string CreateSensor(const ignition::gazebo::Entity &_entity,
                                       const sdf::Sensor &_sdf,
                                       const std::string &_parentName);

    /// \brief Removes a rendering sensor
    /// \param[in] _entity Entity of the sensor
    private : void RemoveSensor(const ignition::gazebo::Entity &_entity);

    /// \brief Private data pointer.
    private: std::unique_ptr<CustomSensorPluginPrivate> dataPtr;
  };
}
#endif // CUSTOM_SENSOR_PLUGIN_HH_