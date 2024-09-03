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

#include "CustomSensorPlugin.hh"



// using namespace gz;
// using namespace gz::sim;
// using namespace systems;

using namespace simulation;


//////////////////////////////////////////////////
void CustomSensorPluginPrivate::WaitForInit()
{
  while (!this->initialized && this->running)
  {
    igndbg << "Waiting for init" << std::endl;
    std::unique_lock<std::mutex> lock(this->renderMutex);
    // Wait to be ready for initialization or stopped running.
    // We need rendering sensors to be available to initialize.
    this->renderCv.wait(lock, [this]()
    {
      return this->doInit || !this->running;
    });

    if (this->doInit)
    {
      // Only initialize if there are rendering sensors
      igndbg << "Initializing render context" << std::endl;
      if (this->backgroundColor)
        this->renderUtil.SetBackgroundColor(*this->backgroundColor);
      if (this->ambientLight)
        this->renderUtil.SetAmbientLight(*this->ambientLight);
      this->renderUtil.Init();
      this->scene = this->renderUtil.Scene();
      this->scene->SetCameraPassCountPerGpuFlush(6u);
      this->initialized = true;
    }

    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
  igndbg << "Rendering Thread initialized" << std::endl;
}

//////////////////////////////////////////////////
void CustomSensorPluginPrivate::RunOnce()
{
  {
    std::unique_lock<std::mutex> cvLock(this->renderMutex);
    this->renderCv.wait_for(cvLock, std::chrono::microseconds(1000), [this]()
    {
      return !this->running || this->updateAvailable;
    });
  }

  if (!this->updateAvailable)
    return;

  if (!this->running)
    return;

  if (!this->scene)
    return;

  IGN_PROFILE("CustomSensorPluginPrivate::RunOnce");
  {
    IGN_PROFILE("Update");
    std::unique_lock<std::mutex> timeLock(this->renderUtilMutex);
    this->renderUtil.Update();
    this->updateTimeApplied = this->updateTime;
    this->updateTimeCv.notify_one();
  }

  bool activeSensorsEmpty = true;
  {
    std::unique_lock<std::mutex> lk(this->sensorsMutex);
    activeSensorsEmpty = this->activeSensors.empty();
  }

  if (!activeSensorsEmpty || this->forceUpdate)
  {
    // disable sensors that are out of battery or re-enable sensors that are
    // being charged
    if (this->disableOnDrainedBattery)
    {
      std::unique_lock<std::mutex> lock2(this->sensorStateMutex);
      for (const auto &sensorIt : this->sensorStateChanged)
      {
        ignition::sensors::Sensor *s =
            this->sensorManager.Sensor(sensorIt.first);
        if (s)
        {
          s->SetActive(sensorIt.second);
        }
      }
      this->sensorStateChanged.clear();
    }

    {
      IGN_PROFILE("PreRender");
      this->eventManager->Emit<ignition::gazebo::events::PreRender>();
      this->scene->SetTime(this->updateTimeApplied);
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor
      this->scene->PreRender();
    }

    // disable sensors that have no subscribers to prevent doing unnecessary
    // work
    std::unordered_set<ignition::sensors::RenderingSensor *> tmpDisabledSensors;
    this->sensorsMutex.lock();
    for (auto id : this->sensorIds)
    {
      ignition::sensors::Sensor *s = this->sensorManager.Sensor(id);
      auto rs = dynamic_cast<ignition::sensors::RenderingSensor *>(s);
      if (rs->IsActive() && !this->HasConnections(rs))
      {
        rs->SetActive(false);
        tmpDisabledSensors.insert(rs);
      }
    }
    this->sensorsMutex.unlock();

    // safety check to see if reset occurred while we're rendering
    // avoid publishing outdated data if reset occurred
    std::unique_lock<std::mutex> timeLock(this->renderMutex);
    if (this->updateTimeApplied <= this->updateTime)
    {
      // publish data
      IGN_PROFILE("RunOnce");
      this->sensorManager.RunOnce(this->updateTimeApplied);
    }

    // re-enble sensors
    for (auto &rs : tmpDisabledSensors)
    {
      rs->SetActive(true);
    }

    {
      IGN_PROFILE("PostRender");
      // Update the scene graph manually to improve performance
      // We only need to do this once per frame It is important to call
      // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
      // so we don't waste cycles doing one scene graph update per sensor
      this->scene->PostRender();
      this->eventManager->Emit<ignition::gazebo::events::PostRender>();
    }

    std::unique_lock<std::mutex> lk(this->sensorsMutex);
    this->activeSensors.clear();
  }

  this->forceUpdate = false;
  {
    std::unique_lock<std::mutex> cvLock(this->renderMutex);
    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
}

//////////////////////////////////////////////////
void CustomSensorPluginPrivate::RenderThread()
{
  IGN_PROFILE_THREAD_NAME("RenderThread");

  igndbg << "CustomSensorPluginPrivate::RenderThread started" << std::endl;

  // We have to wait for rendering sensors to be available
  this->WaitForInit();

  while (this->running)
  {
    this->RunOnce();
  }

  // clean up before exiting
  for (const auto id : this->sensorIds)
    this->sensorManager.Remove(id);

  igndbg << "CustomSensorPluginPrivate::RenderThread stopped" << std::endl;
}

//////////////////////////////////////////////////
void CustomSensorPluginPrivate::Run()
{
  igndbg << "CustomSensorPluginPrivate::Run" << std::endl;
  this->running = true;
  this->renderThread = std::thread(&CustomSensorPluginPrivate::RenderThread, this);
}

//////////////////////////////////////////////////
void CustomSensorPluginPrivate::Stop()
{
  igndbg << "CustomSensorPluginPrivate::Stop" << std::endl;
  std::unique_lock<std::mutex> lock(this->renderMutex);
  this->running = false;

  if (this->stopConn)
  {
    // Clear connection to stop additional incoming events.
    this->stopConn.reset();
  }

  lock.unlock();
  this->renderCv.notify_all();

  if (this->renderThread.joinable())
  {
    this->renderThread.join();
  }
}

//////////////////////////////////////////////////
void CustomSensorPlugin::RemoveSensor(const ignition::gazebo::Entity &_entity)
{
  auto idIter = this->dataPtr->entityToIdMap.find(_entity);
  if (idIter != this->dataPtr->entityToIdMap.end())
  {
    // Remove from active sensors as well
    // Locking mutex to make sure the vector is not being changed while
    // the rendering thread is iterating over it
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->sensorsMutex);
      this->dataPtr->activeSensors.erase(idIter->second);
      this->dataPtr->sensorsToUpdate.erase(idIter->second);
    }

    // update cameras list
    for (auto &it : this->dataPtr->cameras)
    {
      if (it.second->Id() == idIter->second)
      {
        this->dataPtr->cameras.erase(it.first);
        break;
      }
    }

    this->dataPtr->sensorIds.erase(idIter->second);
    this->dataPtr->sensorManager.Remove(idIter->second);
    this->dataPtr->entityToIdMap.erase(idIter);
  }
}

//////////////////////////////////////////////////
CustomSensorPlugin::CustomSensorPlugin() : System(), dataPtr(std::make_unique<CustomSensorPluginPrivate>())
{
}

//////////////////////////////////////////////////
CustomSensorPlugin::~CustomSensorPlugin()
{
  this->dataPtr->Stop();
}

//////////////////////////////////////////////////
void CustomSensorPlugin::Configure(const ignition::gazebo::Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
  igndbg << "Configuring Sensors system" << std::endl;

  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

  // get whether or not to disable sensor when model battery is drained
  this->dataPtr->disableOnDrainedBattery =
      _sdf->Get<bool>("disable_on_drained_battery",
     this->dataPtr-> disableOnDrainedBattery).first;

  // Get the background color, if specified.
  if (_sdf->HasElement("background_color"))
    this->dataPtr->backgroundColor = _sdf->Get<ignition::math::Color>("background_color");

  // Get the ambient light, if specified.
  if (_sdf->HasElement("ambient_light"))
    this->dataPtr->ambientLight = _sdf->Get<ignition::math::Color>("ambient_light");

  this->dataPtr->renderUtil.SetEngineName(engineName);
  this->dataPtr->renderUtil.SetEnableSensors(true,
      std::bind(&CustomSensorPlugin::CreateSensor, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  this->dataPtr->renderUtil.SetRemoveSensorCb(
      std::bind(&CustomSensorPlugin::RemoveSensor, this, std::placeholders::_1));
  this->dataPtr->renderUtil.SetEventManager(&_eventMgr);

  // parse sensor-specific data
  auto worldEntity = _ecm.EntityByComponents(ignition::gazebo::components::World());
  if (ignition::gazebo::kNullEntity != worldEntity)
  {
    // temperature used by thermal camera
    auto atmosphere = _ecm.Component<ignition::gazebo::components::Atmosphere>(worldEntity);
    if (atmosphere)
    {
      auto atmosphereSdf = atmosphere->Data();
      this->dataPtr->ambientTemperature = atmosphereSdf.Temperature().Kelvin();
      this->dataPtr->ambientTemperatureGradient =
          atmosphereSdf.TemperatureGradient();
    }

    // Set render engine if specified from command line
    auto renderEngineServerComp =
      _ecm.Component<ignition::gazebo::components::RenderEngineServerPlugin>(worldEntity);
    if (renderEngineServerComp && !renderEngineServerComp->Data().empty())
    {
      this->dataPtr->renderUtil.SetEngineName(renderEngineServerComp->Data());
    }

    // Set headless mode if specified from command line
    auto renderEngineServerHeadlessComp =
      _ecm.Component<ignition::gazebo::components::RenderEngineServerHeadless>(worldEntity);
    if (renderEngineServerHeadlessComp)
    {
      this->dataPtr->renderUtil.SetHeadlessRendering(
        renderEngineServerHeadlessComp->Data());
    }
  }

  this->dataPtr->eventManager = &_eventMgr;

  this->dataPtr->stopConn = _eventMgr.Connect<ignition::gazebo::events::Stop>(
      std::bind(&CustomSensorPluginPrivate::Stop, this->dataPtr.get()));

  // Kick off worker thread
  this->dataPtr->Run();
}

//////////////////////////////////////////////////
void CustomSensorPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                     ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("CustomSensorPlugin::Update");
  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void CustomSensorPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                         const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("CustomSensorPlugin::PostUpdate");
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  {
    if (!this->dataPtr->initialized &&
        (this->dataPtr->forceUpdate ||
         _ecm.HasComponentType(ignition::gazebo::components::Camera::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::DepthCamera::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::GpuLidar::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::RgbdCamera::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::ThermalCamera::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::SegmentationCamera::typeId) ||
         _ecm.HasComponentType(ignition::gazebo::components::BoundingBoxCamera::typeId)))
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
      igndbg << "Initialization needed" << std::endl;
      this->dataPtr->doInit = true;
      this->dataPtr->renderCv.notify_one();
    }
  }

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    {
      IGN_PROFILE("UpdateFromECM");
      // Make sure we do not override the state in renderUtil if there are
      // still ECM changes that still need to be propagated to the scene,
      // i.e. wait until renderUtil.Update(), has taken place in the
      // rendering thread
      std::unique_lock<std::mutex> lock(this->dataPtr->renderUtilMutex);
      this->dataPtr->updateTimeCv.wait(lock, [this]()
      {
        return !this->dataPtr->updateAvailable ||
               (this->dataPtr->updateTimeToApply ==
               this->dataPtr->updateTimeApplied);
      });

      this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
      this->dataPtr->updateTime = _info.simTime;
    }

    // check connections to render events
    // we will need to perform render updates if there are event subscribers
    // \todo(anyone) This currently forces scene tree updates at the sim update
    // rate which can be too frequent and causes a performance hit.
    // We should look into throttling render updates
    bool hasRenderConnections =
      (this->dataPtr->eventManager->ConnectionCount<ignition::gazebo::events::PreRender>() > 0u ||
      this->dataPtr->eventManager->ConnectionCount<ignition::gazebo::events::PostRender>() > 0u);

    // if nextUpdateTime is max, it probably means there are previously
    // no active sensors or sensors with connections.
    // In this case, check if sensors have connections now. If so, we need to
    // set the nextUpdateTime
    if (this->dataPtr->nextUpdateTime ==
        std::chrono::steady_clock::duration::max() &&
        this->dataPtr->SensorsHaveConnections())
    {
      this->dataPtr->nextUpdateTime = this->dataPtr->NextUpdateTime(
          this->dataPtr->sensorsToUpdate, _info.simTime);
    }

    // notify the render thread if updates are available
    if (hasRenderConnections ||
        this->dataPtr->nextUpdateTime <= _info.simTime ||
        this->dataPtr->renderUtil.PendingSensors() > 0 ||
        this->dataPtr->forceUpdate)
    {
      if (this->dataPtr->disableOnDrainedBattery)
        this->dataPtr->UpdateBatteryState(_ecm);

      {
        std::unique_lock<std::mutex> cvLock(this->dataPtr->renderMutex);
        this->dataPtr->renderCv.wait(cvLock, [this] {
          return !this->dataPtr->running || !this->dataPtr->updateAvailable; });
      }
      if (!this->dataPtr->running)
      {
        return;
      }

      {
        std::unique_lock<std::mutex> lockSensors(this->dataPtr->sensorsMutex);
        this->dataPtr->activeSensors =
            std::move(this->dataPtr->sensorsToUpdate);
      }

      this->dataPtr->nextUpdateTime = this->dataPtr->NextUpdateTime(
          this->dataPtr->sensorsToUpdate, _info.simTime);

      // Force scene tree update if there are sensors to be created or
      // subscribes to the render events. This does not necessary force
      // sensors to update. Only active sensors will be updated
      this->dataPtr->forceUpdate =
          (this->dataPtr->renderUtil.PendingSensors() > 0) ||
          hasRenderConnections;

      {
        std::unique_lock<std::mutex> timeLock(this->dataPtr->renderUtilMutex);
        this->dataPtr->updateTimeToApply = this->dataPtr->updateTime;
      }
      {
        std::unique_lock<std::mutex> cvLock(this->dataPtr->renderMutex);
        this->dataPtr->updateAvailable = true;
        this->dataPtr->renderCv.notify_one();
      }
    }
  }
}

//////////////////////////////////////////////////
void CustomSensorPluginPrivate::UpdateBatteryState(const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Battery state
  _ecm.Each<ignition::gazebo::components::BatterySoC>(
      [&](const ignition::gazebo::Entity & _entity, const ignition::gazebo::components::BatterySoC *_bat)
      {
        bool hasCharge = _bat->Data() > 0;
        auto stateIt =
          this->modelBatteryState.find(_ecm.ParentEntity(_entity));
        if (stateIt != this->modelBatteryState.end())
        {
          // detect a change in battery charge state
          if (stateIt->second != hasCharge)
          {
            this->modelBatteryStateChanged[_ecm.ParentEntity(_entity)] =
                hasCharge;
          }
        }
        this->modelBatteryState[_ecm.ParentEntity(_entity)] = hasCharge;
        return true;
      });

  // disable sensor if parent model is out of battery or re-enable sensor
  // if battery is charging
  for (const auto & modelIt : this->modelBatteryStateChanged)
  {
    // check if sensor is part of this model
    for (const auto & sensorIt : this->entityToIdMap)
    {
      // parent link
      auto parentLinkComp =
          _ecm.Component<ignition::gazebo::components::ParentEntity>(sensorIt.first);
      if (!parentLinkComp)
        continue;

      // parent model
      auto parentModelComp = _ecm.Component<ignition::gazebo::components::ParentEntity>(
          parentLinkComp->Data());
      if (!parentModelComp)
        continue;

      // keep going up the tree in case sensor is in a nested model
      while (parentModelComp)
      {
        auto parentEnt = parentModelComp->Data();
        if (parentEnt == modelIt.first)
        {
          std::unique_lock<std::mutex> lock(this->sensorStateMutex);
          // sensor is part of model - update its active state
          this->sensorStateChanged[sensorIt.second] = modelIt.second;
          break;
        }
        parentModelComp = _ecm.Component<ignition::gazebo::components::ParentEntity>(parentEnt);
      }
    }
  }
  this->modelBatteryStateChanged.clear();
}

//////////////////////////////////////////////////
std::string CustomSensorPlugin::CreateSensor(const ignition::gazebo::Entity &_entity,
    const sdf::Sensor &_sdf, const std::string &_parentName)
{
  if (_sdf.Type() == sdf::SensorType::NONE)
  {
    ignerr << "Unable to create sensor. SDF sensor type is NONE." << std::endl;
    return std::string();
  }

  // Create within ign-sensors
  ignition::sensors::Sensor *sensor{nullptr};
  if (_sdf.Type() == sdf::SensorType::CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::CameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::DEPTH_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::DepthCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::GPU_LIDAR)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::GpuLidarSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::RGBD_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::RgbdCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::THERMAL_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::ThermalCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::BOUNDINGBOX_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::BoundingBoxCameraSensor>(_sdf);
  }
  else if (_sdf.Type() == sdf::SensorType::SEGMENTATION_CAMERA)
  {
    sensor = this->dataPtr->sensorManager.CreateSensor<
      ignition::sensors::SegmentationCameraSensor>(_sdf);
  }

  if (nullptr == sensor)
  {
    ignerr << "Failed to create sensor [" << _sdf.Name()
           << "]." << std::endl;
    return std::string();
  }

  // Store sensor ID
  auto sensorId = sensor->Id();
  this->dataPtr->entityToIdMap.insert({_entity, sensorId});
  this->dataPtr->sensorIds.insert(sensorId);

  // Set the scene so it can create the rendering sensor
  auto renderingSensor = dynamic_cast<ignition::sensors::RenderingSensor *>(sensor);
  renderingSensor->SetScene(this->dataPtr->scene);
  renderingSensor->SetParent(_parentName);
  renderingSensor->SetManualSceneUpdate(true);

  // Special case for stereo cameras
  auto cameraSensor = dynamic_cast<ignition::sensors::CameraSensor *>(sensor);
  if (nullptr != cameraSensor)
  {
    // Parent
    auto parent = cameraSensor->Parent();

    // If parent has other camera children, set the baseline.
    // For stereo pairs, the baseline for the left camera is zero, and for the
    // right camera it's the distance between them.
    // For more than 2 cameras, the first camera's baseline is zero and the
    // others have the distance between them.
    if (this->dataPtr->cameras.find(parent) !=
        this->dataPtr->cameras.end())
    {
      // TODO(anyone) This is safe because we're not removing sensors
      // First camera added to the parent link
      auto leftCamera = this->dataPtr->cameras[parent];
      auto rightCamera = cameraSensor;

      // If cameras have right / left topic, use that to decide which is which
      if (leftCamera->Topic().find("right") != std::string::npos &&
          rightCamera->Topic().find("left") != std::string::npos)
      {
        std::swap(rightCamera, leftCamera);
      }

      // Camera sensor's Y axis is orthogonal to the optical axis
      auto baseline = abs(rightCamera->Pose().Pos().Y() -
                          leftCamera->Pose().Pos().Y());
      rightCamera->SetBaseline(baseline);
    }
    else
    {
      this->dataPtr->cameras[parent] = cameraSensor;
    }
  }

  // Sensor-specific settings
  auto thermalSensor = dynamic_cast<ignition::sensors::ThermalCameraSensor *>(sensor);
  if (nullptr != thermalSensor)
  {
    thermalSensor->SetAmbientTemperature(this->dataPtr->ambientTemperature);

    // temperature gradient is in kelvin per meter - typically change in
    // temperature over change in altitude. However the implementation of
    // thermal sensor in ign-sensors varies temperature for all objects in its
    // view. So we will do an approximation based on camera view's vertical
    // distance.
    auto camSdf = _sdf.CameraSensor();
    double farClip = camSdf->FarClip();
    double angle = camSdf->HorizontalFov().Radian();
    double aspect = camSdf->ImageWidth() / camSdf->ImageHeight();
    double vfov = 2.0 * atan(tan(angle / 2.0) / aspect);
    double height = tan(vfov / 2.0) * farClip * 2.0;
    double tempRange =
        std::fabs(this->dataPtr->ambientTemperatureGradient * height);
    thermalSensor->SetAmbientTemperatureRange(tempRange);

    ignmsg << "Setting ambient temperature to "
           << this->dataPtr->ambientTemperature << " Kelvin and gradient to "
           << this->dataPtr->ambientTemperatureGradient << " K/m. "
           << "The resulting temperature range is: " << tempRange
           << " Kelvin." << std::endl;
  }

  return sensor->Name();
}

//////////////////////////////////////////////////
bool CustomSensorPluginPrivate::HasConnections(ignition::sensors::RenderingSensor *_sensor) const
{
  if (!_sensor)
    return true;

  // \todo(iche033) Remove this function once a virtual
  // ignition::sensors::RenderingSensor::HasConnections function is available
  {
    auto s = dynamic_cast<ignition::sensors::RgbdCameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::DepthCameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::GpuLidarSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::SegmentationCameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::BoundingBoxCameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::ThermalCameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  {
    auto s = dynamic_cast<ignition::sensors::CameraSensor *>(_sensor);
    if (s)
      return s->HasConnections();
  }
  ignwarn << "Unable to check connection count for sensor: " << _sensor->Name()
          << ". Unknown sensor type." << std::endl;
  return true;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration CustomSensorPluginPrivate::NextUpdateTime(
    std::set<ignition::sensors::SensorId> &_sensorsToUpdate,
    const std::chrono::steady_clock::duration &_currentTime)
{
  _sensorsToUpdate.clear();
  std::chrono::steady_clock::duration minNextUpdateTime =
      std::chrono::steady_clock::duration::max();
  for (auto id : this->sensorIds)
  {
    ignition::sensors::Sensor *s = this->sensorManager.Sensor(id);

    if (nullptr == s)
    {
      continue;
    }

    auto rs = dynamic_cast<ignition::sensors::RenderingSensor *>(s);

    if (nullptr == rs)
    {
      continue;
    }

    if (!this->HasConnections(rs))
    {
      continue;
    }

    std::chrono::steady_clock::duration time;
    // if sensor's next update tims is less or equal to current sim time then
    // it's in the process of being updated by the render loop
    // Set their next update time  to be current time + update period
    if (rs->NextDataUpdateTime() <= _currentTime)
    {
      time = rs->NextDataUpdateTime() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rs->UpdateRate()));
    }
    else
    {
      time = rs->NextDataUpdateTime();
    }

    if (time <= minNextUpdateTime)
    {
      _sensorsToUpdate.clear();
      minNextUpdateTime = time;
    }
    _sensorsToUpdate.insert(id);
  }
  return minNextUpdateTime;
}

//////////////////////////////////////////////////
bool CustomSensorPluginPrivate::SensorsHaveConnections()
{
  for (auto id : this->sensorIds)
  {
    ignition::sensors::Sensor *s = this->sensorManager.Sensor(id);
    if (nullptr == s)
    {
      continue;
    }

    auto rs = dynamic_cast<ignition::sensors::RenderingSensor *>(s);

    if (nullptr == rs)
    {
      continue;
    }

    if (this->HasConnections(rs))
    {
      return true;
    }
  }
  return false;
}

IGNITION_ADD_PLUGIN(CustomSensorPlugin, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
 ignition::gazebo::ISystemUpdate,
  ignition::gazebo::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(CustomSensorPlugin, "simulation::CustomSensorPlugin")

