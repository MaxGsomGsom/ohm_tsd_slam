#include "ThreadMapping.h"
#include "SlamNode.h"

#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include <cmath>

namespace ohm_tsd_slam
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid, int r_count):
    ThreadSLAM(*grid),
    _r_count(r_count)
{
    for (int i = 0; i < _r_count; i++)
        _initialized.push_back(false);
}

ThreadMapping::~ThreadMapping()
{
  _thread->join();
}

bool ThreadMapping::initialized(int id)
{
  if (id > _r_count || id < 0)
      return true;
  bool var = false;
  _pushMutex.lock();
  var = _initialized[id];
  _pushMutex.unlock();
  return var;
}

void ThreadMapping::initPush(obvious::SensorPolar2D* sensor, int id)
{
  if(this->initialized(id))
    return;
  _pushMutex.lock();
  for(unsigned int i = 0; i < INIT_PSHS; i++)
    _grid.push(sensor);
  _initialized[id] = true;
  _pushMutex.unlock();
}

void ThreadMapping::eventLoop(void)
{
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    while(_stayActive && !_sensors.empty())
    {
      _pushMutex.lock();
      obvious::SensorPolar2D* sensor = _sensors.back();
      _sensors.pop_back();
      _pushMutex.unlock();

      _grid.push(sensor);
      _pushMutex.lock();
      //cout << "Queue size: " << _sensors.size() << endl;
      delete sensor;
      //_initialized = true;
      _pushMutex.unlock();
    }
  }
}

void ThreadMapping::queuePush(obvious::SensorPolar2D* sensor)
{
  _pushMutex.lock();
  obvious::SensorPolar2D* sensorLocal = new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(),
                                                                   sensor->getMaximumRange(), sensor->getMinimumRange(), sensor->getLowReflectivityRange());
  sensorLocal->setTransformation(sensor->getTransformation());
  sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
  sensorLocal->setStandardMask();
  _sensors.push_back(sensorLocal);
  _pushMutex.unlock();
  this->unblock();
}

} /* namespace */
