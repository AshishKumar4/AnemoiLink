
#include "iostream"
#include "vector"

#include "../common.h"

#include <rpc/client.h>
#include "Sensors.hpp"

using namespace std;

// template <class dataType>
// RPCSensorBackend<dataType>::RPCSensorBackend(std::string ip, int portBase) : SensorBackend<dataType>()
// {
// 	this->rpcStub = new rpc::client(ip, portBase - 1);
// }

// template <class dataType>
// RPCSensorBackend<dataType>::RPCSensorBackend(rpc::client *rpcStub) : SensorBackend<dataType>()
// {
// 	this->rpcStub = rpcStub;
// }

// template <class dataType>
// dataType RPCSensorBackend<dataType>::fetchData(const char *func)
// {
// 	return rpcStub->call(func).as<dataType>();
// }

// template <class dataType>
// dataType RPCSensorBackend<dataType>::fetchData(const char *func, int arg1)
// {
// 	return rpcStub->call(func, arg1).as<dataType>();
// }

/********************************************************************************/

image_t DroneCamera_t::fetchData()
{
	return backDesc->fetchData("getCameraView", cam_id);
}

/********************************************************************************/

DroneIMU_t::DroneIMU_t(SensorBackend<data_imu_t> *backDesc) : DroneSensors_t(backDesc)
{
}

data_imu_t DroneIMU_t::fetchData()
{
	return backDesc->fetchData("getIMU");
}

/********************************************************************************/

DroneGPS_t::DroneGPS_t(SensorBackend<GeoPoint_t> *backDesc) : DroneSensors_t(backDesc)
{
}

GeoPoint_t DroneGPS_t::fetchData()
{
	return backDesc->fetchData("getLocation");
}

/********************************************************************************/

DroneAltimeter_t::DroneAltimeter_t(SensorBackend<float> *backDesc) : DroneSensors_t(backDesc)
{
}

float DroneAltimeter_t::fetchData()
{
	return backDesc->fetchData("getAltitude");
}

/********************************************************************************/

DroneVelometer_t::DroneVelometer_t(SensorBackend<vector3D_t> *backDesc) : DroneSensors_t(backDesc)
{
}

vector3D_t DroneVelometer_t::fetchData()
{
	return backDesc->fetchData("getVelocity");
}
/********************************************************************************/

DroneStateSensor_t::DroneStateSensor_t(SensorBackend<DroneState_t> *backDesc) : DroneSensors_t(backDesc)
{
}

DroneState_t DroneStateSensor_t::fetchData()
{
	return backDesc->fetchData("getCompleteState");
}

/********************************************************************************/
