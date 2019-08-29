#pragma once

#include "iostream"
#include "vector"

#include "../common.h"

#include <rpc/client.h>

using namespace std;

template <class dataType>
class SensorBackend
{
public:
	SensorBackend();

	virtual dataType fetchData(const char* func) = 0;
	virtual dataType fetchData(const char* func, int arg1) = 0;
};

template<class dataType>
class RPCSensorBackend : public SensorBackend<dataType>
{
	rpc::client *rpcStub;
public:
	RPCSensorBackend(std::string ip = "0.0.0.0", int portBase = 8400)
	{
		this->rpcStub = new rpc::client(ip, portBase-1);
	}

	RPCSensorBackend(rpc::client *rpcStub)
	{
		this->rpcStub = rpcStub;
	}

	dataType fetchData(const char* func)
	{
		return rpcStub->call(func).as<dataType>();
	}

	dataType fetchData(const char* func, int arg1)
	{
		return rpcStub->call(func, arg1).as<dataType>();
	}
};

template <class dataType>
class DroneSensors_t
{
public:
	SensorBackend<dataType>* backDesc;
	DroneSensors_t();
	DroneSensors_t(SensorBackend<dataType>* backDesc);

	virtual dataType fetchData() = 0;
};

class DroneCamera_t : public DroneSensors_t<image_t>
{
	int cam_id;
	int mode;

public:
	DroneCamera_t();
	DroneCamera_t(int id);
	int startCameraServer();

	image_t fetchData()
	{
		return backDesc->fetchData("getCameraView", cam_id);
	}
	image_t getDisparity(int cam1, int cam2);
};

class DroneIMU_t : public DroneSensors_t<data_imu_t>
{
public:
	DroneIMU_t();
	DroneIMU_t(SensorBackend<data_imu_t>* backDesc) : DroneSensors_t(backDesc)
	{
	}

	data_imu_t fetchData()
	{
		return backDesc->fetchData("getIMU");
	}
};

class DroneGPS_t : public DroneSensors_t<GeoPoint_t>
{
public:
	DroneGPS_t();
	DroneGPS_t(SensorBackend<GeoPoint_t>* backDesc) : DroneSensors_t(backDesc)
	{
	}
	GeoPoint_t fetchData()
	{
		return backDesc->fetchData("getLocation");
	}
};

class DroneAltimeter_t : public DroneSensors_t<float>
{
public:
	DroneAltimeter_t();
	DroneAltimeter_t(SensorBackend<float>* backDesc) : DroneSensors_t(backDesc)
	{
	}
	float fetchData()
	{
		return backDesc->fetchData("getAltitude");
	}
};


class DroneVelometer_t : public DroneSensors_t<vector3D_t>
{
public:
	DroneVelometer_t();
	DroneVelometer_t(SensorBackend<vector3D_t>* backDesc) : DroneSensors_t(backDesc)
	{
	}
	vector3D_t fetchData()
	{
		return backDesc->fetchData("getVelocity");
	}
};


class DroneStateSensor_t : public DroneSensors_t<DroneState_t>
{
public:
	DroneStateSensor_t();
	DroneStateSensor_t(SensorBackend<DroneState_t>* backDesc) : DroneSensors_t(backDesc)
	{
	}
	DroneState_t fetchData()
	{
		return backDesc->fetchData("getCompleteState");
	}
};
