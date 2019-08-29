#include <iostream>
#include <bits/stdc++.h>

#include <rpc/client.h>

#include "Drone.hpp"

#include "Controls/DirectControls.hpp"

Drone::Drone(std::string ip, int portBase) : DirectController(ip, portBase)
{
	if (this->connectionStatus)
	{
		std::cout << "\nCould not connect to the UAV";
		return;
	}
	std::cout << "Initiating Drone Connection on IP " << ip << ", May the force be with you\n";
	// PortBase - 1 port would be for RPC channel
	rpcStub = new rpc::client(ip, portBase - 1);

	imu = new DroneIMU_t(new RPCSensorBackend<data_imu_t>(rpcStub));
	gps = new DroneGPS_t(new RPCSensorBackend<GeoPoint_t>(rpcStub));
	altimeter = new DroneAltimeter_t(new RPCSensorBackend<float>(rpcStub));
	velometer = new DroneVelometer_t(new RPCSensorBackend<vector3D_t>(rpcStub));
	stateSensor = new DroneStateSensor_t(new RPCSensorBackend<DroneState_t>(rpcStub));
}

int Drone::setHeading(float heading)
{
	printf("%f", heading);
	return rpcStub->call("setHeading", heading).as<int>();
}

int Drone::setRollAngle(float angle)
{
	printf("%f", angle);
	return rpcStub->call("setRollAngle", angle).as<int>();
}

int Drone::setPitchAngle(float angle)
{
	printf("%f", angle);
	return rpcStub->call("setPitchAngle", angle).as<int>();
}

int Drone::setAltitude(float altitude)
{
	printf("%f", altitude);
	return rpcStub->call("setAltitude", altitude).as<int>();
}

int Drone::toggleAutoActuator(char type)
{
	return rpcStub->call("toggleAutoActuator", type).as<int>();
}

int Drone::gotoLocation(float x, float y, float z)
{
	return rpcStub->call("gotoLocation", x, y, z).as<int>();
}

GeoPoint_t Drone::getLocation()
{
	return gps->fetchData();
}

DroneState_t Drone::getState()
{
	return stateSensor->fetchData();//DroneState_t(imu->fetchData(), gp;
}

float Drone::getTargetDistance()
{
	return NULL;
}

vector3D_t Drone::getVelocity()
{
	return velometer->fetchData();
}

vector3D_t Drone::getGyro()
{
	return imu->fetchData().gyro;
}

vector3D_t Drone::getAcc()
{
	return imu->fetchData().acc;
}

vector3D_t Drone::getMag()
{
	return imu->fetchData().mag;
}

float Drone::getAltitude()
{
	return altimeter->fetchData();
}

float Drone::getHeading()
{
	return stateSensor->fetchData().heading;
}

image_t Drone::getCameraView(int id)
{
	return cameras[id]->fetchData();
}

image_t Drone::getCameraView(DroneCamera_t *camera)
{
	return camera->fetchData();
}