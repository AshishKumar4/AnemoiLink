#pragma once

#include <iostream>

#include <rpc/client.h>

#include "Controls/DirectControls.hpp"
#include "Sensors/Sensors.hpp"

class Drone : public DirectController
{
	rpc::client *rpcStub;

protected:
public:
	GeoPoint_t		*currentPosition;
	DroneState_t 	*currentState;
	std::vector<DroneCamera_t*> 		cameras;
	DroneIMU_t			*imu;
	DroneGPS_t 			*gps;
	DroneAltimeter_t 	*altimeter;
	DroneVelometer_t 	*velometer;
	DroneStateSensor_t 	*stateSensor;

	Drone(std::string ip = "0.0.0.0", int portBase = 8400);

	/*
        Provide High level Control APIs
    */
	int setHeading(float heading);
	int setRollAngle(float angle);
	int setPitchAngle(float angle);
	int setAltitude(float altitude);
	int toggleAutoActuator(char type);
	int gotoLocation(float x, float y, float z);
	int addWaypoint(float x, float y, float z);

	int enableAutoNav();
	int disableAutoNav();

	/* APIs to get Data */
	//int startSensorsServer();

	GeoPoint_t		getLocation();
	DroneState_t 	getState();
	float 			getTargetDistance();
	vector3D_t 		getVelocity();
	vector3D_t 		getGyro();
	vector3D_t		getAcc();
	vector3D_t 		getMag();
	float 			getAltitude();
	float 			getHeading();
	image_t 		getCameraView(int id);
	image_t 		getCameraView(DroneCamera_t *camera);

	int 			getCurrentPath();

	/*
        Provide High level Sensor Telemetry
   */
};