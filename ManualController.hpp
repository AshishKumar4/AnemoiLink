#pragma once

#ifndef MANUALCONTROLLER_H
#define MANUALCONTROLLER_H

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <fstream>
#include <thread> // std::thread

#include "Drone.hpp"
#include "Controls/DirectControls.hpp"

//#include "Indirect/SerialRX.h"
#include "Indirect/SerialRX.h"

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX_1 0
#define AUX_2 1
#define AUX_3 2
#define AUX_4 3

#define SHOW_RC_COMMAND

using namespace std;

/*
    This is Code for a physical Remote Controller for manual 
    control of the drone.
*/

class RunningAverage
{
private:
	vector<double> avgBuffs;
	int buffLength;
	double alpha;

public:
	RunningAverage(int blength, int seed, double alph);
	double ExpFilter(double valIn);
	void Reset(double alph);
};

class ManualController
{

	string yaw = "100";
	string pitch = "200";
	string throttle = "300";
	string roll = "400";
	string aux1 = "0";
	string aux2 = "0";

	vector<double> lfactors;
	vector<double> rfactors;
	vector<double> mids;
	vector<double> mins;
	vector<double> maxs;

	int t_val;
	int p_val;
	int r_val;
	int y_val;
	int a1_val = 0;
	int a2_val = 0;
	int a3_val = 0;
	int a4_val = 0;

	bool ExecutionHold = false;

	vector<RunningAverage *> channelFilters;

	SerialRX *serial;

protected:
public:
	int connectionStatus;
	Drone *uavObject;

	int *auxBuffers[4] = {&a1_val, &a2_val, &a3_val, &a4_val};

	//ManualController(int status, char *portName);
	ManualController(char *portName = (char*)"/dev/ttyUSB0", Drone *droneObj = NULL);

	int setUAV(Drone *droneObj)
	{
		uavObject = droneObj;
		if (droneObj == nullptr || droneObj->connectionStatus)
		{
			this->uavObject = nullptr;
			return 1;
		}
		return 0;
	}

	void CalibrateController();

	void ExecutorSerial();
	void parseSerialData_syncd(int sz, int scn_max);
	int filter(int val, int channel);
	void StopExecutor();
	void ResumeExecutor();
	void showChannels();
	void hideChannels();

	//int test123();
	//int testCode(std::string str);
};

#endif