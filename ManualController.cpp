#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <fstream>
#include <thread> // std::thread
#include <algorithm>
#include <sys/stat.h>

#include "Drone.hpp"
#include "Controls/DirectControls.hpp"
//#include "../Controls/AirSimControls/AirSimControls.h"

#include "ManualController.hpp"

int show_channels = 0;

RunningAverage::RunningAverage(int blength, int seed, double alph)
{
	for (int j = 0; j < blength; j++)
		avgBuffs.push_back(seed);
	alpha = alph;
	buffLength = blength;
}

double RunningAverage::ExpFilter(double valIn)
{
	// an Exponential Moving Average
	double y = (alpha * valIn) + ((1 - alpha) * avgBuffs[0]);
	avgBuffs[0] = y;
	return y;
}

void RunningAverage::Reset(double alph)
{
	alpha = alph;
}

ManualController::ManualController(char* portName, Drone* droneObj)
{
	this->connectionStatus = 0;
	if(!strlen(portName))
	{
		++this->connectionStatus;
		return;
	}
	this->setUAV(droneObj);
	// TODO: Add code to Calibrate the Remote data, and add a filter
	serial = new SerialRX(portName);
	for (int i = 0; i < 6; i++)
	{
		channelFilters.push_back(new RunningAverage(3, 1000, 0.45));
	}

	// Load saved Calibration Data. If file not found, Calibrate it! FileName: calibration.dat
	// Check if calibration.dat exists -->
	std::string fname = "calibration.dat";
	struct stat buffer;
	if (stat(fname.c_str(), &buffer) == 0)
	{
		std::fstream fin(fname, std::fstream::in);
		double tmp = 0;
		for (int i = 0; i < 6; i++)
		{
			fin >> tmp;
			mins.push_back(tmp);
			//cout<<tmp<<" ";
			fin >> tmp;
			maxs.push_back(tmp);
			//cout<<tmp<<" ";
			fin >> tmp;
			mids.push_back(tmp);
			//cout<<tmp<<" ";
			fin >> tmp;
			lfactors.push_back(tmp);
			//cout<<tmp<<" ";
			fin >> tmp;
			rfactors.push_back(tmp);
			//cout<<tmp<<" ";
		}
		fin.close();
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			lfactors.push_back(0);
			rfactors.push_back(0);
			mins.push_back(0);
			maxs.push_back(0);
			mids.push_back(0);
		}
		// We need to Calibrate it
		CalibrateController();
	}
	for (int i = 0; i < 6; i++)
	{
		channelFilters[i]->Reset(0.5);
	}
	std::cout<<"\nCalibration Completed!";
}

// ManualController::ManualController(Drone* droneObj, char* portName)
// {
// 	this->setUAV(droneObj);
// }

// int ManualController::setUAV(Drone *droneObj)

void ManualController::CalibrateController()
{
	/************************************************************************************************************/
	/*     Creating Moving Average Buffers, min max mid lfactor and rfactor buffers     */
	for (int i = 0; i < 6; i++)
	{
		channelFilters[i]->Reset(0.45);
	}
	// When we are initializing, We set the alpha value for moving average filter as 0.5 for calibration, but we
	// would increase it to higher value for normal usage.
	/************************************************************************************************************/

	/************************************************************************************************************/
	/*     Gathering Sample data for Calibration     */

	cout << "\n\n***Calibration in progress***\n";
	// TODO: Instead of taking simple values, take in average
	cout << "\n\tPlease push the throttle and pitch to their minimum levels within 2 seconds";
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	parseSerialData_syncd(600, 15);
	//delay1(5);
	mins[THROTTLE] = t_val;
	mins[PITCH] = p_val;
	printf("\n{%d %d}", t_val, p_val);
	cout << "\n\t\tLets hope it's done well, Calibrated accordingly";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	cout << "\n\tPlease push the throttle and pitch to their maximum levels within 2 seconds";
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	parseSerialData_syncd(600, 15);
	//delay1(5);
	maxs[THROTTLE] = t_val;
	maxs[PITCH] = p_val;
	printf("\n{%d %d}", t_val, p_val);
	cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	cout << "\n\tPlease push the yaw and roll to their minimum levels within 2 seconds";
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	parseSerialData_syncd(600, 15);
	//delay1(5);
	mins[ROLL] = r_val;
	mins[YAW] = y_val;
	printf("\n{%d %d}", r_val, y_val);
	cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	cout << "\n\tPlease push the yaw and roll to their maximum levels within 2 seconds";
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	parseSerialData_syncd(600, 15);
	//delay1(5);
	maxs[ROLL] = r_val;
	maxs[YAW] = y_val;
	printf("\n{%d %d}", r_val, y_val);
	cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	cout << "\n\tPlease leave the throttle and place them in the middle positions within 2 seconds";
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	parseSerialData_syncd(600, 15);
	//delay1(5);
	mids[THROTTLE] = t_val;
	mids[PITCH] = p_val;
	mids[ROLL] = r_val;
	mids[YAW] = y_val;
	printf("\n{%d %d %d %d}", t_val, p_val, r_val, y_val);

	/* In case you just decided to switch your uavObject to be mapped in opposite ways -_- */
	for (int i = 0; i < 4; i++)
	{
		if (mins[i] > maxs[i])
		{
			// Swap them
			int tmp = mins[i];
			mins[i] = maxs[i];
			maxs[i] = tmp;
		}
	}

	/* Calibration Computation */
	for (int i = 0; i < 4; i++)
	{
		if (mids[i] - mins[i] != 0)
			lfactors[i] = (127.5 / double(mids[i] - mins[i]));
		//lfactors.push_back(127.5 / double(mids[i] - mins[i]));
		else
			lfactors[i] = (1);
		//lfactors.push_back(1);
		if (maxs[i] - mids[i] != 0)
			rfactors[i] = (127.5 / double(maxs[i] - mids[i]));
		//rfactors.push_back(127.5 / double(maxs[i] - mids[i]));
		else
			rfactors[i] = (1);
		//rfactors.push_back(1);
	}
	printf("\nSaving => ");
	// Save these settings
	std::fstream fin("calibration.dat", std::fstream::out);
	for (int i = 0; i < 6; i++)
	{
		fin << mins[i] << endl;
		fin << maxs[i] << endl;
		fin << mids[i] << endl;
		fin << lfactors[i] << endl;
		fin << rfactors[i] << endl;
		printf(".");
	}
	printf("\nSaved...");
	fin.close();

	cout << "\nCalibration Completed!!!";
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// After Calibration, We shall tune the Moving Average filters a bit...
	for (int i = 0; i < 6; i++)
	{
		channelFilters[i]->Reset(0.5);
	}

	//cout<<"\nRange: "<<t_range<<" "<<r_range<<" "<<y_range<<" "<<p_range<<"\n";
	//cout<<"\nFactors: "<<t_lfactor<<" "<<r_lfactor<<" "<<y_lfactor<<" "<<p_lfactor<<"\n";
	/************************************************************************************************************/
}

void ManualController::ExecutorSerial()
{
	if(!this->uavObject) return;
	int jj = 0;
	int sz = 90;
	int scn_max = 1; //(sz / 30); // We would discard sections of data from start and end, for sanity
	// Basically Take in values from the remote over Serial, Probably via an Arduino as middleware
	// and filter it and send it over to the API layer for Controller, to control the drone.

	//serial->openSerial();
	while (1)
	{
		if (ExecutionHold)
		{
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
			continue;
		}
		parseSerialData_syncd(sz, scn_max);

		uavObject->setThrottle(filter(t_val, THROTTLE)); // (double(t_val - t_min) * t_factor)));
		uavObject->setYaw(filter(y_val, YAW));			 //(double(y_val - y_min) * y_factor)));
		uavObject->setPitch(filter(p_val, PITCH));		 // Reversed Pitch     //(double(p_val - p_min) * p_factor)));
		uavObject->setRoll(255 - filter(r_val, ROLL));   // Reversed Roll       //(double(r_val - r_min) * r_factor)));
		uavObject->setAux(1, a1_val);
		uavObject->setAux(2, a2_val);
		uavObject->setAux(3, a3_val);
		uavObject->setAux(4, a4_val);
		/*uavObject->setAux3(a2_val);
            uavObject->setAux4(a2_val);*/
		//std::cout<<"Help!";
		if (show_channels)
			uavObject->printChannels();
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	//serial->closeSerial();
}

void ManualController::parseSerialData_syncd(int sz, int scn_max)
{
	try
	{
		string pparsed;
		stringstream input_stringstream(serial->getBuff_synced(sz));
		getline(input_stringstream, throttle, '\n'); // Discard the first entry
		int scn = 0;
		while (scn < scn_max && getline(input_stringstream, pparsed, '\n'))
		{
			try
			{
				// Still allow only those lines to influence which have complete sets of characters
				if (count(pparsed.begin(), pparsed.end(), ' ') == 5)
				{
					++scn;
					string buff(pparsed);
					//cout << " Got Data [" << pparsed << "]\n";
					stringstream input_stringstream(buff);
					getline(input_stringstream, throttle, ' ');
					getline(input_stringstream, yaw, ' ');
					getline(input_stringstream, pitch, ' ');
					getline(input_stringstream, roll, ' ');
					getline(input_stringstream, aux1, ' ');
					getline(input_stringstream, aux2, ' ');

					t_val = atoi(throttle.c_str());
					y_val = atoi(yaw.c_str());
					r_val = atoi(roll.c_str());
					p_val = atoi(pitch.c_str());
					/*a1_val = atoi(aux1.c_str());
                        a2_val = atoi(aux2.c_str());*/

					t_val = channelFilters[0]->ExpFilter(t_val);
					y_val = channelFilters[1]->ExpFilter(y_val);
					r_val = channelFilters[2]->ExpFilter(r_val);
					p_val = channelFilters[3]->ExpFilter(p_val);
				}
			}
			catch (exception &e)
			{
				cout << "Error! PANIC!!!!" << e.what();
				break;
			}
		}
	}
	catch (exception &e)
	{
		cout << "FUCK THIS ERROR! " << e.what();
	}
}

int ManualController::filter(int val, int channel)
{
	double vvv = double(val);
	/*if(vvv < lfactors[channel]) vvv = lfactors[channel];
        else if(vvv > rfactors[channel]) vvv = rfactors[channel];
        cout<<"["<<val<<"_";*/
	// Eq --> ((no-ni)/(bo-bi))*(a-bi) + ni; (no-ni)*(bo-bi) is our factor
	// We take into account the mid stick values
	if (vvv <= mids[channel])
	{
		vvv = ((vvv - mins[channel]) * lfactors[channel]); //mins[channel];
	}
	else
	{
		vvv = ((vvv - mids[channel]) * rfactors[channel]) + 127.5; // + mids[channel];
	}
	if (vvv < 0)
		vvv = 0;
	else if (vvv > 255)
		vvv = 255;
	return int(vvv);
}

void ManualController::StopExecutor()
{
	ExecutionHold = false;
}

void ManualController::ResumeExecutor()
{
	ExecutionHold = false;
}

void ManualController::showChannels()
{
	show_channels = true;
}

void ManualController::hideChannels()
{
	show_channels = false;
}

