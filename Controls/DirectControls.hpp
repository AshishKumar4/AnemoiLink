#ifndef DIRECT_CONTROLS_H
#define DIRECT_CONTROLS_H

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>

#define CHANNEL_COUNT 9

#include "../common.h"
#include <unistd.h>

class DirectController
{
	std::vector<int> server_fd;
	std::vector<int> socket_num;
	std::vector<struct sockaddr_in *> addresses;
	int channelBuffs[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

protected:
	std::thread *beaconThread;
	std::thread *beaconListenerThread;

	void sendCommand(int val, int channel);

public:
	int connectionStatus;
	//std::mutex                beaconLock;
	int beaconLock;

	void InitSequence();
	int ConnectChannel(std::string ip, int port, int channel);
	DirectController(std::string ip = "0.0.0.0", int portBase = 8400);
	~DirectController();
	void arm();
	void disarm();
	void balance();
	void altitudeHold();
	/*      APIs for channel Controls       */
	void cmd(int throttle, int yaw, int roll, int pitch, int aux1 = 0, int aux2 = 0, int aux3 = 0, int aux4 = 0);

	void setThrottle(int val);
	void setPitch(int val);
	void setYaw(int val);
	void setRoll(int val);
	void setAux(int channel, int val);
	static void beaconRefresh(DirectController *obj);
	static void beaconListener(DirectController *obj);

	void closeConnections()
	{
		for(auto i : server_fd)
		{
			close(i);
		}
	}

	void printChannels();
};

#endif