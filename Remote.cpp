#include "iostream"
#include "vector"
#include "thread"

#include "ManualController.hpp"
#include "Drone.hpp"

using namespace std;

int event_keyPlus(ManualController *obj)
{
	*(obj->auxBuffers[AUX_3]) = 255;
	return *(obj->auxBuffers[1]);
}

int event_keyMinus(ManualController *obj)
{
	*(obj->auxBuffers[AUX_3]) = 50;
	return *(obj->auxBuffers[1]);
}

int event_key_p(ManualController *obj)
{
	*(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][0];
	*(obj->auxBuffers[AUX_3]) = 127;
	return 1;
}

int event_key_i(ManualController *obj)
{
	*(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][1];
	*(obj->auxBuffers[AUX_3]) = 127;
	return 2;
}

int event_key_d(ManualController *obj)
{
	*(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][2];
	*(obj->auxBuffers[AUX_3]) = 127;
	return 2;
}

int event_key_1(ManualController *obj)
{
	//*(obj->auxBuffers[AUX_1]) = 39; //1150;
	channel_select = 0; // Do Nothing --> 1000
	*(obj->auxBuffers[AUX_2]) = 0;
	*(obj->auxBuffers[AUX_3]) = 127;
	return 1;
}

int event_key_2(ManualController *obj)
{
	//*(obj->auxBuffers[AUX_1]) = 64; //1250;
	channel_select = 1; // Do nothing --> 1349
	*(obj->auxBuffers[AUX_2]) = 0;
	*(obj->auxBuffers[AUX_3]) = 127;
	return 2;
}

int event_key_3(ManualController *obj)
{
	//*(obj->auxBuffers[AUX_1]) = 90; //1350;
	channel_select = 2;
	*(obj->auxBuffers[AUX_2]) = 0;
	*(obj->auxBuffers[AUX_3]) = 127;
	return 2;
}

int event_key_q(ManualController *obj)
{
	if (show_debug)
		show_debug = 0;
	else
		show_debug = 1;
	return 1;
}

int event_key_w(ManualController *obj)
{
	// We need to show PID Status
	return 1;
}

int event_key_e(ManualController *obj)
{
	// We need to show IMU Status
	return 1;
}

int event_key_c(ManualController *obj)
{
	// Calibrate
	obj->StopExecutor();
	obj->CalibrateController();
	obj->ResumeExecutor();
	return 1;
}

int event_key_a(ManualController *obj)
{
	*(obj->auxBuffers[AUX_1]) = 250;
	return 1;
}

int event_key_s(ManualController *obj)
{
	*(obj->auxBuffers[AUX_1]) = 0;
	return 1;
}

int beaconStatus = 0;
int event_key_b(ManualController *obj) // Start/Stop Beacon
{
	if (!beaconStatus)
	{
		beaconStatus = 1;
		obj->uavObject->beaconLock = 1;
		//obj->uavObject->beaconLock.lock();
		printf("\nBeacon Locked!");
	}
	else
	{
		beaconStatus = 0;
		obj->uavObject->beaconLock = 0;
		//obj->uavObject->beaconLock.unlock();
		printf("\nBeacon unlocked!");
	}
	return 1;
}

int event_key_h(ManualController *obj)
{
	printf("\nRAPI CALLED!!!");
	//obj->uavObject->callRAPI(111, 0);
	float x, y, z;
	char axis;
	axis = getc(stdin);
	std::cin >> x >> y >> z;
	switch (axis)
	{
	case 'R': // Rotational, get 3 values for roll,pitch,yaw

		obj->uavObject->setHeading(z);
		obj->uavObject->setRollAngle(y);
		obj->uavObject->setPitchAngle(x);
		break;
	case 'P': // Positional, get 3 values
		obj->uavObject->gotoLocation(x, y, z);
		break;
	case 'A': // Altitude
		obj->uavObject->setAltitude(z);
		printf("[%f]", z);
		break;
	default:
		printf("\nNot understood %c,%d", axis, axis);
		return 0;
	}
	return 1;
}

int event_key_S(ManualController *obj)
{
	printf("\nRAPI, TOGGLE Auto Actuators!");
	char type;
	std::cin >> type;
	// Three types of Auto Actuators -> Rotational (R), Velocity (V), Positional (P)
	obj->uavObject->toggleAutoActuator(type);
	printf("\nDone...");
	return 1;
}

int event_other(ManualController *obj)
{
	*(obj->auxBuffers[0]) = 0;
	*(obj->auxBuffers[1]) = 0;
	*(obj->auxBuffers[2]) = 0;
	return 1;
}

int event_key_enter(ManualController *obj)
{
	return 1;
}

func_t KeyMap[256];

int KeyBindings_thread(ManualController *obj)
{
	while (1)
	{
		try
		{
			char key = getc(stdin);
			//printf("\t\t>>> [%d] <<<", (int)key);
			KeyMap[int(key)](obj);
		}
		catch (exception &e)
		{
			continue;
		}
	}
}

int main(int argc, char **argv)
{
	char *serialport = (char*)"/dev/ttyUSB0";
	Drone *droneControl;
	if (argc == 1)
		droneControl = new Drone("0.0.0.0");
	else if (argc == 2)
		droneControl = new Drone(argv[1]);
	else if (argc == 3)
		droneControl = new Drone(argv[1], atoi(argv[2]));
	else if (argc == 4)
	{
		droneControl = new Drone(argv[1], atoi(argv[2]));
		serialport = argv[3];
	}
	if(droneControl == nullptr || droneControl->connectionStatus)
	{
		cout<<"\nConnection to the UAV could not be established, Terminating...";
		exit(1);
	}

	/*
        Firstly install keybindings
    */
	for (int i = 0; i < 255; i++)
		KeyMap[i] = event_other;

	/*
        1,2,3 to select Pitch, Roll or Yaw; p, i, d to select PID param of that channel, +, - to increase/decrease values 
        q -> hide/show RC Data 
        w -> hide/show PID data 
        e -> hide/show IMU data 
        s -> disarm 
        a -> arm
    */
	KeyMap['+'] = event_keyPlus;
	KeyMap['-'] = event_keyMinus;
	KeyMap['q'] = event_key_q;
	KeyMap['w'] = event_key_w;
	KeyMap['p'] = event_key_p;
	KeyMap['i'] = event_key_i;
	KeyMap['d'] = event_key_d;
	KeyMap['1'] = event_key_1;
	KeyMap['2'] = event_key_2;
	KeyMap['3'] = event_key_3;
	KeyMap['c'] = event_key_c;
	KeyMap['a'] = event_key_a;
	KeyMap['s'] = event_key_s;
	KeyMap['h'] = event_key_h;
	KeyMap['b'] = event_key_b;
	KeyMap['S'] = event_key_S;
	KeyMap['\n'] = event_key_enter;
	KeyMap['\r'] = event_key_enter;
	ManualController remote(droneControl, serialport);
	thread KeyBindings(KeyBindings_thread, &remote);
	//ManualController remote(droneControl, "/dev/ttyUSB0");
	remote.ExecutorSerial();
	KeyBindings.join();
	return 0;
}
