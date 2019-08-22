
swig -c++ -python Drone.i
g++ -fPIC -c Drone_wrap.cxx -I/usr/include/python3.7m 
g++ -shared Drone_wrap.o ../librpc.so build/libDrone.so -o _Drone.so
