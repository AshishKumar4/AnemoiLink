bash build.sh
g++ -L/opt/AnemoiGCS/Modules/BaseSystem/build build/libDrone.so Remote.cpp -o ManualController -lpthread -lrpc
chmod +x ManualController