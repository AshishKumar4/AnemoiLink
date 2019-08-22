bash build.sh
g++ -L/opt/AnemoiGCS/Modules/BaseSystem/build -lDrone Remote.cpp -o ManualController -lpthread -lrpc
chmod +x ManualController