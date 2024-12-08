# Sensor_fusion
This repo is a Max external wrapping [xioTechnologies Fusion](https://github.com/xioTechnologies/Fusion/tree/main?tab=readme-ov-file) library. In short, sensor fusion allows you to take raw IMU data and determine orientation and position. Fusion implements the [Madgwick algorithm](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf) which allows for up to 9DOF tracking by providing the orientation of the tracked object as well as the acceleration of the earth in relation to the tracked object.

# Usage
This external is designed to be used with Raw IMU data from devices such as game controllers and micro controllers within Max. This allows you to both tune the model in the case of micro controller, or use it on a device where you do not have access to the firmware like a game controller. 

# Installation 

## (Optional) Build Sensor Fusion
This requires cmake and a compiler that supports c++ 20
1. Clone this repo 
```
git clone https://github.com/composingcap/max-sensor-fusion.git
```
2. cd to the root of your clone of this repo
3. Configure and build using cmake 
```
mkdir build
cd build
cmake ..
cmake --build . --config release
```

## Installation into Max
Move this folder into .../Max9/Packages on your system.