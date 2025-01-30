# This is a Lab project for the Computer Animation class at Uni-Bonn

In this project algorithms from the ["Project MultiLeap: Making Multiple Hand Tracking Sensors to Act Like One"](https://www.researchgate.net/publication/357257620_Project_MultiLeap_Making_Multiple_Hand_Tracking_Sensors_to_Act_Like_One)
paper by Tomáš Nováček and Marcel Jirina are implemented.

📝*Note*: Those algorithm implementations, though in theory can be used with real-time sensor readings, have been designed with simulated readings in mind.

⚠*Warning*: For now, calibration and hand fusion only works with the right or left hand, but not both.

## To create an executable for this project:
1. Create a build directory in the root folder
```
mkdir build
```
2. Build using cmake
```
cd ./build
cmake .. -G"MinGW Makefiles"
make
```
## Usage
### Execute from the "build" folder:
```
.\Lab.exe [-s <number_of_calibration samples>] [-i <input_file>] [-o <output_file>] [-t <number_of_frames_to_record>] [-r <reference_sensor>] [-f <recording_start_frame>]
```

`-n` - a positive integer indicating a number of hand samples to be taken for calibration

`-i` - a path to a binary input file from the "data" folder. If you want to create/generate your own input file, refer to the src/save_devices_data_structure.cpp and json files in the "data"

`-o` - a path to an output json file

<!-- `-t` - number of frames to be recorded after calibration -->

`-r` - id of the reference sensor. "device_id" field

`-s` - a frame from which a recording can start. If the calibration process takes too long, a recording may start with a delay

`-f` - a frame until which the fused hand is recorded

### To visualize the fused hand run the python script in the "src" folder:
```
py .\src\fused_hand_visualizer.py -i .\results\fused_hand.json
```

`-i` - a path to an input file generated in the previous step

##
*Created by Nikita Morev*

*Special thanks to [Aleksandr Pavlenko](https://github.com/SashaPavlenko) for providing the data loader code and recording the data.*
