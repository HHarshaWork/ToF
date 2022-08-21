# people-detect-application sample code

## Overview

This project serves as an example for the capabilities enabled by the deeplearning sdk.

## Introduction

People Detect Application is a program that uses the APIs exposed in the deeplearning sdk to continuously track people's bodies and their heads and localise them with boxes drawn around them. The application can work in real-time with an depth camera sensor or a binary file recorded with the same sensor.

## Build options

Before attempting to build this project, 
* Ensure the aditof library is built in **Release** mode.
* Confirm installation of the [PeopleTracker](http://swdownloads.analog.com/cse/aditof/aware3d/3DSensor_People_Tracker-Rel1.1.0.exe) libraries with installation path set to **\<local path to ToF repo\>/examples/people-detect-without-pcl**
	
### Build in Visual Studio

* Open the PeopleDetectApplication solution file inside Visual Studio 
* Select Release configuration
* Right click on the project and select build
* "Done building project" message should be seen on the output window if there are no errors
* The exe will be generated in the **\<path to build folder\>/examples/people-detect-without-pcl/Release** folder

### Build in command line (using MS Build)

* Navigate to the folder containing the PeopleDetectApplication.sln file
* Run the following command to build the project
	```
	> msbuild PeopleDetectApplication.sln /p:Configuration=Release /t:Rebuild
	```
* The exe will be generated in the **\<path to build folder\>/examples/people-detect-without-pcl/Release** folder

## Run

### Command Line Options

```
Usage:
		PeopleDetectApplication.exe [OPTIONS] [ARGUMENTS]
		
		Arguments:
			-i camera | <file path>		Specify input format
			
		Options:
			-o  <filename>		Name of the output file if needed to be stored
			-n  <integer>		Number of frames the application needs to process. [default: -1]
			-th <integer>		Number of seconds to wait before tracking the person once in frame. [default: 30]
			-d  <float>		Distance threshold(in cm) beyond which to consider the people as non-static. [default: 30]
			-cm <0|1>		Capture Mode. The supported capture modes are MP (1024x1024, option 0) and QMP (512x512, option 1)
			-dm <0|1|2>		Display Mode. The supported display modes are IR (option 0), depth heatmap (option 1) and silhouette (option 2)
			-off <float>		Height offset(in cm) to be added to detections to compensate for height difference(if any) between camera and audio controller.
			-sp <port name>		Serial COM port to transmit head locations to. [default: COM7]
			-em <0|1>		Enable/disable metadata output. If set to 1 the detections will be logged to a csv file. [default: 0]
			-md <filename>		Name of metadata file to be saved if metadata output enabled. Has no effect if metadata output is disabled
```