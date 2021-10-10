# AirQualSniff

Have you ever wondered what you're breathing?

I have, and this project aims to answer that question--at least in part.

At it's core, it:

1. Gathers data from various instruments
2. Applies corrective factors and computes further useful data
3. Displays it all locally
4. Records and displays it all in the cloud

I can think to myself, "What the habeas gaseous?!" then look down at my battery of sensors and get an idea of the answer.

For various and mostly frivolous reasons, this project targets the Particle Photon microcontroller and a bevy of Sparkfun sensor breakout boards. More information on the bill of materials, wiring topology, and hardware design rationale can be found on the [Hackaday.io project page](https://hackaday.io/project/181918-airqualsniff).

## Building

### Particle Workbench

* Install VS Code
* Install Particle Workbench
* Install environment 3.1.0
* Run `particle install <module>` for each module in `project.properties` (Note that they are masked out in .gitignore)
* Cloud or local build to your heart's content

### Cloud Compile

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`

## Source File Structure

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for AirQualSniff.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h`, `.cpp` & `library.properties` files for your library there. Read the [Firmware Libraries guide](https://docs.particle.io/guide/tools-and-features/libraries/) for more details on how to develop libraries. Note that all contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

### Modules

#### Atmospherics

Implementation of various useful gas chemistry operations useful for atmospheric analysis, like relative humidity to absolute humidity computation and pressure to altitude estimation.

#### Decimator

Library for storing and decimating time series data. Intended to be easy to read for graphing.

#### Delta Clock

Implementation of an Arduino-friendly DeltaClock, an efficient scheduling data structure and algorithm I learned at university.

#### jkunkee-sps30

Arduino-friendly driver for the SPS30 particulate matter sensor.

#### others

The Sparkfun SGP30, u8g2, and ssd1327 libraries are third-party libraries that are not available through the Particle installer.

ssd1327 has been patched with [a fix for a problem in its address calculation logic](https://github.com/bitbank2/ssd1327/pull/8).

## Code Structure

### Data flow

Each sensor produces data at some frequency. Some sensors require inputs from other sensors to provide more accurate values.

These data are consumed a few different ways:

1. Local display of the most recent value
2. Local display of recent history
3. Periodic summaries for remote storage and display

This suggests an overall event-driven architecture might work well.

## Cloud Infrastructure

### Fundamentals

### Data Flow

### Data Display
