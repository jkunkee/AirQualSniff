# AirQualSniff

Have you ever wondered what you're breathing?

I have, and this project aims to answer that question--at least in part.

At it's core, it:

1. Gathers data from various instruments
2. Applies corrective factors and computes further useful data
3. Displays it all locally
4. Reports it to the cloud for storage and display

I can think to myself, "What the habeas gaseous?!" then look down at my battery of sensors and get an idea of the answer.

For various and mostly frivolous reasons, this project targets the Particle Photon microcontroller and a bevy of Sparkfun sensor breakout boards. More information on the bill of materials, wiring topology, and hardware design rationale can be found on the [Hackaday.io project page](https://hackaday.io/project/181918-airqualsniff).

## Building

### Particle Workbench

* Install VS Code
* Install Particle Workbench
* Install and configure project for environment 2.3.1 (the latest Photon long-term-support release)
* Run `particle install <module>` for each module in `project.properties` (Note that they are masked out in .gitignore)
   * From the Particle CLI at the root of this repo, this is as easy as
     `(Get-Content .\project.properties) -match 'dependencies.*' | % { (($_ -isplit '\.')[1] -isplit '=')[0] } | % { particle library copy $_ }`
* Cloud or local build to your heart's content

### Cloud Compile

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
   - This includes the large u8g2 font file.

## Source File Structure

This project follows the standard Particle.io build system's file structure conventions:

- `src` - main source files
- `src/<something>.ino` - main source file
- `lib` - reusable library files
- `project.properties` - project name and dependency specifiers (used for cloud build)

N.B. Typically, when adding a file to any of these folders, the Particle Workbench has to be restarted to see them.

### Checked-in Libraries

#### Atmospherics

Implementation of various useful gas chemistry operations useful for atmospheric analysis, like relative humidity to absolute humidity computation and pressure to altitude estimation.

#### Decimator

Library for storing and decimating time series data. Intended to be easy to read for graphing.

u8g2-based sparkline rendering is included since it didn't really fit anywhere else and turned out to be tightly coupled with the Decimator FIFO class.

#### Eventing

An event-driven code execution framework. This includes an Arduino-friendly DeltaClock implementation, an efficient scheduling data structure and algorithm I learned while at university.

#### jkunkee-sps30

Arduino-friendly driver for the SPS30 particulate matter sensor.

#### u8g2_boxen

Tools displaying numbers with units in boxes using the u8g2 graphics library.

#### others

The Sparkfun SGP30 and u8g2 libraries are third-party libraries that are not available through the Particle installer.

## Code Structure

Besides the standard Arduino `setup` and `loop` functions, the main `.ino` file is organized into namespaces:

- `infrastructure` - general infrastructure initialization
- `peripherals` - init and helpers for non-air-composition-sensor devices
- `sensors` - init and helpers for air composition sensor devices
- `Data` - sensor reading storage, decimation, and easy dissemination
- `UX` - renders data stored in `Data` into several forms (serial, GUI, cloud publish)
- `flow` - Creation of data structures for event-driven execution of all of the above

The convention is for `setup` to call the `init` function from each namespace in the order listed above (which is the order they appear in the file). This ensures that `flow` has access to fully initialized `sensors` to fill `Data` for `UX` to display in its myriad forms.

Getting WiFi initialized (or not) in the right sequence has proved to be a challenge and currently resides in `UX` even though it should be in `peripherals` or *maybe* `infrastructure`.

The `flow::init` function is responsible for actually scheduling and linking together all of the polling and processing functions in the other namespaces.

## Cloud Infrastructure

### Fundamentals

### Data Flow

### Data Display
