# FRC-2020
[![Build Status](https://dev.azure.com/cttewari/FRC-2020/_apis/build/status/frc604.FRC-2020?branchName=master)](https://dev.azure.com/cttewari/FRC-2020/_build/latest?definitionId=1&branchName=master)

604 Quixilver's 2020 Robot Code

## IMPORTANT
#### (as of 3/01/2019)
**Please update the WPILib version on you computer, we cannot use older versions at competition!**

*If you want to be able to image the roboRIO, install the new Update Suite too if you are unsure and have a windows laptop, just install it.*

WPILib : [https://github.com/wpilibsuite/allwpilib/releases]

*Read the information in the github wiki about reinstalling VSCode you probably won't have to do it.*

Update Suite : [http://www.ni.com/download/first-robotics-software-2017/7904/en/]

*Side note : robotRIO firmware version v14 is now required for competitions, and this version of WPILib only works with it.*

## Setting up WPILib
Follow instructions here: [https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html] or install it directly from the repository here [https://github.com/wpilibsuite/allwpilib/releases/tag/v2021.3.1]
## Git

1. Download git for you operating system

 https://git-scm.com/downloads
 
2.Run the downloaded installer
  (On MacOS double click on the `.pkg` file on the prompt(you make have to go to `Settings -> Security & Privacy ->Open prompt at the bottom of page` to open it)

Here are the options you should use when setting it up.

#### Windows
  a. First click next to accept the license.
  
  b. Click next on the Destination Location.
  
  c. Click next on Select Components.
  
  d. Click next on Select Start Menu Folder
  
  e. Select Vim editor for git, then select next.
  
  f. Choose Git from the command line and also from 3rd-party software.
  
  g. Choose to use the OpenSSl library.
  
  h. Choose Checkout Windows-style, commit Unix-style line endings.
  
  i.Choose to use MinTTY.
  
  j.Then click Install.
  
#### MacOs
  a. Click continue
  
  b. Click Install
  
  c. Enter your administrator account username and password.
  
  d. Choose to keep the package.
  
## Robot Code in VSCode

a. To start, make a folder somewhere to store your robot code, then open terminal or the command line and navigate into the folder.

`cd "Destination of Folder`

Ex(Windows). `cd Desktop/FRC_2020`

ex(MacOs). `cd /Desktop/FRC_2020`

b. Now you need to clone the repository into the folder(like copy and pasting).

`git clone https://github.com/frc604/FRC-2020.git`

c. Now go into Visual Studio COde and click `File -> Open Folder` and click on the folder `FRC-2020` in the folder that you created.

## Pulling Certain Branch

a. `cd` into the the code repo, into the folder you made, then into FRC-2020

`cd "Folder Path/FRC-2020"`

b. Then type `git fetch` -v to list all the possible branches.

c. Finally type `git checkout "Name of Branch"` to 2switch to that branch.


## Structure
The main source code can be found in `src/com/_604robotics/`

Custom libraries (Pathfinder) can be found in `lib/`

## Using Gradle
The official build system for FIRST FRC Robotics has officially been changed to gradle.
This means that there is no need to be stuck with any IDE or editor, and grants more freedom.
That being said, it does require the usage of terminal/console/CMD commands.

### Deploying
To deploy code to the robot, two things must happen.

1. When connected to the *internet*, **not** the robot, run `./gradlew downloadAll`
This will download all the needed dependencies for gradle and the project itself.
2. Before deploying, the code must be built with `./gradlew build`
This will create Jars of any code changes
3. Then, when connected to the *robot*, run `./gradlew deploy --offline`
This will push all *previously built* code to the robot

### Setting up editor
Currently, there are two IDEs that are added into the plugin list in `build.gradle`:
* Intellij IDEA
* VSCode

The robot project file should open on either one of these editors, just select the `FRC-2020` folder.

VSCode may display and error, stating that
```
This project is not compatible with this version of the extension. Would you like to import this project into 2020?
```
This can be safely ignored.

## Unit Tests
The JUnit 5 units tests can be run through gradle with `./gradlew test`.

#### Individual Unit Tests
If you are using Intelij, the unit tests can be run individually with the prompts in the text editor.

When using VSCode and Java Test Runner this needs to be added to `.vscode/settings.json` to provide the hal dependency to the test
runner.
```
    "java.test.config": [
        {
            "name": "WPIlibUnitTests",
            "workingDirectory": "${workspaceFolder}/build/tmp/jniExtractDir",
            "vmargs": [ "-Djava.library.path=${workspaceFolder}/build/tmp/jniExtractDir" ],
            "env": { "LD_LIBRARY_PATH": "${workspaceFolder}/build/tmp/jniExtractDir" ,
              "DYLD_LIBRARY_PATH": "${workspaceFolder}/build/tmp/jniExtractDir" }
            
        },
      ],
    "java.test.defaultConfig": "WPIlibUnitTests"
```

## Dependencies
This year's robot code uses [604's modified version of Pathfinder](https://github.com/frc604/Pathfinder), the original version of which can be found [here](https://github.com/JacisNonsense/Pathfinder).

Uses Pixy code adapted from [BHSRobotix/Steamworks2017](https://github.com/BHSRobotix/Steamworks2017)

## Shuffleboard
1. First locate the FRC Shuffleboard shortcut on the desktop or the `shuffleboard.vbs` file.

2. Next open Shuffleboard and click `File -> Open Layout`

3. In the prompt locate your repository location and select the `shuffleboard_FRC-2020.json` file in the `ShuffleboardFiles` folder.

## Limelight
### Flashing
When flashing the Limelight, follow the instructions found [here](http://docs.limelightvision.io/en/latest/getting_started.html#imaging).
However, it is also necessary to install the drivers for a Raspberry Pi compute module as well.
You will know you have succeeded when the Pi shows up as a removable drive (if on Windows).

## Drivers
http://www.sapphiretech.com/product_downloadmore.asp?PID=1482&CataID=30&lang=eng
