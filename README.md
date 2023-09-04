# Lander Control

## Introduction
This repository contains all the control code for the lander's industrial computer (excluding the MATLAB code for the upper computer). Two control methods have been developed: command-line manual control and upper computer planning control. The former starts a command-line window on the industrial computer, where commands can be entered directly. The latter utilizes ROS topic communication/service communication to allow the industrial computer to receive control signals from the upper computer and execute them.

## Control System Architecture and Function Description
### Command-Line Control Method
* Initialization [init]:

After testing, it was found that the encoder readings of the four main motors would change when the 12 Elmo motors were power cycled, while the encoder readings of the eight auxiliary motors generally remain unchanged. The solution is to call the "getpos" function each time the system is powered on to obtain the motor position readings and add them to the "pos_offset" parameter in the XML configuration file.
It is necessary to call "getpos" first to check if the motor positions are correct before using "init".
Before shutting down after each debugging session, "init" must be called to reset the system, so that the positions can be checked correctly at the start of the next session.

Call format:
```
init (initialize all motors)
init -m=0 (initialize motor 0)
init -l=1/2/3/4 (initialize specific leg)
```
* Single Motor Movement [mvm]:
  
Used to move individual motors.
For auxiliary motors, the unit is degrees. Set the "pos" parameter to the desired degree of movement. A positive value rotates the motor towards the outer side of the box.
For main motors, the unit is mm, but multiplied by the scaling factor 57.3. A negative value makes the lead screw move upwards, causing the foot to descend. For example, to move the lead screw upwards by 10 cm, "pos" should be set to -5730.

Call format:
```
mvm --pos=10 -m=1 -t=3
mvm --pos=-5730 -m=3 -t=10
```
* Simulated Point-to-Point Movement of a Single Leg [mvleg]:

This command relies on the settings of simulated points. Improper settings may cause discontinuity in speed and acceleration or motor disablement issues. This needs to be modified.
[Note on the use of interpolation functions: Initialization operations should be performed in RT and can be placed in count=1.]

Call format:
```
mvleg
```
* Foot Linear Movement [mvline]:

This command is used to move a single leg. It specifies the displacement of the foot (unit: mm) and moves it in a straight line to the target point. Trapezoidal trajectory planning is adopted, with a movement speed of about 1 cm/s. After each execution, a new starting position is recorded, allowing consecutive execution of multiple commands. After executing all commands, "init" must be called to restore the initial position.
All legs can also be moved together using the parameter "-a", but it is necessary to ensure that the initial positions of the four legs are identical. This is generally used for initializing before running gait trajectory planning.

Call format: [Note: l=1/2/3/4]
```
mvline -l=1 -x=30 -z=-50
mvline (-a) -z=-75
```
* Movement along Planned Gait Trajectory [mvplan]:

This command takes planned trajectory points and uses the param.interval_time to specify the time between each trajectory point. The program automatically performs cubic spline interpolation to fit the trajectory of the foot at 1ms intervals. The trajectory points are currently stored in the "PlanTrace" file in the build directory. The data in the file should be stored as data_num*12, with the row vector representing the number of points and the column vector representing the x, y, z coordinates of the leg.

Call format:
```
mvplan (-a) - perform movement for all legs
mvplan -l=1 - read in all data, but only execute for leg 1
```

### Upper Computer Control Method
The upper computer control instructions differ from the cmd control instructions in that, before executing any instruction, the isFinishFlag flag in the ROS global parameters is set to false. It is only after all instructions have been executed that the isFinishFlag flag is set to true. The upper computer side continuously reads the value of the isFinishFlag flag in the ROS global parameters and can only publish the next command when it detects true. Apart from this, the underlying implementation of the instructions is the same.

Additionally, in order to re-plan foot motion after receiving ground contact feedback, the upper computer side has designed two sets of MATLAB code, which interact with the industrial computer using either topic communication or service communication. Topic communication does not require waiting for the lander's commands to be executed, and can directly proceed to the next trajectory calculation. Service communication, on the other hand, waits for the lander's commands to be executed or for interruption due to ground contact. In this case, the lander's industrial computer returns the current foot position to MATLAB, which is then used for a new planning process.

The data format for communication with the upper computer is as follows:
```
# Client Request
# 0: getpos; 1: init; 2: planfoot; 3: planmotion
int32 command_index
# For planfoot
int32 leg_index
float64[3] foot1_motion
float64[3] foot2_motion
float64[3] foot3_motion
float64[3] foot4_motion
# For planmotion
int32 data_num
float64[] foot1_trace_x
float64[] foot1_trace_y
float64[] foot1_trace_z
float64[] foot2_trace_x
float64[] foot2_trace_y
float64[] foot2_trace_z
float64[] foot3_trace_x
float64[] foot3_trace_y
float64[] foot3_trace_z
float64[] foot4_trace_x
float64[] foot4_trace_y
float64[] foot4_trace_z
---
# Server Response Data
bool isFinish
float64[3] foot1_position
float64[3] foot2_position
float64[3] foot3_position
float64[3] foot4_position
```
(For server response data, only applicable for feedback control when using service communication)

* Instructions without feedback:

Get initial positions of encoders ```[getposplan][instruction 0]```

Pose initialization ```[initplan][instruction 1]```

Single leg and multi-leg motion simultaneously ```[planfoot][instruction 2]```:

Requires setting leg_index (0/1/2/3 or 12, where 12 represents all legs) and four arrays, each containing three elements, specifying the displacement dx, dy, and dz of the four legs relative to the initial position.

Motion along planned trajectory points ```[planmotion][instruction 3]```:

Requires setting data_num (not exceeding 100) and the displacement trajectory of the four legs.

Clear misalignment and re-enable motors ```[cl][instruction -1]```

* Instructions with feedback:

Motion along planned trajectory points ```[planmotionfeedback][instruction 3]```: 

Returns the positions of the four leg tips.

