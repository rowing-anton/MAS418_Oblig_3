# MAS418: Programming for Intelligent Robotics and Industrial systems
Obligatory Exercise 3 - Distributed Control Systems

## Task 1 - PLC 
### Step 1 - PLC Simulator
Create a PLC program including manual (feed forward and slider gain input from visualization) and auto mode control (velocity feedforward and closed loop P controller and motion ref generator) of the green crane simulator introduced in lecture #2.5. All details are in the Simulink model available in the GreenCraneSimulator_SomplifiedHydMechModel.slx model [GitHub - hagenmek/MAS418_SimulinkModel](https://github.com/hagenmek/MAS418_SimulinkModel)

Test both modes. 

You can also use the solution proposal on [Github (Lab#5)](https://github.com/hagenmek/mas418_TwinCAT/tree/LAB%235/MAS418_LAB1/MAS418_LAB1/LabExercise/Simulator)
from the last year lab exercise as a starting point.

### Step 2 - Boom angle position
Prepare conversion from simulated cylinder pos to boom angels. This value will be sent over ADS to ROS in step 3. 

## Task 2 - ROS 2
### Step 1 - Input signals from PLC
Prepare for receiving boom angle value from PLC to control (visualize) the position of the boom in Rviz and piston and rod side pressures for monitoring.

### Step 2 - Output commands to PLC
Prepare sending Start and Stop commands, and necessary motion reference generator settings (vel SP, pos SP, etc.), to start and stop the auto sequence running on the PLC.

## Task 3 - PLC to ROS 2 communication
### Step 1 - ADS-ROS interface setup
This step will be introduced in lecture #3 on the 11. April. However, if you want you can take a look at examples from last year.

1. Follow this guide: ROS2-ADS-TwinCAT_Guide.pdf Download ROS2-ADS-TwinCAT_Guide.pdf 

2. Follow instruction on this Git repo: https://github.com/kristianmk/ros2-examples-ads

### Step 2 - Test ADS-ROS interface
Expand the example from step 1, add necessary signals according to Simulink model, and connected the node to existing nodes with visualization in Rviz. Minimum required ADS signals are:

Inputs from PLC to ROS: boom angle for visualization, piston and rod side pressures.
Outputs from ROS to PLC: Start and Stop commands, and necessary motion reference generator variables, to run the auto sequence.

