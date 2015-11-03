# qb_interface_node
ROS node for communicate with sequence and multi-sequence of qbDevice

# Install
1. Create a new folder in your catkin workspace call "qb_interface"
2. Load all file in it, no other files are request

# Configure
A Yaml file is saved inside conf/ folder, all parameters inside the file are configurable.

example

**communication port**

port: '/dev/ttyUSB0'

**[Equilibrium/Preset] -> True , [P1/P2/PL] -> False**

eq_preset: false 

**[Hand command in percentual] -> True , [Hand command in TICK] -> False**

hand_perc: true 

**Unit of measurement, ['DEG', 'RAD', 'TICK']**

unit: 'DEG' 

**Step time**

step_time: 0.02

**ID cubes in chain**

IDcubes: [1 2] 

**ID hands in chain**

IDhands: [3] 
