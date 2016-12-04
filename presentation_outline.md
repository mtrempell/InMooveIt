1. Introduction/Overview (**Miguel**)
  1. InMoov robot
  2. We want to use MoveIt and ROS to control the InMoov robot
    1. Existing robots in industry that use MoveIt and/or ROS (**Tobi**)
  3. Discuss how the InMoov robot was not designed to be used with ROS/MoveIt. These things require knowledge of the robot's position (i.e. feedback) and InMoov's original design does not provide it.
  4. In order to use ROS/MoveIt, we must change up the design a little to in order to extract feedback from the robot.
2. Hardware (**Adham**)
  1. Changes we had to make to allow feedback.
  2. Maybe talk about open-servos for a bit (and why we couldn't use them)
  3. Instead, we dissassembled servos, added external h-bridges.
  4. master microcontroller/arduino
  5. show the circuit diagram
3. Software (**Miguel**)
  1. Talk generally how the software pieces will fit together. Show block diagram.
  2. ROS nodes communicating between arduino and PC to relay joint state information
  3. Introduce URDF (**Tobi**)
    1. What is URDF
    2. Examples of URDF
    3. How we are going to use the *Solidworks to URDF Exporter* to export the InMoov CAD files to a URDF model
  4. Updating the URDF (**Miguel**)
  5. If we finish our "simulation" in time, show it.
  6. Introduce *MoveIt* more in-depth (**Arsalan**)
    1. How we are going to use it.
    2. What will it allow us to do.
    3. etc.
4. Budget (**Arsalan**)
5. System Requirements (**Arsalan**)
6. Conclusion (if we need one, **whoever wants to do it**).
