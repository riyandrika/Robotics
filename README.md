# Academic Projects for COMP60019 Robotics
Third-year Undergraduate Course at Department of Computing, Imperial College London

Instructed by Andrew Davison, Professor of Robot Vision

This course uses the powerful [CoppeliaSim](https://coppeliarobotics.com/) (Edu version) simulator for design & analysis of robots. Scripting was done in Lua, a high-level programming language, with great API support by CoppeliaSim.

Scope of course:
- Introduction to Robotics
- Robot Motion
- Sensors
- Probabilistic Robotics
- Monte Carlo Localisation
- Advanced Sensing
- SLAM

## Task: Waypoint Navigation with Monte Carlo Localisation

Desired outcome: Given a set of coordinates in 2D space, the robot should perform waypoint navigation as accurately as possible with one outward-looking depth sensor. The robot needs to track its position in space by Monte Carlo Localisation and adjust its motion parameters accordingly.

Steps in MCL:
1. Motion prediction based on odometry
2. Measurement update based on sonar
3. Normalisation
4. Resampling

In this application, we generate 100 particles to represent the robot position with additive Gaussian noise. These particles are then probabilistically filtered to generate a single instance with the highest likelihood for the current position of the robot.

Scene file: [WaypointNavigation_scene.ttt](WaypointNavigation_scene.ttt) | Robot script: [WaypointNavigation.lua](WaypointNavigation.lua)

https://user-images.githubusercontent.com/59057196/159030503-a514ef2c-891a-4959-a4e4-a3fde26f19c3.mp4

## Task: Random Bounce Control

Desired outcome: The robot should continue moving while avoiding all obstacles in its path. Feedback control system to modify speed & direction of robot based on proximity to an ostacle.

Scene file: [RandomBounce_scene.ttt](RandomBounce_scene.ttt) | Robot script: [RandomBounce.lua](RandomBounce.lua)

https://user-images.githubusercontent.com/59057196/159030484-2751428a-3be2-4afb-bd81-cf5db3d21379.mp4

## Task: Wall Following

Desired outcome: The robot should continue moving along a wall while maintaining a specified distance from the wall. Feedback control system to perpetually perform odometry for robot motion to "follow" the path of the wall.

Scene file: [WallFollowing_scene.ttt](WallFollowing_scene.ttt) | Robot script: [WallFollowing.lua](WallFollowing.lua)

https://user-images.githubusercontent.com/59057196/159030496-2e80e00d-3e70-4cff-8cbd-445e01b16d98.mp4
