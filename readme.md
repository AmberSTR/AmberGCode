#  AmberGCode

**This is a very simple beta version of a G-code compatibility layer for Amber robot arms and comes with absolutely no warranty**


**Need to be improved in the future**

### Install Pink

Install 👉 [tasts-robots/pink](https://github.com/tasts-robots/pink)

### Generate G-code with LaserWeb

Use [LaserWeb](https://github.com/LaserWeb/LaserWeb4-Binaries/)
Rename output file to `gcode.gcode`

### Translate G-code to XY Points

Change config in G2XY.py and run.

//TODO: Need a manual

> UP_SAMPLE_RATIO = 200
>
>
> DOWN_SAMPLE_RATIO = 20
>
> INIT_POSITION_DELAY = 500

 

 ### Generate Joint Trajectory Files

  Configure Pink Solver

  //TODO: Need a manual

  Run 2DTrajectoryGeneratorxx.py

  and get `out.csv`

 ### Execute it somehow

  //TODO

  But you can read and execute it by UDP Protocol

  Safe frequency is <10hz or you may can't get the point