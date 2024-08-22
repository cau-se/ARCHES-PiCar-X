# PiCar (base) from SunFounder©
*can be use for PiCar-V or PiCar-S*

<img src="./doc/images/Full_PiCar-v0.jpg" width="640" alt="" >

This project is intended to provide a CAD base to :
- Have a digital version;
- Make improvements;
- Export for simulators;
- Export for 3D printer;

# Changelog 

## Version 2

Improved Support engine :
- Engine position fix ( 1 -> 3 points)
- Add Encoded wheel for more position accurate

<img src="./doc/images/Support_Engine_assembly-v1.png" width="640" alt="" >
<img src="./doc/images/Support_Engine-v1.png" width="320" alt="" >

## Version 1

Improved steering :
- Bearing assembled with clamping

<img src="./doc/images/Steering-v1.jpg" width="640" alt="" >
<img src="./doc/images/Steering-proto.jpg" width="319" alt="" > <img src="./doc/images/Steering_comparison.jpg" width="319" alt="" >

## Version 0

Version without modification provided by SunFounder

# Structure repository

```
|- CAD : CAD files (freecad)
|- doc : Documentation
|- export : generated file for 3th app
| |- STL : for 3D printer / simulator
| |- DAE : for Simulator(ROS - Gasebo/Ignition)
```

