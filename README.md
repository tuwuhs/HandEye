# HandEye

This package implements robot hand-eye calibration using SwiftFusion. It's still a work-in-progress. The approach will be adapted from this paper:

Kenji Koide and Emanuele Menegatti, General Hand-Eye Calibration based on Reprojection Error Minimization, IEEE Robotics and Automation Letters/ICRA2019 [[link](https://ieeexplore.ieee.org/document/8616862)].

## Building

```
swift build
```

## Run calibration with simulated dataset

Only two basic methods are implemented so far: Tsai's method and pose-based factor graph. Both of these methods assume that the poses from the camera to the calibration object are known (for instance from pose estimation / PnP). Implementation of the method described in the paper is still on progress.

```
swift run SimulatedCalibration
```

## Run tests

In Linux: 
```
swift test --enable-test-discovery
```
