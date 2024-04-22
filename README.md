Standalone ALIP 1-Step Controller
====================

This is a standalone repository containing the Angular Momentum Linear Inverted Pendulum (ALIP) 1-step controller and corresponding Python bindings for the Digit humanoid robot. This is based on an old version of Grant Gibson's [Digit C++ Controller](https://github.com/UMich-BipedLab/digit_locomotion_controller).


## Setup for quick working example
This has been tested on Ubuntu 20 and Ubuntu 22.
```
conda env create -f alip-env.yml
mkdir build
mkdir extern
git submodule add -f https://github.com/pybind/pybind11.git extern/pybind11
make all
```

## Makefile
To build the dependencies for the controller:
```
make init
```
To build the controller (assuming you have the object files from `init`).
```
make controller
```
To build the Python bindings:
```
make pybind
```
To do all of these:
```
make all
```

## Example usage
We have an example running the controller within MuJoCo. From the root directory of this repository, after having built the Python bindings (there should be a .so file in your `example/` directory):
```
$ cd example
$ python mujoco_test.py
```
If you run into an error you may need to rename the `.so` file to `digit_controller_pybind.so`.

## Information on Python bindings
`src/pybind/digit_controller_pybind.cpp` contains the currently existing bindings. Currently, you can initialize the controller as follows:
```
import digit_controller_pybind as dc  # assuming .so bindings file is in same directory

gc = dc.Digit_Controller()
gc.Initialize_(init_ctrl_mode, flag_torque_only)
gc.Set_Initial_Standing_Gains_()
gc.Set_Initial_Walking_Gains_()
```
and then run the controller via:
```
gc.Update_(command, observation, limits)
```

### Adding new bindings
If you would like to add new bindings to other variables or functions within the controller code, please consult [the pybind11 documentation](https://pybind11.readthedocs.io/en/stable/classes.html). You may wish to do this, for example, if you would like access to the variables in `include/controller/Digit_Controller.hpp` via the `DigitController` object.

## References
[Grant Gibson's Digit C++ controller](https://github.com/UMich-BipedLab/digit_locomotion_controller)  
[Angular Momentum about the Contact Point for Control of Bipedal Locomotion: Validation in a LIP-based Controller](https://arxiv.org/abs/2008.10763)
