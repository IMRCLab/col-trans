# col-trans

## Simulation

### Initial setup

```
cd crazyflie-firmware
make cf2_defconfig
make bindings_python
```

### Use Python controller

(Make sure config/initialize.yaml uses 'lee' as controller)

```
cd sim
export PYTHONPATH=$PWD/controllers
python3 controller.py test
```

### Use Firmware controller

(Make sure config/initialize.yaml uses 'lee_firmware' as controller)

```
cd sim
export PYTHONPATH=$PWD/../crazyflie-firmware
python3 controller.py test
```

### Visualization

```
cd vis
python3 visualize.py
```

## osqp demo

### regenerate the code

```
cd osqp
python3 osqp_example.py
```

Note that we have a bunch of custom changes to support floating point arithmetic (rather than double).
On the example, this increases speed from 2ms -> 300us. So don't blindly overwrite the files, but use git
to track the changes.

### firmware

```
osqp/cffirmware_osqp
make
```

Note that this uses a simple out-of-source-build. All submodules need to recursively updated (`git submodule update --init --recursive`).

## ROS2 Package

Use a symlink to your ROS2 workspace

```
ln -s <path-to>/col-trans/coltrans_ros <path-to>/ros2_workspace/src
```