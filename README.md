# col-trans

## Simulation

### Initial setup

```
cd crazyflie-firmware
make cf2_defconfig
make bindings_python
cd ..
mkdir sim/output
```
## Simulation
```
python3 simulate.py cfg/rig (or pm)/10uavs.yaml --plot (if you do not want a plot, remove --plot)
```
### Use Python controller

(Make sure config/2uavs.yaml uses 'lee' as controller)

```
cd sim
export PYTHONPATH=$PWD/controllers
python3 controller.py test
```

### Use Firmware controller

(Make sure config/3uavs.yaml uses 'lee_firmware' as controller)

```
cd sim
export PYTHONPATH=$PWD/../crazyflie-firmware
python3 controller.py test
```

### Visualization

```
python3 vis.py --uavs 3 --load rig (or pm)

```

## osqp demo

### regenerate the workspace: given number of uavs and payload type. 
it will automatically be added to the crazyflie-firmware

```
cd osqp
python3 gen_qp_code.py --uavs 4 --mass point (or rigid)
```
## Whiskers plots
set the cores you want from the script
```python3 qps_plot.py```
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