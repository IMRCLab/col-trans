# col-trans

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