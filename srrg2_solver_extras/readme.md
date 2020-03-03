# Package `srrg2_solver_extras`

Additional AD factors for the `srrg2_solver`.

## How to build
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.

1. initialize the `srrg2` Catkin workspace following the guide [here](https://github.com/srrg-sapienza/srrg2_solver/tree/master/srrg2_solver). As indicated in the aforementioned guide, we suggest to have a directory in which you clone all the `srrg2` repositories (referred here as `SRRG2_SOURCE_ROOT`) and a directory that contains the Catkin workspace (referred here as `SRRG2_WS_ROOT`)

3. link this package in your Catkin workspace
```bash
cd <SRRG2_WS_ROOT>/src
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver_extras .
```

3. build using Catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_solver_extras
```
