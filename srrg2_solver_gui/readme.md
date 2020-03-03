# Package `srrg2_solver_gui`

Graph viewer for the `srrg2_solver`. Still a *work in progress* :D

## How to build
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.

1. initialize the `srrg2` Catkin workspace following the guide [here](https://github.com/srrg-sapienza/srrg2_solver/tree/master/srrg2_solver). As indicated in the aforementioned guide, we suggest to have a directory in which you clone all the `srrg2` repositories (referred here as `SRRG2_SOURCE_ROOT`) and a directory that contains the Catkin workspace (referred here as `SRRG2_WS_ROOT`)

2. install all viewport dependencies following the guide [here](https://github.com/srrg-sapienza/srrg2_qgl_viewport/tree/master/srrg2_qgl_viewport)

3. clone all the `srrg2` packages
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_cmake_modules.git
git clone https://github.com/srrg-sapienza/srrg2_core.git
git clone https://github.com/srrg-sapienza/srrg2_qgl_viewport.git
git clone https://github.com/srrg-sapienza/srrg2_solver.git # includes this package
```

4. link all the Catkin packages in the workspace
```bash
cd <SRRG2_WS_ROOT>/src
ln -s <SRRG2_SOURCE_ROOT>/srrg2_cmake_modules .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_core/srrg2_core .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_qgl_viewport/srrg2_qgl_viewport .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver_gui .
```

5. build using Catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_solver_gui
```

Only a **super experimental** executable is provided:
* [`srrg_solver_gui_app`](src/app/srrg_solver_gui_app.cpp): loads a configuration and a graph from file and performs batch optimization after a bunch of seconds
```bash
source <SRRG2_WS_ROOT>/devel/setup.bash # if you use zsh you have to source the *.zsh file :)
rosrun srrg2_solver srrg_solver_gui_app -h
```
