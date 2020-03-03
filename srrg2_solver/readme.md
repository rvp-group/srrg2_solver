# Package `srrg2_solver`

This package contains our new Iterative Least-Squares based solver.
The `srrg2_solver` is developed in modern `C++17`, allowing to extend it very easily for fast prototyping of custom solutions.
The unified APIs allow to address dense/dynamic problems (e.g. ICP) and sparse/static ones (e.g. BA or PGO) using the same solver.

###### Hot features:
* unified state-of-the-art solver for different problems (sparse/dense static/dynamic)
* types defined at compile time to speed-up runtimes
* extensive support to Automatic Differentiation (AD)
* easy prototyping of new variables/factors/ILS-algorithms
* automatic serialization/deserialization of solver configuration and factor graphs through our `BOSS` library
* minimal code-base that build using Catkin

## How to build
The `srrg2_solver` is developed using our `srrg2` framework.
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.
Please follow this guide to build and run `srrg2_solver` on your machine:

1. initialize the `srrg2` Catkin workspace following the guide [here](https://github.com/srrg-sapienza/srrg2_core/tree/master/srrg2_core). As indicated in the aforementioned guide, we suggest to have a directory in which you clone all the `srrg2` repositories (referred here as `SRRG2_SOURCE_ROOT`) and a directory that contains the Catkin workspace (referred here as `SRRG2_WS_ROOT`)

2. clone all the `srrg2` dependencies of this package
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_cmake_modules.git # basic cmake-modules
git clone https://github.com/srrg-sapienza/srrg2_core.git # core data-structures and utilities
```

3. clone this repository
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_solver.git
```

4. link all the required packages in your Catkin workspace
```bash
cd <SRRG2_WS_ROOT>/src
ln -s <SRRG2_SOURCE_ROOT>/srrg2_cmake_modules .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_core/srrg2_core .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver .
```

5. build using Catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_solver
```

6. [OPTIONAL] build unit-tests using catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_solver --catkin-make-args tests
```

## How to use
We provide a set of self-explanatory examples that cover both sparse (PGO) and dense (ICP) problems - located in the directory [`examples`](examples).
In the [`tests`](tests) folder, we provide also highly commented unit-tests that can act as examples of usage. In the [`data`](examples/data) directory, we provide an example of factor graph in `BOSS` format. Finally, in the [`app`](app) directory we provide two executables:
* [`solver_app_graph_converter`](app/solver_app_graph_converter.cpp): converts graphs from/to `g2o` format; usage:
```bash
source <SRRG2_WS_ROOT>/devel/setup.bash # if you use zsh you have to source the *.zsh file :)
rosrun srrg2_solver solver_app_graph_converter -h
```

* [`solver_app_graph_optimizer`](app/solver_app_graph_optimizer.cpp): loads (or generates) a solver configuration file and performs batch optimization of a factor graph (loaded also from file); usage:
```bash
source <SRRG2_WS_ROOT>/devel/setup.bash # if you use zsh you have to source the *.zsh file :)
rosrun srrg2_solver solver_app_graph_optimizer -h
```
