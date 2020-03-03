# SRRG2-SOLVER
Basic solver specifically designed for SLAM pipelines. Highly efficient, no fancy stuff, but man if it runs! :D

This repository contains several Catkin packages:

  1. [srrg2_solver](srrg2_solver): the barebone `srrr2-solver` package. Provides batch optimization with `BOSS` graphs, conversion of graphs from/to `g2o` format to/from `BOSS` format, and all other solver modules (most used variables and factors, linear solvers, iteration algorithms, robustifiers, termination criteria ... ). Minimal build dependencies are required.

  2. [srrg2_solver_extras](srrg2_solver_extras): provides additional variables and factors (e.g. the AD version of the factors in tha package [srrg2_solver](srrg2_solver)). Minimal build dependencies are required.

  3. [srrg2_solver_experiments](srrg2_solver_experiments): provides comparative examples with `pcl` - based point registration.

  4. [srrg2_solver_gui](srrg2_solver_gui): **super experimental** package that includes a graph viewer (works only with standard pose-graphs now). Requires OpenGL, QT, QGLViewer.

To know more about those packages, refer to the `readme` file in the respective packages.

## Publications
To have a more detailed overview on this solver, you can read our brand new [preprint](https://arxiv.org/abs/2002.11051).
If you use our `srrg2_solver`, please cite us in your work.
```bibtex
@article{grisetti2020least,
  title={Least Squares Optimization: from Theory to Practice},
  author={Grisetti, Giorgio and Guadagnino, Tiziano and Aloise, Irvin and Colosi, Mirco and Della Corte, Bartolomeo and Schlegel, Dominik},
  journal={arXiv preprint arXiv:2002.11051},
  year={2020}
}
```

## Something is not working?
[Open an issue](https://github.com/srrg-sapienza/srrg2_solver/issues/new) or contact the mantainers :)

## Contributors
* Giorgio Grisetti
* Bartolomeo Della Corte
* Dominik Schlegel
* Irvin Aloise
* Mirco Colosi
* Tiziano Guadagnino

## License
BSD 3.0
