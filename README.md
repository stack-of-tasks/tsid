# TSID - Task Space Inverse Dynamics
[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/tsid/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/tsid/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/tsid/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/tsid/master/coverage/)
[![PyPI version](https://badge.fury.io/py/tsid.svg)](https://badge.fury.io/py/tsid)

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

TSID is a C++ library for optimization-based inverse-dynamics control based on the rigid multi-body dynamics library [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

## Documentation
* Take a look at the project [wiki](https://github.com/stack-of-tasks/tsid/wiki) for an overview of the design of the library.
* In the exercises folder you can find several examples of how to use TSID in Python with robot manipulators, humanoids, or quadrupeds.
* On the [website of Andrea Del Prete](https://andreadelprete.github.io/#teaching) you can find slides and video lessons on TSID.
* [Memmo 2020 summer school](https://memory-of-motion.github.io/summer-school/)

## Installation with Conda

If you want to directly dive into TSID in Python, only one single line is sufficient (assuming you have Conda installed):

<p align="center">
conda install tsid -c conda-forge
</p>


## Installation from Debian/Ubuntu packages, with robotpkg
If you have never added robotpkg's software repository you can do it with the following commands:
```
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
EOF

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt update
```
You can install TSID and its python bindings (replace * with you Python version) with:
```
sudo apt install robotpkg-py3*-tsid
```


## Installation from sources

First you need to install the following dependencies:
* boost (unit_test_framework)
* eigen3
* [pinocchio](https://github.com/stack-of-tasks/pinocchio)
* [eiquadprog](https://github.com/stack-of-tasks/eiquadprog)
* [example-robot-data](https://github.com/Gepetto/example-robot-data) (only for running the examples)

To install eigen3 on Ubuntu you can use apt-get:
  `sudo apt-get install libeigen3-dev`

To install [pinocchio](https://github.com/stack-of-tasks/pinocchio) follow the instruction on its website.

To compile TSID:

    cd $DEVEL/openrobots/src/
    git clone --recursive git@github.com:stack-of-tasks/tsid.git
    cd tsid
    mkdir _build-RELEASE
    cd _build-RELEASE
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=$DEVEL/openrobots
    make install

## Python Bindings
To use this library in python, we offer python bindings based on Boost.Python and EigenPy.

To install EigenPy you can compile the source code:

    git clone https://github.com/stack-of-tasks/eigenpy

or, on Ubuntu, you can use apt-get:

    sudo apt-get install robotpkg-py3*-eigenpy

For testing the python bindings, you can run the unit test scripts in the `script` folder, for instance:

    ipython script/test_formulation.py

To run the demo using gepetto-viewer:

    ipython demo/demo_romeo.py

## Credits

This package is authored by:

- [Andrea Del Prete](https://andreadelprete.github.io) (University of Trento)
- [Justin Carpentier](https://jcarpent.github.io) (INRIA)

It includes key contributions from:

- [Julian Viereck](https://github.com/jviereck) (Max Planck Institute, New  York  University)
- [Sanghyun Kim](https://github.com/ggory15) (Seoul National University)
- [Eloise Dalin](https://github.com/dalinel) (LORIA, INRIA Lorraine)
- [Noelie Ramuzat](https://github.com/NoelieRamuzat) (LAAS, CNRS)
- [Pierre Fernbach](https://github.com/pFernbach) (LAAS, CNRS)
- [Aurelie Bonnefoy](https://github.com/ABonnefoy) (LAAS, CNRS)
- [Etienne Arlaud](https://github.com/EtienneAr) (INRIA)
- [Fabian Schramm](https://github.com/fabinsch) (INRIA)

And is maintained by:

- [Guilhem Saurel](https://github.com/nim65s) (LAAS-CNRS)

## Citing

If you are (or not) happy with TSID and want to cite it, please use the following citation:

    @inproceedings {adelprete:jnrh:2016,
	    title = {Implementing Torque Control with High-Ratio Gear Boxes and without Joint-Torque Sensors},
	    booktitle = {Int. Journal of Humanoid Robotics},
	    year = {2016},
	    pages = {1550044},
	    url = {https://hal.archives-ouvertes.fr/hal-01136936/document},
	    author = {Andrea Del Prete, Nicolas Mansard, Oscar E Ramos, Olivier Stasse, Francesco Nori}
    }
