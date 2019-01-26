# TSID - Task Space Inverse Dynamics

[![Building Status](https://travis-ci.org/stack-of-tasks/tsid.svg?branch=master)](https://travis-ci.org/stack-of-tasks/tsid)
[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/tsid/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/tsid/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/tsid/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/tsid/master/coverage/)

TSID is C++ library for optimization-based inverse-dynamics control based on the rigid multi-body dynamics library [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
Take a look at the project [wiki](https://github.com/stack-of-tasks/tsid/wiki) for more details.

## Dependencies
* boost (unit_test_framework)
* eigen3
* [pinocchio](https://github.com/stack-of-tasks/pinocchio)

To install eigen3 on Ubuntu you can use apt-get:
  `sudo apt-get install libeigen3-dev`

To install [pinocchio](https://github.com/stack-of-tasks/pinocchio) follow the instruction on its website.

## Installation

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

    sudo apt-get install robotpkg-py27-eigenpy
     
For testing the python bindings, you can run the unit test scripts in the `script` folder, for instance:

    ipython script/test_formulation.py
    
To run the demo using gepetto-viewer:

    ipython demo/demo_romeo.py

