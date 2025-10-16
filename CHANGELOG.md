# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.9.0] - 1980-01-01

- Fix value of nEq when removeRigidContact for contact with priority > 0
- update jrl-cmakemodules to v1
- CMake: honor BUILD_STANDALONE_PYTHON_INTERFACE option

## [1.8.0] - 2025-03-29

- Fix missing `const` specifier in python bindings for methods `RobotInertia.set_rotor_inertia()`
 and `RobotInertia.set_gear_ratios()`
- Add python bidings for the measured force contact adn task actuation equality
- Deprecated `tsid/contacts/measured-3Dforce.hpp` and `tsid/contacts/measured-6Dwrench.hpp`
  use `tsid/contacts/measured-3d-force.hpp` and `tsid/contacts/measured-6d-wrench.hpp` instead
- Introduce use of override keyword for virtual methods

## [1.7.1] - 2024-08-26

- Fix a typo in ex_4_walking
- check solver status, set eps_abs to 1e-6, fix seed
- CMake: require >= 3.10
- add changelog
- setup ruff & fix isort pre-commit config
- update ROS CI

## [1.7.0] - 2023-05-13

- expose SE3ToVector and vectorToSE3
- remove warnings
- Enhance Python target packaging
- Add CI with GitHub Action for conda
- Add support for proxqp and osqp solver
- Clean and update contributors list
- pre-commit autoupdate
- Expose TaskJointPosVelAccBounds
- Expose AddMotionTask for TaskJointPosVelAccBounds
- Fix qpmad
- Add clang-format Google style
- Add Measured force as an external force task with moving objects
- update CMake: fetch submodule, set default build type, fix RPATH
- fix for eigenpy v3
- np.matrix → np.array
- tooling: setup black isort & toml-sort

## [1.6.3] - 2022-11-02

- Require C++17
- fix tests in 18.04
- update pinocchio use

## [1.6.2] - 2022-09-05

- update python tests for np array
- add task actuation equality
- use generated headers
- Choose floating-base (or not) when creating a robot wrapper
- Add optional support of qpmad solver
- update packaging for eigenpy 2.7.12

## [1.6.1] - 2021-10-19

This new release is mostly a maintenance release, including fixes for the recent version of Eigen 3.4.

## [1.6.0] - 2021-03-18

- Add bindings for method Contact6d::getMotionTask
- Add Center of Pressure Taks
- fix warnings

## [1.5.0] - 2021-03-03

- [py] Add config variable to specify whether end-effector task should
   be formulated in local frame or world frame.
- [C++] Standardize names of methods to get and set mask in motion tasks.
- [py] Add accessors masks in TaskCOM and TaskSE3Equality.
- bugfixes

## [1.4.2] - 2020-11-26

- Add a mask to task-com-equality
- Fix bug in Contact6d::setRegularizationTaskWeightVector
- CMake: fix configure without tests

## [1.4.1] - 2020-09-25

- fix memory leaks thanks to shared_ptr
- fix warnings
- fix package.xml for ROS

## [1.4.0] - 2020-09-09

- add setGravity
- stop using StdVec in python to make code more user friendly
- add 6d contact with motion constraint at priority level 1 in python
- use example-robot-data in notebooks

## [1.3.1] - 2020-06-05

- fix license
- fix generated tsid.pc

## [1.3.0] - 2020-05-26

- reactive test
- add package.xml
- add CheatSheet
- updates for numpy.array & python 3
- updateRigidContactWeights: fix and add to python API
- use eiquadprog
- fix compatibility with pinocchio v2.4.5

## [1.2.3] - 2020-03-30

- renamed tests dir
- fix python tests
- CMake: export project and use exports from dependencies
- CMake: keep minimal required instructions

## [1.2.2] - 2020-03-01

- add angular momentum equality task
- update to pinocchio changes
- Python 3 compatibility
- add some documentation
- fix python issue

## [1.2.1] - 2019-09-19

- fix compatibility with recent pinocchio versions

## [1.2.0] - 2019-03-06

- Pinocchio v2, fix #31
- Fix demo_romeo for pinocchio v2
- Pull request for use TSID in python.
- Add missing includes, fix #18

## [1.1.0] - 2018-10-10

This release updates to non backward-compatibles changes in pinocchio v1.3.0

## [1.0.2] - 2018-06-12

This release is mostly a maintenance release.
It fixes some bug with respect to **Pinocchio**.
It also fixes some issues with respect to the packaging.

## [1.0.1] - 2018-01-12

- [joint-posture-task] Fix bug in computation of task matrix from mask
- [task-se3-equality] Add method to get frame-id
- [Robot] Add missing evaluation of the center of mass acceleration provided zero joint acceleration
- [Math] Fix bug related to Eigen undefined function set_is_malloc_allowed
- [CMake] Correct minimal version of Eigen3
- [inv-dyn-form-acc-force] Fix potential bug in removal of contact constraint
- [task-se3-equality] Fix small bug in computaiton of acceleration (just used for debugging)
- [contact-6d] Add methods to set reference force and weight vector
- [inv-dyn-form-acc-force] Fix bug: weight of force regularization task was not updated
- [robot-wrapper] Fix bug in mass matrix: copy upper triangular part to lower triangular part (before this it was set to zero)
- [formulations] Remove debug prints
- [formulations] Fix bug in update of task weights
- [robot-wrapper] BUG FIX: compute center of mass acceleration in computeAllTerms (before it was not computed so we were introducing random number in the CoM task)
- [tsid-formulations] Add method to change the weight of a task.
- [eigquadprog-fast] In DEBUG, in case a constraint is not verified, set the status flag to ERROR (even if the solver said the problem has been solved)
- [task-com-equality] Fix little bugs in methods to get pos/vel references and errors, and in method to get desired acceleration
- [math-utils] Add function to check if matrix/vector contains NaN
- [math-utils] Pass JacobiSVD by reference in function solveWithDampingFromSvd
- [math-utils] Add function to solve linear system of equations from svd decomposition

## [1.0.0] - 2017-06-16

This is the first release of TSID.
This release includes minimal features for the torque control of humanoid robots such as HRP-2.

[Unreleased]: https://github.com/stack-of-tasks/tsid/compare/v1.9.0...HEAD
[1.9.0]: https://github.com/stack-of-tasks/tsid/compare/v1.8.0...v1.9.0
[1.8.0]: https://github.com/stack-of-tasks/tsid/compare/v1.7.1...v1.8.0
[1.7.1]: https://github.com/stack-of-tasks/tsid/compare/v1.7.0...v1.7.1
[1.7.0]: https://github.com/stack-of-tasks/tsid/compare/v1.6.3...v1.7.0
[1.6.3]: https://github.com/stack-of-tasks/tsid/compare/v1.6.2...v1.6.3
[1.6.2]: https://github.com/stack-of-tasks/tsid/compare/v1.6.1...v1.6.2
[1.6.1]: https://github.com/stack-of-tasks/tsid/compare/v1.6.0...v1.6.1
[1.6.0]: https://github.com/stack-of-tasks/tsid/compare/v1.5.0...v1.6.0
[1.5.0]: https://github.com/stack-of-tasks/tsid/compare/v1.4.2...v1.5.0
[1.4.2]: https://github.com/stack-of-tasks/tsid/compare/v1.4.1...v1.4.2
[1.4.1]: https://github.com/stack-of-tasks/tsid/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/stack-of-tasks/tsid/compare/v1.3.1...v1.4.0
[1.3.1]: https://github.com/stack-of-tasks/tsid/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/stack-of-tasks/tsid/compare/v1.2.3...v1.3.0
[1.2.3]: https://github.com/stack-of-tasks/tsid/compare/v1.2.2...v1.2.3
[1.2.2]: https://github.com/stack-of-tasks/tsid/compare/v1.2.1...v1.2.2
[1.2.1]: https://github.com/stack-of-tasks/tsid/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/stack-of-tasks/tsid/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/stack-of-tasks/tsid/compare/v1.0.2...v1.1.0
[1.0.2]: https://github.com/stack-of-tasks/tsid/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/stack-of-tasks/tsid/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/stack-of-tasks/tsid/releases/tag/v1.0.0
