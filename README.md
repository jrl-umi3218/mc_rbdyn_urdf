mc_rbdyn_urdf
==

[![License LGPL 3](https://img.shields.io/badge/license-LGPLv3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0.txt)
[![build status](https://gite.lirmm.fr/ci/projects/4/status.png?ref=master)](https://gite.lirmm.fr/ci/projects/4?ref=master)

This library allows to parse an URDF file and create RBDyn structure from it. It is entirely ROS-free.

Dependencies
--

- tinyxml2
- Eigen (>= 3.0)
- SpaceVecAlg (with Python bindings if needed)
- RBDyn (with Python bindings if needed)
- Cython (>= 0.25) (Python bindings)
- Eigen3ToPython (Python bindings)
- Boost (unit tests)
