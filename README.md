mc_rbdyn_urdf
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Build Status](https://travis-ci.org/jrl-umi3218/mc_rbdyn_urdf.svg?branch=master)](https://travis-ci.org/jrl-umi3218/mc_rbdyn_urdf)
[![AppVeyor status](https://ci.appveyor.com/api/projects/status/vciabwbng3jgkymi/branch/master?svg=true)](https://ci.appveyor.com/project/gergondet/mc-rbdyn-urdf/branch/master)
[ ![Download](https://api.bintray.com/packages/gergondet/multi-contact/mc_rbdyn_urdf%3Agergondet/images/download.svg) ](https://bintray.com/gergondet/multi-contact/mc_rbdyn_urdf%3Agergondet/_latestVersion)

This library allows to parse an URDF file and create RBDyn structure from it. It is entirely ROS-free.

Installing
------

## Ubuntu LTS (14.04, 16.04, 18.04): PPA

Use the [multi-contact-unstable](https://launchpad.net/~pierre-gergondet+ppa/+archive/ubuntu/multi-contact-unstable) ppa:
```bash
sudo add-apt-repository ppa:pierre-gergondet+ppa/multi-contact-unstable
sudo apt-get update
sudo apt-get install librbdyn-dev librbdyn-doc python-rbdyn python3-rbdyn
```

## Manually build from source

#### Dependencies

- tinyxml2
- Eigen (>= 3.0)
- SpaceVecAlg (with Python bindings if needed)
- RBDyn (with Python bindings if needed)
- Cython (>= 0.2) (Python bindings)
- Eigen3ToPython (Python bindings)
- Boost (unit tests)

### Building

```sh
git clone --recursive https://github.com/jrl-umi3218/mc_rbydn_urdf
cd mc_rbydn_urdf
mkdir _build
cd _build
cmake [options] ..
make && make intall
```

#### CMake options

By default, the build will use the `python` and `pip` command to install the bindings for the default system version (this behaviour can be used to build the bindings in a given virtualenv). The following options allow to control this behaviour:

 * `PYTHON_BINDING` Build the python binding (ON/OFF, default: ON)
 * `PYTHON_BINDING_FORCE_PYTHON2`: use `python2` and `pip2` instead of `python` and `pip`
 * `PYTHON_BINDING_FORCE_PYTHON3`: use `python3` and `pip3` instead of `python` and `pip`
 * `PYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON2`: builds two sets of bindings one with `python2` and `pip2`, the other with `python3` and `pip3`
 * `BUILD_TESTING` Enable unit tests building (ON/OFF, default: ON)
