mc_rbdyn_urdf
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[ ![Download](https://api.bintray.com/packages/gergondet/multi-contact/mc_rbdyn_urdf%3Agergondet/images/download.svg) ](https://bintray.com/gergondet/multi-contact/mc_rbdyn_urdf%3Agergondet/_latestVersion)
[![CI](https://github.com/jrl-umi3218/mc_rbdyn_urdf/workflows/CI%20of%20mc_rbdyn_urdf/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_rbdyn_urdf/actions?query=workflow%3A%22CI+of+mc_rbdyn_urdf%22)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/mc_rbdyn_urdf/doxygen/HEAD/index.html)

This library allows to parse an URDF file and create RBDyn structure from it. It is entirely ROS-free.

Note: this package is deprecated, URDF parsing is now available directly in [RBDyn](https://github.com/jrl-umi3218/RBDyn)

Installing
------

## Ubuntu LTS (16.04, 18.04, 20.04)

```bash
# Make sure you have required tools
sudo apt install apt-transport-https lsb-release
# Add our key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 892EA6EE273707C6495A6FB6220D644C64666806
# Add our repository (stable versions)
sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-release $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/multi-contact.list'
# Use this to setup the HEAD version
# sudo sh -c 'echo "deb https://dl.bintray.com/gergondet/multi-contact-release $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/multi-contact.list'
# Update packages list
sudo apt update
# Install packages
sudo apt install librbdyn-dev python-rbdyn python3-rbdyn
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
