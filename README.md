mc_rbdyn_urdf
==

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Hosted By: Cloudsmith](https://img.shields.io/badge/OSS%20hosting%20by-cloudsmith-blue?logo=cloudsmith)](https://cloudsmith.com)
[![CI](https://github.com/jrl-umi3218/mc_rbdyn_urdf/workflows/CI%20of%20mc_rbdyn_urdf/badge.svg?branch=master)](https://github.com/jrl-umi3218/mc_rbdyn_urdf/actions?query=workflow%3A%22CI+of+mc_rbdyn_urdf%22)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/mc_rbdyn_urdf/doxygen/HEAD/index.html)

This library allows to parse an URDF file and create RBDyn structure from it. It is entirely ROS-free.

Note: this package is deprecated, URDF parsing is now available directly in [RBDyn](https://github.com/jrl-umi3218/RBDyn)

Installing
------

## Ubuntu LTS (16.04, 18.04, 20.04)

You must first setup our package mirror:

```
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash
```

You can also choose the head mirror which will have the latest version of this package:

```
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash
```

You can then install the package:

```bash
sudo apt install libmc-rbdyn-urdf-dev python-mc-rbdyn-urdf python3-mc-rbdyn-urdf
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
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf
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
