# -*- coding: utf-8 -*-
#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from conans import python_requires, tools
import os

base = python_requires("RBDyn/1.1.0@gergondet/stable")


class MCRBDynURDFConan(base.RBDynConan):
    name = "mc_rbdyn_urdf"
    version = "1.1.0"
    description = "Import URDF models as RBDyn objects"
    topics = ("robotics", "model", "python")
    url = "https://github.com/jrl-umi3218/mc_rbdyn_urdf"
    homepage = "https://github.com/jrl-umi3218/mc_rbdyn_urdf"
    author = "Pierre Gergondet <pierre.gergondet@gmail.com>"
    license = "BSD-2-Clause"
    exports = ["LICENSE"]
    exports_sources = ["CMakeLists.txt", "conan/CMakeLists.txt", "binding/*", "cmake/*", "CMakeModules/*", "doc/*", "include/*", "src/*"]
    generators = "cmake"
    settings = "os", "arch", "compiler", "build_type"
    options = { "python_version": ["2.7", "3.3", "3.4", "3.5", "3.6", "3.7"] }
    default_options = { "python_version": base.get_python_version() }

    requires = (
        "tinyxml2/7.0.1@nicolastagliani/stable",
        "RBDyn/1.1.0@gergondet/stable"
    )
