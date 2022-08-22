"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

from os import getenv

import lsst.ts.m2com
from documenteer.conf.pipelinespkg import *

project = "ts_m2com"
html_theme_options["logotext"] = project
html_title = project
html_short_title = project
doxylink = {}  # Avoid warning: Could not find tag file _doxygen/doxygen.tag

intersphinx_mapping["ts_salobj"] = ("https://ts-salobj.lsst.io", None)

# Support the sphinx extension of plantuml
extensions.append("sphinxcontrib.plantuml")

# Put the path to plantuml.jar
path_plantuml = (
    "/home/saluser/plantuml.jar"
    if getenv("PATH_PLANTUML") is None
    else getenv("PATH_PLANTUML")
)
plantuml = f"java -jar {path_plantuml}"
