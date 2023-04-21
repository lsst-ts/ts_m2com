"""Sphinx configuration file for an LSST stack package.

This configuration only affects single-package Sphinx documentation builds.
"""

from os import getenv

import lsst.ts.m2com
from documenteer.conf.pipelinespkg import *

project = "ts_m2com"
html_theme_options["logotext"] = project  # type: ignore # noqa
html_title = project
html_short_title = project
doxylink = {}  # type: ignore # noqa

intersphinx_mapping["ts_salobj"] = ("https://ts-salobj.lsst.io", None)  # type: ignore # noqa

# Support the sphinx extension of plantuml
extensions.append("sphinxcontrib.plantuml")  # type: ignore # noqa

# Put the path to plantuml.jar
path_plantuml = (
    "/home/saluser/plantuml.jar"
    if getenv("PATH_PLANTUML") is None
    else getenv("PATH_PLANTUML")
)  # type: ignore # noqa
plantuml = f"java -jar {path_plantuml}"
