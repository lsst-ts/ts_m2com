.. |CSC_developer| replace:: *Tiago Ribeiro <tribeiro@lsst.org>* and *Te-Wei Tsai <ttsai@lsst.org>*
.. |CSC_product_owner| replace:: *Sandrine Thomas <sthomas@lsst.org>*

.. Note that the ts_ prefix is omitted from the title

########################
M2 Common Code
########################

.. image:: https://img.shields.io/badge/GitHub-ts__m2com-green.svg
    :target: https://github.com/lsst-ts/ts_m2com
.. image:: https://img.shields.io/badge/Jenkins-ts__m2com-green.svg
    :target: https://tssw-ci.lsst.org/job/LSST_Telescope-and-Site/job/ts_m2com
.. image:: https://img.shields.io/badge/Jira-ts__m2com-green.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels%20in%20(ts_m2com%2C%20%20M2)

.. _Overview:

Overview
========

This module contains the common code for the main telescope M2 commandable SAL component (CSC) and graphical user interface (GUI) to operate the M2 mirror control system.
The `eups <https://github.com/RobertLuptonTheGood/eups>`_ is used as the package manager.
This package also supports the `conda <https://docs.conda.io/en/latest>`_ package manager.

The badges above navigate to the GitHub repository for the common code and Jira issues.

.. _Dependencies:

Dependencies
============

* `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_
* `ts_tcpip <https://github.com/lsst-ts/ts_tcpip>`_
* `ts_utils <https://github.com/lsst-ts/ts_utils>`_

.. _JSON_String:

JSON String
===========

The `JSON <https://www.json.org>`_ string is used in the TCP/IP communication between the M2 CSC/GUI and hardware.
The design here takes the following documents as references: `TS JSON Message Format Proposal <https://confluence.lsstcorp.org/display/LTS/TS+JSON+Message+Format+Proposal>`_ and `TS JSON Message Format Proposal <https://confluence.lsstcorp.org/display/LTS/TS+JSON+Message+Format+Proposal>`_.

.. toctree::
    json-string-format
    :maxdepth: 1

.. _Architecture:

Architecture
=============

The classes and files for each module are listed below.

.. _lsst.ts.m2com-modules_m2com:

m2com
-------------

.. uml:: uml/class_m2com.uml
    :caption: Class diagram of M2 common code

* **Controller** has the business logic to communicate with hardware by TCP/IP interface.
* **TcpClient** is a TCP/IP client.

.. _lsst.ts.m2com-modules_m2com_mock:

m2com.mock
-------------

.. uml:: uml/mock/class_mock.uml
    :caption: Class diagram of mock module in M2 common code

* **MockServer** is a mock server of M2 to support the simulation mode.
* **MockModel** simulates the hardware behavior to be used by **MockServer**.
* **MockScriptEngine** simulates the execution of binary script.
* **MockControlOpenLoop** simulates the open-loop control.
* **MockCommand** simulates the execution of command in real hardware.
* **MockMessageEvent** simulates the message of event from real hardware.
* **MockMessageTelemetry** simulates the message of telemetry from real hardware.

.. _API:

APIs
=============

This section is autogenerated from docstrings.

.. automodapi:: lsst.ts.m2com
    :no-inheritance-diagram:

.. _Build_And_Test:

Build and Test
==============

Using `docker <https://www.docker.com>`_ is highly recommended.
The built docker image is `develop_env <https://hub.docker.com/repository/docker/lsstts/develop-env>`_.

To setup and test the code using Docker, enter:

.. code-block:: bash

    docker run -it --rm -v ${repo_location}:/home/saluser/ts_m2com lsstts/develop-env:${tag}
    cd ts_m2com
    setup -k -r .

The environment shall be setup either from */home/saluser/.setup_dev.sh*.

.. _Version_History:

Version History
===============

The version history is at the following link.

.. toctree::
    version_history
    :maxdepth: 1

The released version is `here <https://github.com/lsst-ts/ts_m2com/releases>`_.

.. _Contributing:

Contributing
============

To contribute, please start a new pull request on `GitHub <https://github.com/lsst-ts/ts_m2com>`_.
Feature requests shall be filled in JIRA with the *ts_m2com* or *M2* label.
In all cases, reaching out to the :ref:`contacts for this CSC <Contact_Personnel>` is recommended.

.. _Contact_Personnel:

Contact Personnel
=================

For questions not covered in the documentation, emails should be addressed to the developers: |CSC_developer|.
The product owner is |CSC_product_owner|.

This page was last modified |today|.
