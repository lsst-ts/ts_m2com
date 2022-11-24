===============
Version History
===============

v0.6.2
------

* Update the temperature offset.

v0.6.1
------

* Calculate the temperature inversion matrix.
* Add the **status** to **enableOpenLoopMaxLimit** command.

v0.6.0
------

* Add the **MockErrorHandler** class.
* Add the enums of **LimitSwitchType** and **MockErrorCode**.
* Add the following events:

  * openLoopMaxLimit
  * limitSwitchStatus

v0.5.2
------

* Support the EUI specifc items:

  * Switch digital output command.
  * Configuration event.
  * Tangent force error telemetry.
  * Telescope mount assembly (TMA) inclinometer angle telemetry.
  * Raw power status telemetry.

v0.5.1
------

* Slow down the pace to udpate the actuator steps according to forces in **MockServer** to decrease the CPU usage.

v0.5.0
------

* Add the **ControllerCell** class.

v0.4.4
------

* Ignore and log the errors when run the open-loop control or script.

v0.4.3
------

* Properly reports cause when connecting to non-existing host.
* Increase test timeouts as running those on TSSW Jenkins takes more time than expected (due to limited container resources).

v0.4.2
------

* Add the **.pre-commit-config.yaml**.
* Support the **isort**.

v0.4.1
------

* Fix the conda build.

v0.4.0
------

* Add the **MockControlClosedLoop** class.
* Use the **TS_CONFIG_MTTCS_DIR** to get the configuration files.

v0.3.0
------

* Add the **MockControlOpenLoop** class.
* Support the mock commands of engineering user interface (EUI):

  * Set mirror home
  * Move actuators (under the open-loop control)

v0.2.0
------

* Reorganize the project to have the **mock** module.
* Add the **MockScriptEngine** class.
* Publish the documents.
* Support the mock commands of engineering user interface (EUI):

  * Switch command source
  * Run script
  * Reset breakers
  * Reboot controller
  * Enable open loop maximum limits
  * Save mirror position

v0.1.0
------

* Migrate the codes from `ts_m2 <https://github.com/lsst-ts/ts_m2>`_.
* Rename **Model** class to **Controller** class.
