===============
Version History
===============

v0.9.2
------

* Calculate the actuator displacements based on the rigid body movement.
* Calculate the rigid body position based on the hardpoint displacements.

v0.9.1
------

* Remove the **root** workaround from **Jenkinsfile**.

v0.9.0
------

* Adapt the **ts_tcpip** v1.0.0:

  * Use the **LOCALHOST_IPV4** instead of **LOCAL_HOST**.
  * Put the ``MockServer._connect_state_changed_callback_command()`` and ``MockServer._connect_state_changed_callback_telemetry()`` to be asynchronous.

v0.8.2
------

* Calculate the hardpoint compensation matrix instead of reading the related file.

v0.8.1
------

* Update the ``MockCommand.switch_force_balance_system()`` to drop the **TelemetryOnly** from **ClosedLoopControlMode** event.
* Reports digital input & output, force balance system status, and open-loop maximum limit when ``MockCommand.power()`` is called.

v0.8.0
------

* Support to reset the actuator forces and steps.
* Simulate the inner-loop controller.
* Set the closed-loop control mode.
* Set the inner-loop control mode.

v0.7.0
------

* Move the constants of force limit to submodule.
* Add the **MockPowerSystem** class.

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
