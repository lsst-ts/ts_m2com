===============
Version History
===============

v1.5.12 (2025-06-09)
====================

New Features
------------

- Support the towncrier. (`OSW-498 <https://rubinobs.atlassian.net//browse/OSW-498>`_)
- Update the setup.py to support the Python 3.12 only. (`OSW-498 <https://rubinobs.atlassian.net//browse/OSW-498>`_)


Bug Fixes
---------

- Improve the unit test. (`OSW-498 <https://rubinobs.atlassian.net//browse/OSW-498>`_)

v1.5.11
-------

* Fix the import of version.

v1.5.10
-------

* Improve the ``setup.py`` to support the version of Python 3.11 and 3.12.

v1.5.9
------

* Remove the sleep times in **MockCommand** and **MockPowerSystem**.
* Refactor the checking of command status in **Controller**.

v1.5.8
------

* Remove the **ts_idl**.

v1.5.7
------

* Send the ILC modes in the ``MockServer._send_welcome_message()``.

v1.5.6
------

* Update the **LIMIT_FORCE_TANGENT_CLOSED_LOOP** in ``constant.py`` to be 5800.0 N

v1.5.5
------

* Update the ``ErrorHandler.__init__()`` to have the optional argument to read the error code file.

v1.5.4
------

* Check the temperature sensor if the temperature LUT is applied.

v1.5.3
------

* Add the ``Controller.get_ilc_modes()`` and ``Controller.set_ilc_modes_to_nan()``.
* Fix the bug of ``MockCommand.get_inner_loop_control_mode()`` and ``MockCommand.set_inner_loop_control_mode()`` that the ILC is related to the communication power instead of the motor power.

v1.5.2
------

* Send the power status in the welcome message.

v1.5.1
------

* Add the functions to get the power status.

v1.5.0
------

* Add the **ILC_READ_WARNING_ERROR_CODES** in ``constant.py``.
* Add the ``MockMessageEvent.write_bypassed_actuator_ilcs()`` and update the ``MockServer._send_welcome_message()``.
* Allow the bypass of temperature LUT calculation in ``MockControlClosedLoop.calc_look_up_forces()``.
* Update the ``Controller`` class to have the **bypassed_ilcs** attribute.
* Make the function of ``Controller.switch_force_balance_system()`` to be robust.

v1.4.6
------

* Update the version of ts-conda-build to 0.4 in the conda recipe.

v1.4.5
------

* Use the **mermaid** to replace the **PlantUML**.

v1.4.4
------

* Improve the **force-control-loop.rst**.
* Add the **hardpoint-compensation.rst** and **lut-gravity.rst**.

v1.4.3
------

* Update the enum **DigitalOutput** bit 5 to be **ClosedLoopControl**.

v1.4.2
------

* Change the **TANGENT_LINK_LOAD_BEARING_LINK** and **TANGENT_LINK_NON_LOAD_BEARING_LINK** to be consistent with the controller.

v1.4.1
------

* Add the **force-control-loop.rst**, **calc-pos-ims.rst**, and **tan-fault-det.rst**.

v1.4.0
------

* Update the mock module to use the **MockControlLoop**.
* Add the **MockInPosition** class.
* Improve the rigid body movement.

v1.3.0
------

* Improve the doc string of ``MockControlClosedLoop.calc_hp_comp_matrix()``.
* Fix the ``MockModel._uniq_ilc_status_generator()``.
* Add the **MockPlant**, **MockControlLoop**, **SingleBiquadraticFilter**, **BiquadraticFilter**, **SimpleDelayFilter**, **MockDeadbandControl**, and **MockGainSchedular** classes.

v1.2.0
------

* Improve the readibility of the ``MockControlClosedLoop.calc_temp_inv_matrix()``.
* Improve the architecture and performance.

v1.1.7
------

* Add the ``MockControlClosedLoop.update_hardpoints()`` and ``MockCommand.set_hardpoint_list()`` methods.
* Use the corrected angle in ``MockModel._calculate_force_error_tangent()``.
* Fix the ``MockModel._simulate_zenith_angle()`` for the calculation of zenith angle.

v1.1.6
------

* Move the ``check_hardpoints()`` and ``select_axial_hardpoints()`` to **utility.py** from ``MockControlClosedLoop`` class.

v1.1.5
------

* Update the ``.ts_pre_commit_config.yaml``.
* Calculate the dynamic kinetic matrix.
* Calculate the command pre-filter, delay filter, and force cotrol filter parameters.

v1.1.4
------

* Remove the legacy code.
* Use the enums in **ts_xml** instead of **ts_idl**.

v1.1.3
------

* Use the enums in **ts_idl** and remove the duplication in **enum.py**.
* Publish the same telemetry for CSC and EUI.

v1.1.2
------

* Move the ``TabAlarmWarn._calc_enabled_faults_mask()`` in **ts_m2gui** to ``ErrorHandler.calc_enabled_faults_mask()``.
* Add the try-catch loop when processing the event and telemetry.
* Fix the event of interlock.

v1.1.1
------

* Fix the typo of enum: **DigitalInput**.
* Add the constant: **OUTLIER_INCLINOMETER_RAW**.
* Fix the bug of ``Controller.set_ilc_to_enabled()`` for the unknown state.
* Add some auxiliary functions to **Controller** class.

v1.1.0
------

* Migrate the functions from **ts_m2gui**.

v1.0.0
-------

* Adapt the **ts_tcpip** v1.2.0.

v0.12.0
-------

* Fix the bug of external elevation angle, which should have the same coordinate system as the look-up table angle.
The related error code is added.
* Support the control parameters in **MockModel** class.
* Fix the ``MockCommand.clear_errors()`` for GUI.
* Fix the ``MockModel.fault()`` that the error codes might be bypassed.
* Fix the deprecation warning in **test_tcp_client.py** to access the **reader** and **writer** of **BaseClientOrServer** in **ts_tcpip** module directly.

v0.11.0
-------

* Migrate the functions to **Controller** class from **ts_m2**.

v0.10.7
-------

* Use the reversed direction of hardpoint correction in ``MockControlClosedLoop`` class.

v0.10.6
-------

* Add the ``MockMessageEvent.write_configuration_files()`` and the ``MockMessageEvent.configuration_file`` attribute.
* Allow the controller to set the configuration file.

v0.10.5
-------

* Adapt the behaviour that the M2 cell LabVIEW project only reports the summary faults status instead of error code.
* Simulate the enabled faults mask.

v0.10.4
-------

* Fix the rz calculation in ``MockControlClosedLoop.hardpoint_to_rigid_body()``.
* Add the ``MockModel.is_force_error_tangent_out_limit()`` and do the related check in ``MockServer._check_error_force()``.

v0.10.3
-------

* Add the ``MockCommand.report_interlock()`` to report the interlock event.
* Update the ``MockServer._send_welcome_message()`` to send the lost connection error at welcome message.

v0.10.2
-------

* Add the ``Controller.set_control_parameters()`` to set the control parameters of closed-loop controller (CLC).
* Do not check the communication power status (True/False) in ``Controller._callback_check_power_status()`` because sometimes, the cRIO simulator might put it on even though it should be off theoretically.
* Allow to change the status of bit value of digital output.
* Use the internal annotation instead of importing the **typing** module.

v0.10.1
-------

* Adapt the **.ts_pre_commit_config.yaml**.

v0.10.0
-------

* Add the **ErrorHandler** class and support the reading of summary faults status.
* Update the **MockErrorHandler** class to be inherited from **ErrorHandler**.
* In the Python simulator, when the motor power is on, the event of summary faults status will be sent to simulate the event from cell controller.
* Update the ``MockControlClosedLoop.is_actuator_force_out_limit()`` to have the option of using the measured forces.
* Only trigger the error of limit switch if the open-loop maximum is enabled.
* Allow the bypass of state checking in ``Controller.clear_errors()``.

v0.9.7
------

* Support the mypy.

v0.9.6
------

* Fix the bug to run the simulation mode with mock server on summit.

v0.9.5
------

* Adapt black v23.1.0.

v0.9.4
------

* Calculate the rigid body position based on the hardpoint displacements in ``MockModel``.
* Put the ``MockModel.handle_position_mirror()`` as a placeholder at this moment.
Need to translate the forward modeling of hardpoint correction first before the realization of this function.

v0.9.3
------

* Increase the default timeout from 10 sec to 20 sec in ``Controller.power()``.
* Add the ``MockCommand.load_configuration()`` and ``Controller.load_configuration()``.

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
