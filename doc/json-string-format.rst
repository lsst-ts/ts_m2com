.. _JSON_String_Format:

#########################
JSON String Format
#########################

The format of JSON string in TCP/IP communication is defined here.

.. _Header:

Header
======

Command
-------

When the M2 CSC/GUI sends the command, the JSON string should have the following keys:

* **id**: Specify the name of command that begins with *cmd_*.
* **sequence_id**: A unique ID that the value increases from 0 to N among different commands. This item is unrelated to the command name or details. It is used to track the internet traffic.

Once the TCP/IP server (e.g. M2 cell controller) received the command or the M2 cell controller finished the command, the related information will be sent to the M2 CSC/GUI.
To support this, the following header information is needed:

* **id**: Specify the command status. The value should be in *ack*, *noack*, *success*, or *fail*. The *ack* or *noack* is by the TCP/IP module directly. The *success* or *fail* will wait for the response of the M2 cell controller.

When the TCP/IP server responds to the M2 CSC/GUI, the following information is required: **id** and **sequence_id**.

Event
-----

The JSON string should have the following keys:

* **id**: Specify the name of event that begins with *evt_*.
* **compName**: Specify which component's event is being subscribed (optional). The M2 cell controller needs this to identify the subscribed event from other components (by SAL). But when the M2 cell controller publishes the event, this key is not required.

Telemetry
---------

The JSON string should have the following keys:

* **id**: Specify the name of telemetry that begins with *tel_*.
* **compName**: Specify which component's telemetry is being subscribed (optional). The M2 cell controller needs this to identify the subscribed telemetry from other components (by SAL). But when the M2 cell controller publishes the telemetry, this key is not required.

.. _Command_Acknowledgement:

Command Acknowledgement
=======================

The M2 cell controller should acknowledge the M2 CSC/GUI actively for the command status with *ack* if:

1. The command is in the list of registered commands by the M2 cell controller.
2. The **sequence_id** is bigger than the previous by one if it is not the first command. Let the M2 CSC/GUI decides the first value of **sequence_id**.

Otherwise, the *noack* should be used.
In the condition 2 above, if the M2 cell controller received a command with **sequence_id: 1** followed by **sequence_id: 3**, it should do the *noack* as the following and vice versa:

.. code-block:: json

    {
        "id": "noack",
        "sequence_id": 2
    }

.. _Details_Parameters:

Details of the Parameters
=========================

The parameter details should be inline with the JSON string.
For example, the M2 CSC/GUI might issue a *move* command with *x*, *y*, and *z* as the following:

.. code-block:: json

    {
        "id": "cmd_move",
        "sequence_id": 1,
        "x": 0.1,
        "y": 0.2,
        "z": 0.3
    }

The M2 CSC/GUI might issue multiple commands in a short time, but the M2 cell controller only executes a single command in a single time and reply the result.

Take the above command as an example, the acknowledgement from the M2 cell controller at each time of receiving the command will be:

.. code-block:: json

    {
        "id": "ack",
        "sequence_id": 1
    }

If this command succeeds after the execution by M2 cell controller, the following message will be issued to the M2 CSC/GUI:

.. code-block:: json

    {
        "id": "success",
        "sequence_id": 1
    }

In some cases, the parameters might not be needed.
For example, the M2 cell controller might publish a inPosition event:

.. code-block:: json

    {
        "id": "inPosition"
    }

The M2 cell controller might subscribe the *inPosition* event from *MTMount* component with the parameter of *tolerance* (the M2 CSC/GUI should send this message to the M2 cell controller):

.. code-block:: json

    {
        "id": "evt_inPosition",
        "compName": "MTMount",
        "tolerance": 0.001
    }

The telemetry is similar to the event.
The main difference is that the telemetry will be in a fixed rate (in the ideal case).
