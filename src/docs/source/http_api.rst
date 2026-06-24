==================
LiDAR HTTP API CLI
==================

The ``mech_driver`` executable can run one HTTP command and exit without
starting UDP or PCAP polling. These commands are intended for LiDAR
configuration, diagnostics and Safety zones maintenance.

Connection options
------------------

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver \
        --lidar-ip 192.168.0.120 \
        --http-port 80 \
        --http-timeout-ms 3000 \
        <command>

``--lidar-ip`` is an alias for ``--http-host``. Defaults are:

* host: ``192.168.0.120``
* port: ``80``
* timeout: ``3000 ms``

The driver does not send Basic Auth credentials. Endpoints or parameters
requiring authorization return an HTTP error such as ``401 Unauthorized``.

Read commands
-------------

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --get-config
    rosrun mech_lidar_driver mech_driver --get-status
    rosrun mech_lidar_driver mech_driver --get-version
    rosrun mech_lidar_driver mech_driver --get-version-string
    rosrun mech_lidar_driver mech_driver --get-log

These commands call:

* ``GET /config.json``
* ``GET /status.json``
* ``GET /version.txt``
* ``GET /version_string.txt``
* ``GET /log.txt``

Configuration writes
--------------------

Change one configuration parameter:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --set-config motor_speed 10
    rosrun mech_lidar_driver mech_driver --set-config motor_speed 0
    rosrun mech_lidar_driver mech_driver --set-config preemptive_conns true
    rosrun mech_lidar_driver mech_driver --set-config logger_level '"INFO"'

The third argument to ``--set-config`` is sent as the raw JSON token body for
``POST /config/{name}``. Strings must include JSON quotes, for example
``'"INFO"'``.

Service commands
----------------

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --save-preset
    rosrun mech_lidar_driver mech_driver --jump-to-bootloader

These commands call:

* ``POST /save_preset.cgi``
* ``POST /jump_to_bootloader.cgi``

Safety zones
------------

Safety zones commands are available only on firmware built with
``FEATURE_SAFETY_ZONES``.

Read current zones, settings and status:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --get-zones
    rosrun mech_lidar_driver mech_driver --get-zone-settings
    rosrun mech_lidar_driver mech_driver --get-zone-status

Write zones and settings. JSON bodies can be passed inline or with ``@file``:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --set-zones @zones.json
    rosrun mech_lidar_driver mech_driver --add-zone @zone.json
    rosrun mech_lidar_driver mech_driver --set-zone-settings @zone_settings.json

Delete, reset and save:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --delete-zone front_sector
    rosrun mech_lidar_driver mech_driver --reset-zones
    rosrun mech_lidar_driver mech_driver --reset-zones front_sector
    rosrun mech_lidar_driver mech_driver --save-zones

Download LUT and stream status events:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --get-zones-lut /tmp/zones_lut.bin
    rosrun mech_lidar_driver mech_driver --safety-events

``--safety-events`` opens ``GET /safety_events`` and prints the server-sent
event stream until the connection closes or the command is interrupted.

Exit status and output
----------------------

Every HTTP command prints the HTTP status line and the response body. The
process returns ``0`` for ``2xx`` responses and non-zero for HTTP errors or
transport errors.

Not implemented by this driver CLI
----------------------------------

The following motor-register and RTOS-task endpoints are intentionally not
exposed by the driver CLI:

* ``/motor_status.json``
* ``/motor_reg/{regIndex}``
* ``/motor_regs?...``
* ``/list_tasks.txt``
* ``POST /motor_regs?...``
