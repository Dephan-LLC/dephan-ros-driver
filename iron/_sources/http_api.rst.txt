==================
LiDAR HTTP API CLI
==================

The ``mech_driver`` executable can run one HTTP command and exit without
starting UDP or PCAP polling. These commands are intended for LiDAR
configuration, diagnostics and Safety zones maintenance.

Connection options
------------------

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver \
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

    ros2 run mech_lidar_driver mech_driver --get-config
    ros2 run mech_lidar_driver mech_driver --get-status
    ros2 run mech_lidar_driver mech_driver --get-features
    ros2 run mech_lidar_driver mech_driver --get-version
    ros2 run mech_lidar_driver mech_driver --get-version-string
    ros2 run mech_lidar_driver mech_driver --get-log
    ros2 run mech_lidar_driver mech_driver --get-timestamp

These commands call:

* ``GET /config.json``
* ``GET /status.json``
* ``GET /features.json``
* ``GET /version.txt``
* ``GET /version_string.txt``
* ``GET /log.txt``
* ``GET /timestamp.txt``

Configuration writes
--------------------

Change one configuration parameter:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --set-config motor_speed 10
    ros2 run mech_lidar_driver mech_driver --set-config motor_speed 0
    ros2 run mech_lidar_driver mech_driver --set-config preemptive_conns true
    ros2 run mech_lidar_driver mech_driver --set-config logger_level '"INFO"'

The third argument to ``--set-config`` is sent as the raw JSON token body for
``POST /config/{name}``. Strings must include JSON quotes, for example
``'"INFO"'``.

Change several configuration parameters with one request:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --set-config-json '{"motor_speed":10,"udp_dest_port":50007}'
    ros2 run mech_lidar_driver mech_driver --set-config-json @config_patch.json

``--set-config-json`` sends a flat JSON object to ``POST /config.json``.
Parameters are applied one by one by the device firmware; the operation is not
atomic.

Service commands
----------------

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --save-preset
    ros2 run mech_lidar_driver mech_driver --jump-to-bootloader

These commands call:

* ``POST /save_preset.cgi``
* ``POST /jump_to_bootloader.cgi``

Safety zones
------------

Safety zones commands are available only on firmware built with
``FEATURE_SAFETY_ZONES``.

Read current zones, settings and status:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --get-zones
    ros2 run mech_lidar_driver mech_driver --get-zone-settings
    ros2 run mech_lidar_driver mech_driver --get-zone-status

Write zones and settings. JSON bodies can be passed inline or with ``@file``:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --set-zones @zones.json
    ros2 run mech_lidar_driver mech_driver --add-zone @zone.json
    ros2 run mech_lidar_driver mech_driver --set-zone-settings @zone_settings.json

Delete, reset and save:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --delete-zone front_sector
    ros2 run mech_lidar_driver mech_driver --reset-zones
    ros2 run mech_lidar_driver mech_driver --reset-zones front_sector
    ros2 run mech_lidar_driver mech_driver --save-zones

Download LUT and stream status events:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --get-zones-lut /tmp/zones_lut.bin
    ros2 run mech_lidar_driver mech_driver --safety-events

``--safety-events`` opens ``GET /safety_events`` and prints the server-sent
event stream until the connection closes or the command is interrupted.

Zone JSON accepted by ``--set-zones`` and ``--add-zone`` follows the firmware
contract. Each zone must include ``name``, ``monitoring_case`` and
``zone_type``. ``monitoring_case`` is ``0`` for no active monitoring case or
``1..8`` for a concrete case. ``zone_type`` is ``INFO``, ``WARNING`` or
``PROTECTIVE``. Global zone settings may include ``zone_confirm_scans``,
``zone_restart_delay_ms``, ``zone_restart_mode`` and ``zone_current_case``.
Segments can be ``sector``, ``polygon`` or ``two_points``.

Log events
----------

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --log-events

``--log-events`` opens ``GET /log_events`` and prints new log server-sent
events until the connection closes or the command is interrupted. Use
``--get-log`` first if an initial log snapshot is required.

Exit status and output
----------------------

Every HTTP command prints the HTTP status line and the response body. The
process returns ``0`` for ``2xx`` responses and non-zero for HTTP errors or
transport errors.
