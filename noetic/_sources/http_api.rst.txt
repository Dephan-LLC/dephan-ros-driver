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
    rosrun mech_lidar_driver mech_driver --get-features
    rosrun mech_lidar_driver mech_driver --get-version
    rosrun mech_lidar_driver mech_driver --get-version-string
    rosrun mech_lidar_driver mech_driver --get-log
    rosrun mech_lidar_driver mech_driver --get-timestamp

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

    rosrun mech_lidar_driver mech_driver --set-config motor_speed 10
    rosrun mech_lidar_driver mech_driver --set-config motor_speed 0
    rosrun mech_lidar_driver mech_driver --set-config preemptive_conns true
    rosrun mech_lidar_driver mech_driver --set-config logger_level '"INFO"'

The third argument to ``--set-config`` is sent as the raw JSON token body for
``POST /config/{name}``. Strings must include JSON quotes, for example
``'"INFO"'``.

Change several configuration parameters with one request:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --set-config-json '{"motor_speed":10,"udp_dest_port":50007}'
    rosrun mech_lidar_driver mech_driver --set-config-json @config_patch.json

``--set-config-json`` sends a flat JSON object to ``POST /config.json``.
Parameters are applied one by one by the device firmware; the operation is not
atomic.

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
event stream until the connection closes or the command is interrupted. A
``zone_status`` event contains ``{"<zone_name>": boolean}`` for zones in the
current monitoring case. Zones assigned to monitoring case ``0`` are omitted.

Zone JSON accepted by ``--set-zones`` and ``--add-zone`` follows the firmware
contract. Each zone must include ``name``, ``monitoring_case`` and
``zone_type``. ``monitoring_case`` is ``0`` for no active monitoring case or
``1..8`` for a concrete case. ``zone_type`` is ``INFO``, ``WARNING`` or
``PROTECTIVE``. Global zone settings may include ``zone_confirm_scans``,
``zone_restart_delay_ms``, ``zone_restart_mode`` and ``zone_current_case``.
Segments can be ``sector``, ``polygon`` or ``two_points``.
The firmware accepts at most 16 zones in total and at most 8 zones in each
non-zero monitoring case. Zone names are limited to 47 bytes plus a terminating
null byte.

The LUT file starts with a one-byte zone count for the current monitoring case.
Each zone then occupies 48 bytes for its null-padded name followed by 2300
little-endian ``uint32`` distance thresholds in millimetres. Its total size is
``1 + zone_count * (48 + 2300 * 4)`` bytes; it is not a triggered-state
snapshot.

Log events
----------

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --log-events

``--log-events`` opens ``GET /log_events`` and prints new log server-sent
events until the connection closes or the command is interrupted. Use
``--get-log`` first if an initial log snapshot is required.

Contamination analysis
----------------------

These commands are available only on firmware built with
``FEATURE_CONTAMINATION_ANALYSIS``:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --get-contamination-status
    rosrun mech_lidar_driver mech_driver --contamination-events

The first command reads ``GET /contamination/status.json``. The second opens
the ``GET /contamination_events`` server-sent event stream until it is closed
or interrupted. A ``contamination_status`` event carries
``{"contaminated":boolean,"percent":number}`` and is also repeated as a
status snapshot approximately every two seconds. Firmware without this feature
returns ``404 Not Found``.

The analysis sector and threshold use the regular configuration commands:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --set-config contamination_angle_start_deg 0.0
    rosrun mech_lidar_driver mech_driver --set-config contamination_angle_end_deg 90.0
    rosrun mech_lidar_driver mech_driver --set-config contamination_threshold_percent 30

Angles must be in the ``0..359.9`` degree range. The threshold is an integer
percentage in the ``0..100`` range.

Exit status and output
----------------------

Every HTTP command prints the HTTP status line and the response body. The
process returns ``0`` for ``2xx`` responses and non-zero for HTTP errors or
transport errors.
