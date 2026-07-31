Klipper Moonraker API Reference
================================

This document provides a complete reference for all Moonraker-compatible
API endpoints exposed by Tether's Klipper UDS server.

.. contents::
   :depth: 2
   :local:

Overview
--------

Tether's ``KlippyUdsServer`` exposes **135+ JSON-RPC endpoints** over a Unix
domain socket, compatible with the Moonraker API used by frontends like
Mainsail and Fluidd.

Server Endpoints
----------------

=============  ==========================================  =============================
Endpoint       Description                                Parameters
=============  ==========================================  =============================
``server/info``  Get server info and klippy state          --
``server/config``  Get emulated Klipper configuration      --
``server/files/list``  List files in a root               root
``server/files/get``  Get file contents                   root, filename
``server/files/metadata``  Get file metadata              filename
``server/files/directory``  List directory contents        path, root
``server/files/move``  Move/rename a file                 source, dest
``server/files/copy``  Copy a file                        source, dest
``server/files/delete``  Delete a file                    path
``server/files/upload``  Upload a file                    (multipart)
``server/files/roots``  List registered file roots         --
``server/files/create_dir``  Create a directory            path, root
``server/files/metascan``  Scan file metadata              filename
``server/files/thumbnails``  Get file thumbnails            filename
``server/temperature_store``  Get temperature history        --
``server/gcode_store``  Get G-code response history        count
``server/logs/list``  List available log files              --
``server/logs/rollover``  Rollover log files              log
``server/klippy_log``  Get klippy log file info            --
``server/moonraker_log``  Get moonraker log file info     --
=============  ==========================================  =============================

Printer Endpoints
-----------------

================================  ==========================================  =============================
Endpoint                          Description                                Parameters
================================  ==========================================  =============================
``printer/info``                  Get printer info                           --
``printer/subscriptions``         Get/update subscriptions                  objects
``printer/emergency_stop``        Emergency stop                            --
``printer/restart``               Restart klippy                            --
``printer/firmware_restart``      Firmware restart                          --
``printer/gcode/help``            Get G-code help                           --
``printer/gcode/script``          Execute G-code script                     script
``printer/gcode/subscribe_output``  Subscribe to G-code output              --
``printer/objects/list``          List available printer objects            --
``printer/objects/query``         Query printer objects                     objects
``printer/objects/subscribe``     Subscribe to printer objects              objects
``printer/print/start``           Start a print                             filename
``printer/print/cancel``          Cancel current print                      --
``printer/print/pause``           Pause current print                       --
``printer/print/resume``          Resume paused print                       --
================================  ==========================================  =============================

Machine Endpoints
-----------------

=====================================  ==========================================  =============================
Endpoint                               Description                                Parameters
=====================================  ==========================================  =============================
``machine/system_info``                Get system information                     --
``machine/procstats``                  Get process statistics                     --
``machine/system_perms``               Get system permissions                     --
``machine/reboot``                     Reboot the system                          --
``machine/shutdown``                   Shutdown the system                        --
``machine/services/list``              List systemd services                      --
``machine/services/start``             Start a service                            service
``machine/services/stop``              Stop a service                             service
``machine/services/restart``           Restart a service                          service
``machine/update/status``              Get update status                          --
``machine/update/list``                List available updates                     --
``machine/update/refresh``             Refresh update info                        --
``machine/update/update``              Update a component                         component
``machine/update/recover``             Recover a failed update                    component
``machine/device_power/devices``       List power devices                         --
``machine/device_power/state``         Get/set device power state                 device, action
``machine/device_power/on``            Turn device on                             device
``machine/device_power/off``           Turn device off                            device
``machine/device_power/toggle``        Toggle device power                        device
=====================================  ==========================================  =============================

Database Endpoints
------------------

==================  ==========================================  =============================
Endpoint            Description                                Parameters
==================  ==========================================  =============================
``database/list``   List keys in a namespace                   namespace
``database/get``    Get a value                                namespace, key
``database/put``    Put a value                                namespace, key, value
``database/delete`` Delete a value                             namespace, key
``database/ns``     List all namespaces                        --
==================  ==========================================  =============================

Job Queue Endpoints
-------------------

=======================  ==========================================  =============================
Endpoint                  Description                                Parameters
=======================  ==========================================  =============================
``job_queue/status``      Get job queue status                       --
``job_queue/post_job``    Add job(s) to queue                        filename(s)
``job_queue/delete_job``  Remove job from queue                      filename
``job_queue/pause``       Pause the job queue                        --
``job_queue/start``       Start/resume the job queue                 --
``job_queue/jump_to``     Jump to a specific job                     job_id
=======================  ==========================================  =============================

Job History Endpoints
---------------------

========================  ==========================================  =============================
Endpoint                  Description                                Parameters
========================  ==========================================  =============================
``job_history/list``      List job history entries                   limit, start, since, order
``job_history/get``       Get a specific job                         job_id
``job_history/delete``    Delete a job history entry                 job_id, all
========================  ==========================================  =============================

Announcements Endpoints
-----------------------

============================  ==========================================  =============================
Endpoint                      Description                                Parameters
============================  ==========================================  =============================
``announcements/list``        List all announcements                     --
``announcements/update``      Update announcement feeds                  --
``announcements/dismiss``     Dismiss an announcement                    entry_id
``announcements/feed``        Get announcement feed (non-dismissed)      --
============================  ==========================================  =============================

Webcam Endpoints
----------------

==================  ==========================================  =============================
Endpoint            Description                                Parameters
==================  ==========================================  =============================
``webcams/list``    List all webcams                           --
``webcams/get``     Get a specific webcam                      name
``webcams/test``    Test a webcam connection                   name
``webcams/update``  Update webcam configuration                name, url, service, etc.
``webcams/delete``  Delete a webcam                            name
==================  ==========================================  =============================

Device Endpoints
----------------

====================  ==========================================  =============================
Endpoint              Description                                Parameters
====================  ==========================================  =============================
``devices/list``      List all devices                           --
``devices/get``       Get a specific device                      device
``devices/create``    Create a new device                        device, type
``devices/delete``    Delete a device                            device
====================  ==========================================  =============================

Access Endpoints
----------------

============================  ==========================================  =============================
Endpoint                      Description                                Parameters
============================  ==========================================  =============================
``access/login``              User login                                 username, password
``access/logout``             User logout                                --
``access/user``               List/manage users                          --
``access/refresh_jwt``        Refresh JWT token                          username
``access/api_key``            Get API key                                --
``access/oneshot_token``      Generate one-time token                    --
============================  ==========================================  =============================

Bot Endpoints
-------------

==============  ==========================================  =============================
Endpoint        Description                                Parameters
==============  ==========================================  =============================
``bot/list``    List all bots                              --
``bot/get``     Get a specific bot                         name
``bot/update``  Update bot configuration                   name, enabled, token, etc.
``bot/delete``  Delete a bot                              name
==============  ==========================================  =============================

Notepad Endpoints
-----------------

==================  ==========================================  =============================
Endpoint            Description                                Parameters
==================  ==========================================  =============================
``notepad/list``    List all notepad entries                   --
``notepad/get``     Get a notepad entry                        key
``notepad/put``     Create/update a notepad entry              key, value
``notepad/delete``  Delete a notepad entry                     key
==================  ==========================================  =============================

Spoolman Endpoints
------------------

=====================  ==========================================  =============================
Endpoint               Description                                Parameters
=====================  ==========================================  =============================
``spoolman/info``      Get Spoolman connection info               --
``spoolman/spool_id``  Get/set current spool ID                   spool_id
``spoolman/proxy``     Proxy request to Spoolman API              (any)
=====================  ==========================================  =============================

Public C++ API
--------------

In addition to the JSON-RPC endpoints, the ``KlippyUdsServer`` class provides
these public C++ methods for programmatic access:

.. code-block:: cpp

    // Database
    server.databasePut("namespace", "key", JsonValue(42));
    auto val = server.databaseGet("namespace", "key");
    server.databaseDelete("namespace", "key");

    // Job queue
    server.jobQueueAdd("print.gcode");

    // Job history
    int64_t id = server.jobHistoryAdd("print.gcode", "completed");

    // Announcements
    server.announcementAdd("id", "Title", "Description", "info");

    // Webcams
    server.registerWebcam("cam1", "http://...", "mjpegstreamer");

    // Power devices
    server.registerPowerDevice("psu", "on");

    // Services
    server.registerService("klipper", "active", "running");

    // File roots
    server.registerFileRoot("gcodes", "/path/to/gcodes", true);

    // Access control
    server.registerUser("admin", "password", {"read", "write"});

    // Bots
    server.registerBot("telegram", "telegram", "token", true);

    // Notepad
    server.notepadPut("key", "value");
    auto val = server.notepadGet("key");

    // Spoolman
    server.setSpoolmanConnected(true, "http://localhost:8000");
    server.setSpoolId(42);

    // Log files
    server.addLogFile("klippy.log", "/tmp/klippy.log");

    // System permissions
    server.setSystemPerms("admin", {"read", "write", "admin"});

Examples
--------

See the **klipper_moonraker_api** example for a full demonstration of all
endpoints via the public C++ API.
