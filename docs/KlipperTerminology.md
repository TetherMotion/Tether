# tether_klippy: Terminology and Sequencing Reference

This document provides extensive terminology definitions and sequencing
specifications for the `tether_klippy` server's Moonraker-facing interface.
It is derived solely from the cleanroom protocol specification at
`/home/uli/dev/Metexon/klipper_moonraker_docs/` and does not reference any
external source code.

## 1. Terminology

### 1.1 Roles

| Term | Definition |
|------|------------|
| **Server** | The `tether_klippy` process. It creates the listening Unix domain socket, receives requests, emits asynchronous notifications, and is the authority on printer state and motion control. In the wire protocol, this is the "Klipper" role. |
| **Client** | The Moonraker process (or any compatible client). It connects to the socket, issues requests, consumes notifications, and is the authority on HTTP/WebSocket front-end traffic, file management, and job orchestration. |
| **Connection** | A single accepted TCP-like stream on the Unix domain socket. Each connection is independent and identified internally by the Server. The protocol is fully symmetric with respect to which connection a notification is delivered on: a subscription or remote-method registration is bound to the connection that created it. |
| **Front-end client** | A downstream consumer of the Client (e.g. a web UI, a mobile app, or an API integrator). Front-end clients never talk to the Server directly; they go through the Client. |

### 1.2 Transport and Framing

| Term | Definition |
|------|------------|
| **Unix domain socket** | An `AF_UNIX` stream socket (`SOCK_STREAM`) used as the transport. The conventional default path is `/tmp/klippy_uds`, supplied as a command-line argument. |
| **ETX byte** | The single byte `0x03` (ASCII "End of Text"). It is the sole frame delimiter on the byte stream. |
| **Frame** | The UTF-8 byte sequence of exactly one JSON document, terminated by a trailing `0x03` byte. A stream is a concatenation of frames: `<frame_1><0x03><frame_2><0x03>...`. |
| **Partial buffer** | A receiver-side buffer (`partial`) that accumulates bytes from reads that did not end on a frame boundary. The buffer is split on `0x03`; the final segment after the last `0x03` is retained as the new partial buffer for the next read. |
| **Compact JSON** | The JSON dialect used on the wire: item separator `,`, key separator `:`, no insignificant whitespace. A receiver should accept any valid JSON regardless of whitespace, but a conformant sender emits compact JSON. |
| **Top-level type** | Every frame must be a JSON object (dictionary) at the top level. A frame that decodes to any other JSON type (array, string, number, etc.) is malformed and must be ignored (logged and skipped; the connection is not torn down). |
| **Backpressure** | The condition where a Client is not draining the socket fast enough, causing the Server's send buffer to grow. The Server monitors write-readiness and forcibly closes persistently slow clients. |
| **Read buffer** | The Client's socket receive buffer. A large size (20 MiB) is recommended to avoid spurious backpressure on connections that are mostly idle but occasionally receive large status snapshots. |

### 1.3 Message Types

| Term | Definition |
|------|------------|
| **Request** | A Client→Server message carrying `id`, `method`, and optional `params`. The Server processes it and, if `id` is present and non-null, sends a Response. |
| **Notification** | A Client→Server message with no `id` (or `id: null`). The Server processes it but sends no response. Fire-and-forget. |
| **Response** | A Server→Client message echoing a Request's `id`, carrying either `result` (success) or `error` (failure). Never carries `method`. |
| **Push message** | A Server→Client asynchronous notification carrying `method` and `params` but no `id`. Emitted by subscriptions and remote-method callbacks. Expects no reply. |
| **`id`** | A JSON value of any type, supplied by the Client in a Request and echoed verbatim by the Server in the Response. Used for correlation. Must be unique among all currently in-flight (not yet answered) requests on the same connection. The Server treats it opaquely. |
| **`method`** | A string naming the endpoint to invoke. Used verbatim as the `method` field; there is no namespace prefixing on the wire. Conventionally of the form `<module>/<name>` (e.g. `gcode/script`). |
| **`params`** | An optional dictionary in a Request providing endpoint-specific arguments. Defaults to `{}` if absent. Must be a dictionary; any other type is a malformed request. |
| **`result`** | A dictionary in a success Response. The Server guarantees it is a dictionary; if the handler produced no explicit result, the Server substitutes `{}`. |
| **`error`** | A dictionary in a failure Response, present instead of `result`. Contains `message` (human-readable string) and `error` (symbolic category, currently always `"WebRequestError"`). |
| **`WebRequestError`** | The only defined error category. Covers any failure arising from argument validation, G-code execution error, or an internal exception in a handler. There is no numeric error code on the wire. |
| **Internal-error escalation** | If a handler raises an unexpected exception (not a normal command error), the Server replies with an `error` response AND transitions the whole printer to the `shutdown` state. This is a safety measure: an unhandled exception is treated as a firmware fault. |

### 1.4 Endpoints

| Term | Definition |
|------|------------|
| **Endpoint** | A named request handler on the Server side, addressed by the `method` field of a Request. |
| **Core endpoint** | Always present, registered by the Server's webhooks subsystem: `info`, `emergency_stop`, `register_remote_method`, `list_endpoints`. |
| **G-code endpoint** | Always present: `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart`, `gcode/subscribe_output`. |
| **Query/subscribe endpoint** | Always present: `objects/list`, `objects/query`, `objects/subscribe`. |
| **Module endpoint** | Registered by individual firmware modules; presence depends on printer configuration. Examples: `pause_resume/pause`, `query_endstops/status`, `bed_mesh/dump_mesh`, `motion_report/dump_stepper`. Discovered at runtime via `list_endpoints`. |
| **Reserved endpoint** | An endpoint that must not be exposed by the Client to its front-end clients: `list_endpoints`, `gcode/subscribe_output`, `register_remote_method`. These are infrastructure used internally by the Client. |
| **Mux endpoint** | A single endpoint name that dispatches to different handlers based on the value of a key parameter (e.g. `sensor` or `name`). The wire contract is unchanged: the Client supplies the selector in `params`; the Server routes internally. Listed once by `list_endpoints`, not once per instance. |
| **Remote endpoint** | A Client-side forwarding of a Server endpoint to the Client's own HTTP/WS API. The handler does no local processing; it forwards the request to the Server and returns the response. |

### 1.5 Subscriptions

| Term | Definition |
|------|------------|
| **Subscription** | A long-lived, per-connection arrangement by which the Server pushes asynchronous updates to a Client. The primary mechanism for observing printer state without polling. |
| **`response_template`** | A dictionary supplied by the Client in a subscription request, stored verbatim by the Server as the envelope for all future push messages. When pushing, the Server takes a shallow copy of the template, inserts a `params` key with the payload, and sends the result. Defaults to `{}` if omitted. |
| **Push message** (subscription) | An asynchronous Server→Client notification constructed from the stored `response_template` plus a `params` payload. Carries no `id`; expects no reply. |
| **Coalescing** | The Server does not push on every field change. Instead, a periodic refresh (every 0.25 s) computes the diff of each subscriber's requested fields against the last-pushed baseline and pushes only the differences. When no fields changed, no push is emitted. |
| **Refresh interval** | The fixed 0.25-second period at which the Server recomputes status diffs for all subscribers. |
| **Diff** | A per-subscriber dictionary containing only the fields whose values changed since the last push. Unchanged fields are omitted; objects with no changes are omitted entirely. |
| **Baseline** | The "last pushed" values per subscriber, against which the next refresh computes the diff. Updated to current values after each push. |
| **Snapshot** | A full status query result (all requested fields at current values), returned as the response to `objects/subscribe` and `objects/query`. Contrasted with diffs, which carry only changed fields. |
| **`webhooks` object** | A printer object exposing two fields: `state` and `state_message`. Subscribing to it is the canonical way to observe Server state transitions as push messages. |
| **Timer lifecycle** | The periodic refresh timer is started when the first subscription is created and stopped when there are no subscriptions left (all subscribers disconnected or unsubscribed). This keeps CPU usage at zero when nobody is subscribed. |

### 1.6 Remote Methods

| Term | Definition |
|------|------------|
| **Remote method** | The inverse of a request: lets the Server invoke a callback registered by the Client. Used by firmware code that needs to notify or request services from the front-end world. |
| **`register_remote_method`** | The endpoint by which a Client registers a remote method. The Server records, for the connection that issued the request, the association: method name → response_template. |
| **Per-method connection map** | The Server's data structure mapping each method name to a set of `{connection → template}` entries. The same method name may be registered by multiple connections simultaneously. |
| **Fan-out** | When a remote method is invoked, the Server delivers the push message to all connections that registered the method. There is no aggregation, ordering, or "first responder wins" semantics. |
| **Fire-and-forget** | A remote-method invocation is a notification (push message). The Server does not expect, and the Client must not send, any response. |
| **Built-in methods** | Two method names wired up via subscription templates rather than explicit registration: `process_gcode_response` (G-code output) and `process_status_update` (status diffs). |

### 1.7 State Machine

| Term | Definition |
|------|------------|
| **`startup`** | Server state: initializing (reading config, identifying MCUs, connecting to them). Not ready to accept motion commands. |
| **`ready`** | Server state: initialization succeeded; the printer is ready to accept G-code and motion commands. |
| **`error`** | Server state: initialization failed (config error, MCU protocol error, MCU connect error, or internal error during startup). Halted; will not become ready without a restart. Exclusively a startup-failure state. |
| **`shutdown`** | Server state: a fatal condition occurred after `ready` (or during runtime): emergency stop, unhandled exception in a handler, or MCU move-fault. The Server has stopped motion and is in a safe, halted state. |
| **`disconnected`** | Client-side pseudo-state when the socket is not connected. Not a Server state. |
| **`state_message`** | A human-readable string accompanying each state. Authoritative explanation of the current state. Should be surfaced to operators when the state is not `ready`. |
| **State transition** | A change from one Server state to another. Only four transitions are defined: `startup→ready`, `startup→error`, `ready→shutdown`, `error→shutdown` (no-op). There is no direct `ready→error` transition; runtime faults go to `shutdown`. Recovery from `error` or `shutdown` requires a process restart. |

### 1.8 Handshake and Lifecycle

| Term | Definition |
|------|------------|
| **Handshake** | The four-phase sequence by which a Client establishes a connection and brings itself to a fully operational state: socket acquisition, identification, readiness, method registration. |
| **Socket acquisition** | Handshake phase 1: wait for the socket file to exist and be connectable. Loops with a 0.25 s sleep. |
| **Identification** | Handshake phase 2: send the first `info` request with `client_info`, record the Server's identity (version, paths, pid). Call `list_endpoints` and register non-reserved endpoints. |
| **Readiness** | Handshake phase 3: poll `info` until the Server leaves `startup`, then set up initial subscriptions (`webhooks`, `gcode/subscribe_output`) and re-discover endpoints. |
| **Method registration** | Handshake phase 4: once the Server is `ready`, verify required printer objects, resolve the G-code path, and register remote methods. |
| **Phase flags** | Boolean trackers for each handshake phase, enabling reconnection to resume from the appropriate phase. |
| **`client_info`** | An optional dictionary in the first `info` request identifying the Client (program name and version). The Server records it for logging/diagnostics but does not validate its structure. |
| **Reconnection** | Automatic, unconditional establishment of a new connection after an unexpected disconnect. The Client starts a new connection task and runs the full handshake from scratch. |
| **Connection-closed handler** | The Client-side routine invoked on unexpected disconnect: resets handshake flags, fails pending requests, clears subscriptions and cache, emits `klippy_disconnect`, and starts reconnection. |
| **Idempotent handshake** | Every reconnect re-derives identification, re-subscribes, re-registers endpoints, and re-registers remote methods from scratch. The Server does not retain any per-Client state across a disconnect. |
| **Persistent state** | A small amount of Client-side state (Server source path, Python path, service info) stored across reconnects. An optimization; the wire protocol does not depend on it. |

### 1.9 Request Correlation

| Term | Definition |
|------|------------|
| **Pending-request table** | A per-connection map `{id → {waitable, method_name}}` maintained by the Client. The read loop removes entries on response; the write path adds entries on request. |
| **Correlation by `id`** | Responses are matched to requests by the `id` field, not by arrival order. A Response never carries `method`; a Push never carries `id`. There is no ambiguity. |
| **Late response** | A response that arrives after the request timed out and was removed from the pending table. Handled by the "not found" path: logged and discarded. |
| **Default timeout** | 60 seconds for ordinary requests. If the Server does not respond, the request fails with a timeout error. |
| **Subscription timeout** | 20 seconds for the initial `objects/subscribe` snapshot response (gathering a full status snapshot can take longer). The ongoing push stream has no timeout. |
| **Indefinite-wait mode** | A request issued with no explicit timeout. The Client waits forever but logs a "still pending" notice every 60 s. Used for internal calls that must not time out. |

### 1.10 Subscription Aggregation

| Term | Definition |
|------|------------|
| **Subscription aggregation** | The Client-side algorithm that multiplexes many front-end subscriptions onto a single Server subscription (the union), then fans out incoming diffs to each front-end client filtered to that client's requested fields. |
| **Union subscription** | The single `objects/subscribe` request sent to the Server, whose object set is the union of all front-end clients' requested objects and fields. |
| **`subscription_cache`** | The Client's view of full printer status `{object: {field: value}}`, kept fresh by the Server's push messages. Used for diffing and per-client filtering. |
| **`exclusions`** | A map of object fields that are never cached (e.g. `configfile.config`, `configfile.settings`) because they are large and never change after startup. Still delivered to subscribers; just not stored in the cache. |
| **Lazy recompute** | When a front-end client disconnects, the Client does not immediately recompute and re-send the union. The union is recomputed only when the next front-end client subscribes. Avoids bursts of `objects/subscribe` requests on mass disconnect. |
| **Host-internal subscription** | A subscription by a Client-internal component (e.g. job-state tracking) without a front-end connection. Merged into the same union sent to the Server. |

### 1.11 Tether-Specific Terms

| Term | Definition |
|------|------------|
| **`tether_klippy`** | The Tether implementation of the Server role. It combines the Klipper wire protocol (to MCU devices) with the Moonraker-facing Unix domain socket protocol. |
| **`KlippyUdsServer`** | The Tether class that implements the Unix domain socket server, framing, endpoint dispatch, subscriptions, and remote methods. |
| **`KlippyHost`** | The Tether class that manages the MCU-facing side: data dictionary download, clock sync, command dispatch, and motion translation. |
| **`KlipperDevice`** | The Tether class that implements the MCU device role: serves the data dictionary, processes commands, executes motion. |
| **`MotionPlan`** | A Tether motion-planning abstraction. The `gcode/script` endpoint parses G-code into a `MotionPlan`, which the `MotionTranslator` converts to `queue_step` sequences for the device. |
| **Printer object** | In the Moonraker interface, a named entity exposing a status interface (e.g. `toolhead`, `extruder`, `webhooks`). In Tether, these are backed by the peripheral objects (Stepper, DigitalOut, PWMOut, etc.) and the host state. |

---

## 2. Sequencing

### 2.1 Server Startup Sequence

The `tether_klippy` server starts up in the following order:

```
┌─────────────────────────────────────────────────────────────────┐
│                     Server Startup                              │
└─────────────────────────────────────────────────────────────────┘

 1. Remove pre-existing socket file at the configured path
    (If the file exists and cannot be removed, abort UDS startup;
     the rest of the firmware may still run, but no API clients
     can connect.)

 2. Create AF_UNIX SOCK_STREAM socket, bind to path, listen(backlog=1)

 3. Set socket non-blocking; register with event loop

 4. Initialize printer object model
    ├─ Allocate OIDs for configured peripherals
    ├─ Register core endpoints (info, emergency_stop,
    │  register_remote_method, list_endpoints)
    ├─ Register G-code endpoints (gcode/help, gcode/script,
    │  gcode/restart, gcode/firmware_restart, gcode/subscribe_output)
    ├─ Register query/subscribe endpoints (objects/list,
    │  objects/query, objects/subscribe)
    └─ Register module endpoints (pause_resume/*, query_endstops/status,
       motion_report/dump_stepper, ...)

 5. State = "startup"
    state_message = "Printer is not ready, retry in a few moments"

 6. Connect to MCU device(s) via Klipper wire protocol
    ├─ Download data dictionary (identify handshake)
    ├─ Synchronize clock (get_clock exchanges)
    └─ Configure peripherals (allocate_oid, config commands)

 7. On success:
    └─ State = "ready"
       state_message = "Printer is ready"
       → webhooks subscription push: {"state": "ready", ...}

 8. On failure:
    └─ State = "error"
       state_message = "<specific error text> ... use RESTART after fixing"
       → webhooks subscription push: {"state": "error", ...}
```

### 2.2 Client Connection and Handshake Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Client Handshake (4 phases)                         │
└─────────────────────────────────────────────────────────────────┘

 Phase 1: Socket Acquisition
 ─────────────────────────────
   Loop (0.25 s sleep):
     1. If socket file does not exist → continue looping
     2. If socket file exists but lacks r/w permission → log once, loop
     3. Connect to AF_UNIX socket (read buffer = 20 MiB)
     4. On success: start read loop (ETX framing)
     5. Optionally retrieve SO_PEERCRED (pid/user/group)
        - If pid == 1: systemd socket activation, skip service lookup
        - Else: look up owning systemd service unit

 Phase 2: Identification
 ─────────────────────────
   Loop (0.25 s sleep), poll `info`:
     First successful `info`:
       1. Record software_version
       2. Record full info result as Server identity
       3. Store klipper_path and python_path persistently
       4. Mark Server as "identified"
       5. Emit klippy_identified event
       6. Call list_endpoints
       7. Register all non-reserved endpoints as remote
     First `info` request includes client_info:
       {"id": 1, "method": "info",
        "params": {"client_info": {"program": "Moonraker", "version": "..."}}}
     Subsequent polls omit client_info.

     If `info` fails: keep looping, log periodic hints.

 Phase 3: Readiness
 ───────────────────
   Continue polling `info` (0.25 s):
     If state == "startup" (or absent): keep polling
     If state in {ready, error, shutdown}:
       1. Update internal state (state, state_message)
       2. Subscribe to webhooks object:
          {"id": N, "method": "objects/subscribe",
           "params": {"objects": {"webhooks": null},
                      "response_template": {"method": "process_status_update"}}}
       3. Subscribe to G-code output:
          {"id": N+1, "method": "gcode/subscribe_output",
           "params": {"response_template": {"method": "process_gcode_response"}}}
       4. Re-call list_endpoints (catch newly available module endpoints)
       5. Register any newly appeared endpoints (idempotent)
       6. Emit klippy_started event with startup state
       7. Mark Server as "started"

     Note: If state is "shutdown" at this point, emit klippy_shutdown
     but still complete the "started" phase (subscriptions/endpoints
     are valid in shutdown state).

 Phase 4: Method Registration (only when state == "ready")
 ──────────────────────────────────────────────────────────
   1. Query objects/list, verify required objects:
      - virtual_sdcard
      - display_status
      - pause_resume
      (Missing → log warning, record set, do not abort)
   2. Resolve G-code path from configfile object
   3. For each remote method to register:
      {"id": N, "method": "register_remote_method",
       "params": {"response_template": {"method": "<name>"},
                  "remote_method": "<name>"}}
      (Failures logged per-method, do not abort handshake)
   4. Mark methods as registered
   5. Emit klippy_ready event

 After Phase 4:
   - State == "ready" → fully operational; forward front-end requests
   - State == "error" or "shutdown" → connected and observing,
     but motion commands will fail; operator must restart
   - All future state changes observed via webhooks subscription push
     (not via info polling)
```

### 2.3 Request/Response Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Request/Response Flow                               │
└─────────────────────────────────────────────────────────────────┘

 Client side (write path):
   1. Allocate unique id (integer, unique among in-flight requests)
   2. Create pending entry: {id → {waitable, method_name}}
   3. Insert into pending-request table
   4. Encode: {"id": <id>, "method": "<endpoint>", "params": <args>}
      as compact JSON + 0x03
   5. Write to socket, drain write buffer
   6. Await waitable with timeout:
      - Default: 60 s
      - Subscription (objects/subscribe): 20 s
      - Indefinite-wait: no timeout, log every 60 s
   7. On success: return result to caller
      On failure: raise error
   8. Finally: remove entry from pending table

 Server side (processing):
   1. Receive frame, decode JSON, validate top-level is object
   2. If malformed: log and skip (no response, even if id present)
   3. If has method + id: dispatch to endpoint handler
   4. Handler processes in arrival order
   5. Handler MAY complete asynchronously (e.g. gcode/script waiting
      on temperature) → responses may be sent out of order
   6. On success: {"id": <id>, "result": <dict or {}>}
   7. On command error: {"id": <id>, "error": {"message": "...", "error": "WebRequestError"}}
   8. On unhandled exception: send error response AND transition to shutdown

 Client side (read path):
   1. Receive bytes, append to partial buffer
   2. Split on 0x03; decode each complete frame as JSON
   3. For each decoded frame:
      ├─ If has id, no method → Response
      │   ├─ Look up id in pending table
      │   ├─ Not found → log "no request matching id", discard
      │   └─ Found → remove entry, resolve/fail waitable
      │       ├─ result present → resolve with result
      │       │   (empty/falsy result → MAY substitute "ok")
      │       └─ error present → fail with error.message
      │
      └─ If has method, no id → Push message
          ├─ Dispatch to handler registered for that method
          └─ No handler → log and discard
```

### 2.4 Subscription Update Coalescing Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Status Update Coalescing (every 0.25 s)             │
└─────────────────────────────────────────────────────────────────┘

 Timer started when first subscription created.
 Timer stopped when no subscriptions remain.

 Each tick (every 0.25 s):
   For each subscriber S:
     1. Call each subscribed object's status function at current eventtime
     2. Compute requested field set
        (resolve null → "all fields" on first iteration)
     3. Compare each field's current value to S's last-pushed baseline
     4. Build per-subscriber diff: only fields whose values changed
     5. If diff is non-empty:
        a. Emit push message:
           {<response_template keys...>,
            "params": {"status": {<obj: {changed fields}>},
                       "eventtime": <float>}}
        b. Update S's baseline to current values
     6. If diff is empty: no push (field that doesn't change is never re-pushed)

 First push after subscribing:
   - Diff of initial snapshot against empty baseline
   - Effectively the full requested field set (everything is "new")
   - Subsequent pushes contain only changes
```

### 2.5 Subscription Aggregation Sequence (Client-side)

```
┌─────────────────────────────────────────────────────────────────┐
│         Front-end Subscribe → Server Union → Fan-out            │
└─────────────────────────────────────────────────────────────────┘

 Establishing a front-end subscription:
   1. Acquire subscription lock (serialize concurrent subscriptions)
   2. Read front-end client's requested_sub: {object: [fields] or null}
   3. Remove existing entry for this connection from subscriptions map
   4. Build all_subs = copy of requested_sub
   5. Union in every other connection's subscription:
      For each (object, items) in every other subscription:
        If object already in all_subs:
          If either items is null → union = null (all fields)
          Else → union = set-union of field lists
        Else → add object with its items
   6. Send to Server:
      {"id": N, "method": "objects/subscribe",
       "params": {"objects": all_subs,
                  "response_template": {"method": "process_status_update"}}}
      (with 20 s timeout)
   7. Process full snapshot response (see below)
   8. Record requested_sub as this connection's subscription

 Processing the subscription response (initial snapshot):
   1. Diff against cache, update cache (skip excluded fields)
   2. Prune cache to match snapshot (remove objects not in snapshot)
   3. If diff non-empty: push to existing subscribers (fan-out)
   4. Prune response for requesting connection:
      - null → all fields; else → only requested fields
   5. Return {status: pruned_status, eventtime} to front-end client

 Fanning out a push update (steady state):
   1. Update cache: merge incoming diff fields into subscription_cache
   2. If webhooks in update: refresh internal state, emit events
   3. For each front-end connection C in subscriptions:
      a. Build conn_status by filtering diff to C's requested fields:
         - null → all fields from diff
         - [fields] → only requested fields present in diff
      b. If conn_status non-empty: send update to C
```

### 2.6 Remote Method Invocation Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Remote Method Registration and Invocation          │
└─────────────────────────────────────────────────────────────────┘

 Registration (Client → Server):
   1. Client sends:
      {"id": N, "method": "register_remote_method",
       "params": {"response_template": {"method": "paneldue_beep"},
                  "remote_method": "paneldue_beep"}}
   2. Server records for this connection:
      method_name "paneldue_beep" → response_template
   3. Server responds: {"id": N, "result": {}}
   4. Registration is per-connection; connection close drops the entry

 Invocation (Server → Client, fire-and-forget):
   1. Firmware code calls method by name with keyword arguments
   2. Server looks up method in per-method connection map
   3. For each connection with a non-closed entry:
      a. Copy stored response_template
      b. Insert params = keyword arguments dictionary
      c. Send push message on that connection:
         {"method": "paneldue_beep",
          "params": {"frequency": 300, "duration": 1.0}}
   4. Remove closed connections from map
   5. If no live connections remain:
      → Delete method entry, notify firmware caller of failure

 Client dispatch:
   - Push message has method, no id → dispatch to registered handler
   - No response expected (fire-and-forget)
   - If Client needs to feed data back: issue a normal request
     (e.g. gcode/script) in the opposite direction
```

### 2.7 State Transition Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                    Server State Machine                          │
└─────────────────────────────────────────────────────────────────┘

         +----------+
 init →  | startup  |
         +----------+
           |     |
           |     +──────────────> error     (config/protocol/MCU/
           |                                internal error during init)
           v
         +----------+
         |  ready   |
         +----------+
           |
           v
         +----------+
         | shutdown |           (emergency_stop, runtime fault,
         +----------+            unhandled handler exception)
           |
           v
         (restart cycles back to startup via process restart)

 Transition rules:
   startup → ready:    init completed successfully
                        → webhooks push: {"state": "ready", ...}
   startup → error:     init failed
                        → webhooks push: {"state": "error", ...}
                        → stays in error until process restart
   ready → shutdown:    emergency_stop, M112, unhandled exception,
                        or MCU move-fault
                        → webhooks push: {"state": "shutdown", ...}
                        → safe halted state, no self-recovery
   error → shutdown:    already terminal; no further change

 NO direct ready → error transition exists.
 NO spontaneous shutdown → ready transition exists.
 Recovery from error or shutdown requires process restart
   (gcode/restart or gcode/firmware_restart → process exit →
    service manager restart → new process → startup).
```

### 2.8 Emergency Stop Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Emergency Stop Flow                                 │
└─────────────────────────────────────────────────────────────────┘

 Trigger: emergency_stop endpoint OR M112 G-code

 1. Server receives request: {"id": N, "method": "emergency_stop"}
 2. Server immediately halts all motion
 3. Server responds: {"id": N, "result": {}}
    (Response sent BEFORE shutdown transition fully propagated)
 4. Server transitions: ready → shutdown
    state_message = "Shutdown due to webhooks request"
 5. webhooks subscription push:
    {"method": "process_status_update",
     "params": {"status": {"webhooks": {"state": "shutdown",
      "state_message": "Shutdown due to webhooks request"}},
                "eventtime": <float>}}
 6. Client observes shutdown via subscription push
 7. All subsequent motion commands return errors
 8. Recovery requires process restart (gcode/restart)
```

### 2.9 G-code Script Execution Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              gcode/script Execution Flow                         │
└─────────────────────────────────────────────────────────────────┘

 1. Client sends:
    {"id": N, "method": "gcode/script",
     "params": {"script": "G90\nG1 X200"}}

 2. Client emits internal gcode_received event (before request sent)

 3. Server receives request, queues script under G-code mutex

 4. If a G-code command is already running:
    → New script is queued; delay can be significant
    (e.g. waiting for a temperature)

 5. Server parses and executes each line in order:
    ├─ G90 → set absolute positioning
    └─ G1 X200 → plan motion to X=200
       ├─ MotionPlan generated via Tether motion planner
       ├─ MotionTranslator converts to queue_step sequences
       └─ queue_step commands dispatched to MCU device

 6. Terminal output produced by the script:
    → NOT returned in the response
    → Delivered via gcode/subscribe_output subscription push:
      {"method": "process_gcode_response",
       "params": {"response": "// ..."}}

 7. After entire script finishes:
    Server responds: {"id": N, "result": {}}

 8. If script raises G-code error:
    Server responds: {"id": N, "error": {"message": "...", "error": "WebRequestError"}}

 9. If script triggers internal fault:
    Server responds with error AND transitions to shutdown

 Note: Status push messages continue to flow on the same connection
       while gcode/script is pending, interleaved with (and possibly
       before) the script's response.
```

### 2.10 Restart Sequence (End-to-End)

```
┌─────────────────────────────────────────────────────────────────┐
│              Restart Flow (gcode/restart)                        │
└─────────────────────────────────────────────────────────────────┘

 1. Client sends: {"id": N, "method": "gcode/restart"}
 2. Server executes restart G-code
 3. Server responds: {"id": N, "result": {}}
 4. Server process exits
 5. Socket closes (process exit)
 6. Client read loop sees EOF → connection-closed handler:
    a. Reset identification/started/methods-registered flags
    b. Set state to "disconnected", message "Klippy Disconnected"
    c. Fail all pending requests (HTTP 503), clear pending table
    d. Clear all front-end subscriptions and subscription cache
    e. Clear peer credentials and missing-requirements set
    f. Emit klippy_disconnect event
    g. Start new connection task (automatic reconnection)
 7. Service manager restarts Server process
 8. New Server process creates socket, enters "startup"
 9. Client connection task connects → full handshake:
    → klippy_identified → klippy_started → (klippy_ready)

 Front-end event sequence:
   klippy_shutdown? → klippy_disconnect → klippy_identified →
   klippy_started → (klippy_ready)

 (klippy_shutdown may or may not fire before disconnect, depending
  on whether restart G-code triggered a shutdown state push before
  process exit.)

 firmware_restart variant:
   Same flow, but additionally re-initializes MCU firmware
   (serial flash/reboot of microcontrollers), which takes longer.
```

### 2.11 Unexpected Disconnect Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Unexpected Disconnect (Server gone)                 │
└─────────────────────────────────────────────────────────────────┘

 Trigger: read loop receives EOF (zero bytes) or non-transient read error

 Connection-closed handler:
   1. Reset identification/started/initializing/methods-registered flags
   2. Set internal state to "disconnected"
      message = "Klippy Disconnected"
   3. Fail every pending request with "Klippy Disconnected" error (HTTP 503)
      Clear pending-request table
   4. Clear all front-end subscriptions and subscription cache
   5. Clear peer credentials and missing-requirements set
   6. Emit klippy_disconnect event to Client components
   7. If Client itself is still running:
      → Automatically start new connection task
      → Go back to handshake phase 1 (socket acquisition)

 Reconnection is unconditional as long as the Client service is running.
 Handles: Server process crash, Server restart, MCU disconnect causing
 Server exit, transient socket file removal.
```

### 2.12 Normal Close Sequence (Client shutting down)

```
┌─────────────────────────────────────────────────────────────────┐
│              Normal Close by Client                              │
└─────────────────────────────────────────────────────────────────┘

 1. Set closing flag (read loop and write path stop initiating work)
 2. Cancel connection task if still running
 3. Close writer, wait for drain/close
 4. Run connection-closed handler (same as unexpected disconnect,
    steps 1-6 above)
 5. Do NOT attempt to reconnect (the whole Client is stopping)
```

### 2.13 Server State Change Observation Sequence (Runtime)

```
┌─────────────────────────────────────────────────────────────────┐
│         Server State Change Observed at Runtime                  │
└─────────────────────────────────────────────────────────────────┘

 After handshake is complete, Client observes state via webhooks
 subscription. When a process_status_update push arrives containing
 the webhooks object:

 1. If state_message is present:
    → Update Client's stored state message

 2. If state is present:
    a. If new state is "shutdown"
       AND Client is NOT in initialization phase
       AND previous state was NOT already "shutdown":
       → Emit klippy_shutdown event
       → Log "Klippy has shutdown"
    b. Update Client's internal state to new state

 The "not during initialization" guard matters:
   If Server transitions to shutdown while handshake is still polling
   info, the handshake code itself handles the shutdown event
   (emitting klippy_shutdown from the startup path); the runtime
   handler defers to avoid double-firing.

 No special handling for ready → error:
   That transition does not occur at runtime (errors are startup-only;
   runtime faults go to shutdown).

 No spontaneous shutdown → ready:
   Requires process restart → manifests as disconnect → reconnect →
   fresh handshake.
```

### 2.14 Endpoint Discovery Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│              Endpoint Discovery (during handshake)               │
└─────────────────────────────────────────────────────────────────┘

 Discovery happens at two points during the handshake:

 Wave 1 (Phase 2: first successful info):
   1. Call list_endpoints:
      {"id": N, "method": "list_endpoints"}
   2. Response: {"endpoints": ["info", "emergency_stop", "gcode/script", ...]}
   3. For each endpoint NOT in reserved set:
      → Register as remote endpoint in Client's API
   Purpose: Make info and emergency_stop available to front-end
            clients even before Server is fully ready.

 Wave 2 (Phase 3: leaving startup):
   1. Re-call list_endpoints
   2. For each newly appeared endpoint (not already registered):
      → Register as remote endpoint (idempotent)
   Purpose: Catch module endpoints that only appear after ready.

 Reserved endpoints (never exposed to front-end):
   - list_endpoints
   - gcode/subscribe_output
   - register_remote_method

 All other endpoints SHOULD be exposed as forwardable.
```

### 2.15 Full Connection Lifecycle Summary

```
┌─────────────────────────────────────────────────────────────────┐
│              Full Connection Lifecycle                           │
└─────────────────────────────────────────────────────────────────┘

     ┌──────────┐
     │Disconnected│◄──────────────────────────────────┐
     └─────┬─────┘                                     │
           │ connect                                   │
           v                                           │
     ┌──────────┐  info poll       ┌──────────┐        │
     │Connecting │──────────────► │Connected │        │
     └──────────┘                  └─────┬────┘        │
                                          │ handshake  │
                                          │ complete   │
                                          v            │
                                    ┌──────────┐       │
                                    │Observing │       │
                                    │(steady   │       │
                                    │ state)   │       │
                                    └─────┬────┘       │
                                          │            │
              ┌─────────────────────────────┤            │
              │                             │            │
              v                             v            │
        ┌──────────┐              ┌──────────┐         │
        │  Closing  │              │Unexpected│         │
        │(intentional)│            │Disconnect│         │
        └─────┬─────┘              └─────┬────┘         │
              │                          │              │
              │ no reconnect             │ reconnect    │
              v                          └──────────────┘
        ┌──────────┐
        │Disconnected│
        │(permanent) │
        └──────────┘

 Client reported state to front-end:
   Not connected → "disconnected"
   Connected, Server in startup → "startup"
   Otherwise → Server's state ("ready", "error", "shutdown")
```

### 2.16 Concurrent Request and Push Interleaving

```
┌─────────────────────────────────────────────────────────────────┐
│         Interleaving of Requests, Responses, and Pushes          │
└─────────────────────────────────────────────────────────────────┘

 The socket is full-duplex. At any moment, the Client may have
 multiple in-flight requests, and the Server may be emitting push
 messages. All of these are interleaved on the same byte stream,
 framed by ETX (0x03).

 Example timeline (Client perspective):

   Client sends:  {"id":1,"method":"gcode/script","params":{"script":"G1 X100"}} <0x03>
   Client sends:  {"id":2,"method":"objects/query","params":{"objects":{"toolhead":["position"]}}} <0x03>

   Server pushes: {"method":"process_status_update","params":{"status":{"toolhead":{"position":[50,0,0,0]}},"eventtime":1234.5}} <0x03>
   Server resp:   {"id":2,"result":{"status":{"toolhead":{"position":[50,0,0,0]}},"eventtime":1234.5}} <0x03>
   Server pushes: {"method":"process_gcode_response","params":{"response":"// moving..."}} <0x03>
   Server resp:   {"id":1,"result":{}} <0x03>

 Key rules:
   - Requests processed by Server in arrival order
   - Responses may be out of order (handler 2 completed before handler 1)
   - Push messages interleaved with responses in any order
   - Client MUST correlate by id, not by arrival order
   - Client MUST dispatch each frame independently based on its kind:
     has id, no method → Response (lookup pending request)
     has method, no id → Push (dispatch to handler)
```

### 2.17 Tether-Specific: G-code to Motion Execution Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│      Tether: gcode/script → MotionPlan → queue_step             │
└─────────────────────────────────────────────────────────────────┘

 When tether_klippy receives gcode/script:

 1. KlippyUdsServer receives {"method": "gcode/script",
                                "params": {"script": "G1 X100 F600"}}
 2. G-code parser (tether_gcode) parses lines into tokens/commands
 3. Motion planner (tether_motion_planner) generates MotionPlan:
    ├─ Geometry: NURBS curves, path blending
    ├─ Velocity profile: trapezoidal/SCURVE with limits
    └─ Blend: corner blending between segments
 4. MotionTranslator converts MotionPlan → queue_step sequences:
    ├─ Each step: {interval, count, add} (VLQ-encoded)
    ├─ Multiple steppers: one queue_step per stepper per segment
    └─ Acceleration: add field adjusts interval per step
 5. KlippyHost sends queue_step commands to KlipperDevice via:
    ├─ SerialQueue (reliability: sliding window, ack, retransmit)
    ├─ Transport (TCP, pipe, loopback, or CAN)
    └─ Wire protocol (message blocks, CRC, VLQ encoding)
 6. KlipperDevice receives and executes:
    ├─ StepExecutor ticks Stepper objects at scheduled clocks
    ├─ McuClock advances; clock sync keeps host and device aligned
    └─ MotionReconstructor can reconstruct trajectory for analysis
 7. Terminal output (if any) pushed via gcode/subscribe_output:
    {"method": "process_gcode_response",
     "params": {"response": "// ..."}}
 8. Response sent after script completes:
    {"id": N, "result": {}}
```

---

## 3. Field Reference Summary

### 3.1 Message Field Matrix

| Field     | Type   | Request | Response (success) | Response (error) | Push   |
|-----------|--------|---------|--------------------|------------------|--------|
| `id`      | any    | opt.    | yes (echo)         | yes (echo)       | no     |
| `method`  | string | yes     | no                 | no               | yes    |
| `params`  | dict   | opt.    | no                 | no               | yes    |
| `result`  | dict   | no      | yes                | no               | no     |
| `error`   | dict   | no      | no                 | yes              | no     |

### 3.2 `info` Response Fields

| Field              | Type   | Meaning                                      |
|--------------------|--------|----------------------------------------------|
| `state`            | string | `startup`, `ready`, `error`, `shutdown`      |
| `state_message`    | string | Human-readable description of current state  |
| `hostname`         | string | OS hostname                                  |
| `klipper_path`     | string | Absolute path to Server source directory      |
| `python_path`      | string | Absolute path to Python interpreter           |
| `process_id`       | int    | OS process id                                |
| `user_id`          | int    | Numeric Unix user id                         |
| `group_id`         | int    | Numeric Unix group id                        |
| `log_file`         | string | Absolute path to Server log file              |
| `config_file`      | string | Absolute path to printer configuration file  |
| `software_version` | string | Server software version string              |
| `cpu_info`         | string | Short description of host CPU               |

### 3.3 Error Semantics (Client-side)

| Outcome                       | Wire cause                          | Client error code |
|-------------------------------|-------------------------------------|-------------------|
| Success with result           | Response with `result`              | 200               |
| Server-reported error         | Response with `error`               | 400               |
| Not connected / connection lost | No socket, or write/read failure | 503               |
| Timeout                       | No response within timeout          | 500               |
| Malformed Server response     | Response frame could not be decoded | (logged; discarded) |

### 3.4 Timeout Reference

| Request type           | Timeout  | Notes                                    |
|------------------------|----------|------------------------------------------|
| Default                | 60 s     | Most requests                            |
| `objects/subscribe`    | 20 s     | Initial snapshot response; push stream has no timeout |
| Indefinite-wait        | ∞        | Logs "still pending" every 60 s         |
| `info` poll (handshake) | re-issue on failure | No single timeout; loops until state changes |

### 3.5 Endpoint Categories

| Category         | Always present? | Examples                                                     |
|------------------|-----------------|--------------------------------------------------------------|
| Core             | yes             | `info`, `emergency_stop`, `register_remote_method`, `list_endpoints` |
| G-code           | yes             | `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart`, `gcode/subscribe_output` |
| Query/subscribe  | yes             | `objects/list`, `objects/query`, `objects/subscribe`         |
| Module           | no (config-dep.) | `pause_resume/pause`, `query_endstops/status`, `bed_mesh/dump_mesh`, `motion_report/dump_stepper` |

### 3.6 Reserved Endpoints

| Endpoint                | Reason for being reserved                          |
|-------------------------|----------------------------------------------------|
| `list_endpoints`        | Used internally by Client for discovery            |
| `gcode/subscribe_output` | Client captures G-code terminal stream; re-broadcasts as its own event |
| `register_remote_method` | Client wires up remote callbacks; would interfere with its own registration state |

### 3.7 Required Printer Objects (checked at readiness)

| Object           | Purpose                                           |
|------------------|---------------------------------------------------|
| `virtual_sdcard` | File-based printing; Client needs to locate the G-code path |
| `display_status` | Display state for front-end display widgets        |
| `pause_resume`   | Pause/resume/cancel functionality                  |

Missing objects are logged as warnings but do not abort the handshake.

### 3.8 Built-in Remote Methods (implicit registration)

| Method                   | Wired via                          | Payload of push `params`                          |
|--------------------------|------------------------------------|---------------------------------------------------|
| `process_gcode_response` | `gcode/subscribe_output` template | `{"response": "<gcode line>"}`                    |
| `process_status_update`  | `objects/subscribe` template       | `{"status": {<obj: {fields}}}, "eventtime": <float>}` |

### 3.9 Cache Exclusions

| Object       | Excluded fields     | Reason                                    |
|--------------|---------------------|-------------------------------------------|
| `configfile` | `config`, `settings` | Large, never change after startup; caching wastes memory and diffing is pointless |

Excluded fields are still delivered to front-end subscribers that requested them; they are removed only from the cache, not from the response or push delivery.
