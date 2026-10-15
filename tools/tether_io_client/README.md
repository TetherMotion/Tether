# tether_io_client

Standalone C++ client for the Tether IO (V6) protocol — replaces the
webinterface for direct access to parameters, signals, functions, streams,
logs, datalogging, thresholds and input streams over **WebSocket**
(`ws://`), **TCP/SLIP** (`tcp://`), or **serial/SLIP** (`serial://`).

## Build

```bash
cmake --build build --target tether_io_client -j8
./build/bin/tether_io_client --help
```

The TUI requires `tether_terminal_ui` (ncurses) — built automatically when
ncurses is found; otherwise `tui` prints a clear error.

## Connect

```bash
tether_io_client -u ws://127.0.0.1:8080/tether-io   # WS (default)
tether_io_client -u tcp://127.0.0.1:4000            # raw TCP + SLIP
tether_io_client -u serial:///dev/ttyACM0?baud=115200
tether_io_client -u ws://host?token=T&role=dev      # auth / dev role
# or: export TETHER_IO_ADDR=tcp://127.0.0.1:4000
```

Globals: `-u/--url`, `--token`, `--role`, `--timeout-ms`,
`--connect-timeout-ms`, `-j/--json`, `--verbose`, `--no-confirm`.

## Commands

```
ping [--count N]               RTT probe
info                           epoch, limits, catalog summary
list params|signals|functions [-g GLOB] [--filter STR]
get NAME... [-r|--raw]         schema-decoded read
set NAME VALUE                 schema-typed write (JSON for composites)
snapshot [params|signals]      bulk read
metadata NAME                  key/value metadata
call NAME k=v... [--invoke-ex] function call (name=raw:<hex> escapes)
stream NAME... [--interval-ms N] [--chunk N] [--count N] [--duration S]
               [--csv] [--record FILE] [--stats]
watch NAME...                  polling live table
log tail [--severity L]        subscribe to the log stream
datalog status|configure ...   datalog recorder
threshold set NAME --rule=...  threshold filter
instream create|send|close     client→server input streams
peer register NAME [--id N]    host a client-side function
stress [ping|catalog] [--ops N]
shell                          REPL (same grammar)
tui                            ncurses explorer (Catalog/Watch/Stream/
                               Functions tabs, `:` prompt, `?` keys)
version
```

Names resolve as exact name, `group.name`, numeric/`0x` id, or
`p:`/`s:`/`f:` prefixes. Function args are schema-encoded; when a
parameter's schema can't express the literal use `name=raw:<hex>`.

## Simulator

`examples/tether_io_sim` serves the CiA402 fleet + writable `sim.*`
params over TCP/SLIP :4000 and (optionally) WS :4001:

```bash
./build/bin/tether_io_sim --port 4000 --ws-port 4001
tether_io_client -u tcp://127.0.0.1:4000 get machine.cia402.machine.snapshot
tether_io_client -u ws://127.0.0.1:4001 stream sim.counter --count 5
```

## Layout

- `client_session` — URL→transport factory, `IoSession` (catalog cache,
  name resolution, background pump thread), `runTui`.
- `value_text` — literal/JSON → `SchemaValue` → wire bytes.
- `commands` — `dispatch(ctx, args)` shared by CLI/shell/TUI `:`.
- `tui` — `PanelScreen` views (Catalog/Watch/Stream/Functions + log).
- `main` — argparse globals, `shell`, entry.

Tests: `tests/io/test_io_client_commands.cpp` (`tether_io_client_tests`)
drives the dispatcher over an in-proc `MessagePipeTransport` ↔ `Session`.
