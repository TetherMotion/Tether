# Running the Simulated Printer with Mainsail or Fluidd

The `klipper_http` example runs a fully simulated 3D printer
(thermal models, motion, G-code execution, print playback) and exposes
the complete Moonraker HTTP + WebSocket API. You can connect
[Mainsail](https://docs.mainsail.xyz/) or [Fluidd](https://docs.fluidd.xyz/)
directly — no Moonraker or Klipper process needed.

## Build

```bash
cmake -B build \
  -DTETHER_ENABLE_KLIPPER=1 \
  -DTETHER_ENABLE_KLIPPER_HTTP=1
cmake --build build --target klipper_http -j4
```

## Quick Start

### With Mainsail

```bash
./build/bin/klipper_http --no-auth --mainsail ~/mainsail
```

On first run, the example detects that `~/mainsail` doesn't have a built
`dist/` directory and asks for permission to clone and build:

```
Mainsail not found at /home/user/mainsail.

I will run the following commands:
  git clone https://github.com/mainsail-crew/mainsail.git /home/user/mainsail
  cd /home/user/mainsail
  npm install
  npm run build

Proceed? [y/N]
```

Type `y` and press Enter. After the build completes, Tether starts serving
Mainsail and the simulated printer API on the same port. Open
**http://localhost:7125** in a browser.

On subsequent runs, the existing build is reused automatically — no
prompt, no rebuild.

### With Fluidd

```bash
./build/bin/klipper_http --no-auth --fluidd ~/fluidd
```

Same flow: clones `https://github.com/fluidd-core/fluidd.git`, runs
`npm install && npm run build`, then serves the result.

### With pre-built static assets

If you already have a built Mainsail or Fluidd `dist/` directory (e.g.
downloaded from the releases page), use `--web-root`:

```bash
# Download pre-built Mainsail
wget https://github.com/mainsail-crew/mainsail/releases/latest/download/mainsail.zip
unzip mainsail.zip -d /opt/mainsail

# Serve it directly
./build/bin/klipper_http --no-auth --web-root /opt/mainsail
```

## All Options

```
Usage: klipper_http [--help] [--port VAR] [--with-moonraker]
       [--uds-path VAR] [--mainsail DIR] [--fluidd DIR] [--web-root DIR]
       [--gcodes-root VAR] [--config-root VAR] [--logs-root VAR]
       [--api-key VAR] [--no-auth] [--sim-tick-ms VAR]

  --mainsail DIR     Clone+build Mainsail into DIR (asks y/N first).
                     Reuses existing build if dist/ is present.
  --fluidd DIR       Same for Fluidd.
  --web-root DIR     Serve pre-built static assets from DIR as-is.
  --port PORT        HTTP listen port (default: 7125).
  --with-moonraker   Enable UDS transport for a separate Moonraker process
                     (disabled by default; the built-in HTTP server replaces
                     Moonraker).
  --uds-path PATH    UDS socket path (only used with --with-moonraker).
  --no-auth          Disable API authentication.
  --sim-tick-ms MS   Simulation tick interval (default: 100, min: 10).
```

Run `--help` to see all options.

## What You Can Do

Once Mainsail/Fluidd is loaded in the browser:

- **Dashboard** — extruder and bed temperatures (start at ~25°C, rise/fall
  with the simulated thermal model when you set targets)
- **Temperature panel** — set heater targets and watch the PID controller
  drive the simulated thermal model
- **G-code Files** — a sample file (`tether_calibration_square.gcode`) is
  auto-generated; click it and press **Print** to start a simulated print
- **Console** — send G-code commands (`G28`, `M104 S210`, `G1 X50 Y50`)
- **Toolhead** — position updates live as G-code executes during prints
- **Print progress** — layer count, filament used, duration

## How It Works

```
  Browser (http://localhost:7125)
        │
        ▼
┌──────────────────────────────────┐
│  Tether (single process)          │
│                                   │
│  KlippyHttpServer on :7125        │
│  ├── Serves Mainsail/Fluidd SPA   │
│  ├── HTTP REST API (120+ endpoints)│
│  └── WebSocket JSON-RPC           │
│                                   │
│  KlippyInstance (simulated)       │
│  ├── PID heaters (thermal model)  │
│  ├── G-code executor              │
│  ├── Virtual SD card + print      │
│  └── 68 printer objects           │
└───────────────────────────────────┘
```

Tether serves both the web UI static files and the Moonraker API from the
same port. The SPA's `config.json` points to `localhost:7125`, so the
browser connects back to the same server for WebSocket and REST calls.
No nginx proxy or Docker needed.

## Prerequisites for --mainsail / --fluidd

The auto-clone-and-build feature requires:

- **git** — to clone the repository
- **Node.js 20+** and **npm** — to build the SPA

If these aren't available, use `--web-root` with pre-built static assets
instead.

## Alternative: Docker

If you prefer to run Mainsail in a Docker container (e.g. on a headless
server), see the Docker setup below.

### 1. Start Tether

```bash
./build/bin/klipper_http --no-auth --port 7125
```

### 2. Start Mainsail container

```bash
docker run -d \
  --name tether_mainsail \
  -p 8080:8080 \
  --add-host host.docker.internal:host-gateway \
  -v "$(pwd)/docker/mainsail/mainsail-config.json:/usr/share/nginx/html/config.json" \
  -v "$(pwd)/docker/mainsail/mainsail-proxy.conf:/etc/nginx/extra-conf.d/proxy.conf" \
  ghcr.io/mainsail-crew/mainsail:latest
```

### 3. Open http://localhost:8080

The Mainsail container's nginx proxies `/server`, `/api`, `/machine`,
`/access`, `/client`, and `/websocket` to `host.docker.internal:7125`,
which reaches Tether on the host.

### Stop

```bash
docker rm -f tether_mainsail
```

## Troubleshooting

### "Address already in use" on port 7125

Another process is using the port. Use a different one:

```bash
./build/bin/klipper_http --no-auth --port 7130 --mainsail ~/mainsail
```

### Mainsail shows "Moonraker not connected"

1. Verify Tether is running: `curl http://localhost:7125/server/info`
2. Check that `config.json` in the dist directory points to the correct
   port. The example writes this automatically; verify with:
   ```bash
   cat ~/mainsail/dist/config.json
   ```

### npm build fails

Ensure Node.js 20+ is installed:
```bash
node --version  # should be v20 or higher
```

If the build still fails, use `--web-root` with a pre-built release
instead (see Quick Start above).
