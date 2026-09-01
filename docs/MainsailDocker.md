# Running the Simulated Printer with Mainsail (Docker)

This guide walks you through starting the Tether simulated 3D printer on
your host and connecting [Mainsail](https://docs.mainsail.xyz/) to it via a
Docker container. No Moonraker or Klipper process is needed — Tether
implements the full Moonraker HTTP + WebSocket API natively.

## Architecture

```
  Your browser (http://localhost:8080)
        │
        ▼
┌──────────────────────────────────┐
│  Mainsail Docker Container        │
│  (ghcr.io/mainsail-crew/mainsail) │
│                                   │
│  Nginx serves the SPA on :8080    │
│  Proxies /server, /api, /machine, │
│  /access, /client, /websocket     │
│  → host.docker.internal:7125      │
└───────────────┬───────────────────┘
                │  (host bridge, port 7125)
                ▼
┌──────────────────────────────────┐
│  Tether (host process)            │
│  ./build/bin/klipper_http_mainsail│
│                                   │
│  KlippyHttpServer on :7125        │
│  + simulated heaters, motion,     │
│    print playback, G-code exec    │
└───────────────────────────────────┘
```

## Prerequisites

- **Docker** and **Docker Compose** (v2+)
- **Tether** built with HTTP support:

  ```bash
  cmake -B build \
    -DTETHER_ENABLE_KLIPPER=1 \
    -DTETHER_ENABLE_KLIPPER_HTTP=1
  cmake --build build --target klipper_http_mainsail -j4
  ```

## Quick Start (3 steps)

### 1. Start the Tether simulated printer

From the Tether repo root:

```bash
./build/bin/klipper_http_mainsail --no-auth --port 7125
```

You should see:

```
=== Tether Simulated 3D Printer (Moonraker Replacement) ===
...
HTTP/WebSocket server listening on port 7125
Open http://localhost:8080/ in a browser to access the web UI.
```

Leave this running in a terminal.

### 2. Start the Mainsail container

From the Tether repo root:

```bash
docker compose -f docker/docker-compose.yml up -d
```

This pulls the official Mainsail image and starts it on port **8080**.
The container's nginx is configured (via `docker/mainsail/mainsail-proxy.conf`)
to proxy all API calls to `host.docker.internal:7125`, which reaches the
Tether process on your host.

### 3. Open Mainsail

Open **http://localhost:8080** in a browser.

Mainsail will load and automatically connect to Tether via the WebSocket
proxy. You should see:

- **Dashboard** with extruder and bed temperatures (starting at ~25°C)
- **Temperature** panel — set targets and watch them rise/fall with the
  simulated thermal model
- **G-code Files** panel — a sample file
  (`tether_calibration_square.gcode`) is pre-generated; click it and press
  **Print** to start a simulated print
- **Console** — send G-code commands (e.g. `G28`, `M104 S210`, `G1 X50 Y50`)
- **Toolhead** panel — position updates live during prints

## How It Works

The setup uses two files in `docker/mainsail/`:

| File | Purpose |
|------|---------|
| `mainsail-config.json` | Tells the Mainsail SPA to connect to `localhost:8080` (itself, via the nginx proxy) |
| `mainsail-proxy.conf` | Nginx config that proxies `/server`, `/api`, `/machine`, `/access`, `/client`, and `/websocket` to `host.docker.internal:7125` |

The `docker-compose.yml` mounts these into the Mainsail container and adds
`host.docker.internal:host-gateway` so the container can reach the host
(this is needed on Linux; Docker Desktop handles it automatically).

## Changing the Tether Port

If you want Tether on a different port (e.g. 7130):

1. Start Tether on the new port:
   ```bash
   ./build/bin/klipper_http_mainsail --no-auth --port 7130
   ```

2. Edit `docker/mainsail/mainsail-proxy.conf` and replace all `7125`
   with `7130`.

3. Restart the container:
   ```bash
   docker compose -f docker/docker-compose.yml restart mainsail
   ```

## Stopping

```bash
# Stop Mainsail
docker compose -f docker/docker-compose.yml down

# Stop Tether (Ctrl+C in its terminal)
```

## Running Without Docker (Built-in Static Assets)

If you prefer not to use Docker at all, you can download the Mainsail
static assets and serve them directly from Tether:

```bash
# Download Mainsail release
wget https://github.com/mainsail-crew/mainsail/releases/latest/download/mainsail.zip
unzip mainsail.zip -d /opt/mainsail

# Run Tether with Mainsail static assets
./build/bin/klipper_http_mainsail \
    --port 7125 \
    --web-root /opt/mainsail \
    --no-auth
```

Then open **http://localhost:7125/** — Tether serves both the API and the
Mainsail SPA from the same port. No nginx proxy needed.

## Troubleshooting

### "Address already in use" on port 7125

Another process is using the port. Either stop it or use a different port:

```bash
./build/bin/klipper_http_mainsail --no-auth --port 7130
```

(And update `mainsail-proxy.conf` as described above.)

### Mainsail shows "Moonraker not connected"

1. Verify Tether is running: `curl http://localhost:7125/server/info`
2. Verify the container can reach the host:
   ```bash
   docker exec tether_mainsail \
     curl -s http://host.docker.internal:7125/server/info
   ```
3. Check the proxy config in `docker/mainsail/mainsail-proxy.conf` —
   the port must match the Tether `--port` argument.

### Port 8080 already in use

Change the host-side mapping in `docker/docker-compose.yml`:

```yaml
    ports:
      - "8081:8080"   # map host 8081 → container 8080
```

Then open `http://localhost:8081` instead.
