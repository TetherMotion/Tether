# Running Tether with Mainsail (Docker)

This guide explains how to run Tether as a Moonraker replacement and connect
it to [Mainsail](https://docs.mainsail.xyz/) using Docker Compose.

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  Tether Process                  │
│                                                  │
│  ┌──────────────┐    ┌───────────────────────┐  │
│  │  KlippyServer │    │  KlippyUdsServer      │  │
│  │ (business     │◄───┤  (UDS transport)      │  │
│  │  logic)       │    │  /tmp/klippy_uds      │  │
│  │               │    └───────────────────────┘  │
│  │               │    ┌───────────────────────┐  │
│  │               │◄───┤  KlippyHttpServer     │  │
│  │               │    │  (HTTP/WS transport)  │  │
│  │               │    │  Port 7125            │  │
│  └──────────────┘    └───────────────────────┘  │
└────────────────────────┬────────────────────────┘
                         │
                    Port 7125
                         │
┌────────────────────────┴────────────────────────┐
│              Mainsail Container                  │
│                                                  │
│  Nginx serves Mainsail static assets             │
│  Proxies /server, /printer, /machine, /api       │
│  to Tether on port 7125                          │
└──────────────────────────────────────────────────┘
```

Tether's `KlippyServer` holds all business logic (endpoints, state, data
stores). The `KlippyHttpServer` is a thin HTTP/WebSocket transport that
exposes the full Moonraker HTTP + WebSocket API. Mainsail connects to it
as if it were a real Moonraker instance — no Moonraker process needed.

## Prerequisites

- Docker and Docker Compose
- Tether built with HTTP support:
  ```bash
  cmake -B build \
    -DTETHER_ENABLE_KLIPPER=1 \
    -DTETHER_ENABLE_KLIPPER_HTTP=1
  cmake --build build --target klipper_http_mainsail -j$(nproc)
  ```

## Quick Start

### 1. Create the Dockerfile

Create `docker/Dockerfile.tether`:

```dockerfile
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake g++ ninja-build git \
    libjsoncpp-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tether
COPY . .

RUN cmake -B build \
    -DTETHER_ENABLE_KLIPPER=1 \
    -DTETHER_ENABLE_KLIPPER_HTTP=1 \
    -DTETHER_BUILD_TESTS=OFF \
    -DTETHER_BUILD_EXAMPLES=ON \
    && cmake --build build --target klipper_http_mainsail -j$(nproc)

EXPOSE 7125

CMD ["/opt/tether/build/bin/klipper_http_mainsail", \
     "--port", "7125", \
     "--no-auth", \
     "--gcodes-root", "/data/gcodes", \
     "--config-root", "/data/config", \
     "--logs-root", "/data/logs"]
```

### 2. Create the Docker Compose file

Create `docker-compose.yml`:

```yaml
version: "3.8"

services:
  tether:
    build:
      context: .
      dockerfile: docker/Dockerfile.tether
    ports:
      - "7125:7125"
    volumes:
      - tether-data:/data
    restart: unless-stopped

  mainsail:
    image: ghcr.io/mainsail-crew/mainsail:latest
    ports:
      - "80:80"
    environment:
      - TZ=UTC
    volumes:
      - mainsail-config:/etc/nginx/conf.d
    depends_on:
      - tether
    restart: unless-stopped

volumes:
  tether-data:
  mainsail-config:
```

### 3. Configure Mainsail to connect to Tether

Create `docker/mainsail/default.conf`:

```nginx
server {
    listen 80;
    server_name _;

    root /usr/share/nginx/html;
    index index.html;

    # Serve Mainsail SPA
    location / {
        try_files $uri $uri/ /index.html;
    }

    # Proxy Moonraker API to Tether
    location /server/ {
        proxy_pass http://tether:7125/server/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    location /printer/ {
        proxy_pass http://tether:7125/printer/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    location /machine/ {
        proxy_pass http://tether:7125/machine/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    location /api/ {
        proxy_pass http://tether:7125/api/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    location /access/ {
        proxy_pass http://tether:7125/access/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    # WebSocket proxy
    location /websocket {
        proxy_pass http://tether:7125/websocket;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_read_timeout 86400;
    }
}
```

Update `docker-compose.yml` to mount the config:

```yaml
  mainsail:
    image: ghcr.io/mainsail-crew/mainsail:latest
    ports:
      - "80:80"
    volumes:
      - ./docker/mainsail/default.conf:/etc/nginx/conf.d/default.conf:ro
    depends_on:
      - tether
    restart: unless-stopped
```

### 4. Start the stack

```bash
docker compose up -d
```

### 5. Access Mainsail

Open `http://localhost/` in your browser. Mainsail will connect to Tether
via the WebSocket proxy and display the printer dashboard.

## Running Without Docker

You can also run Tether directly and serve Mainsail's static assets from
the same process:

```bash
# Download Mainsail release
wget https://github.com/mainsail-crew/mainsail/releases/latest/download/mainsail.zip
unzip mainsail.zip -d /opt/mainsail

# Run Tether with Mainsail static assets
./build/bin/klipper_http_mainsail \
    --port 7125 \
    --web-root /opt/mainsail \
    --gcodes-root /home/pi/gcodes \
    --no-auth
```

Then open `http://localhost:7125/` — Tether serves both the API and the
Mainsail SPA from the same port.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--port` | 7125 | HTTP listen port |
| `--uds-path` | /tmp/klippy_uds | UDS socket path |
| `--web-root` | (disabled) | Directory for Mainsail static assets |
| `--gcodes-root` | /tmp/tether_sdcard | G-code file root |
| `--config-root` | /etc/tether | Config file root |
| `--logs-root` | /var/log | Log file root |
| `--api-key` | tether_default_api_key | API key for auth |
| `--no-auth` | (auth enabled) | Disable authentication |

## Transport-Agnostic Architecture

The refactored architecture separates business logic from transport:

- **KlippyServer** — All endpoint handlers, state management, data stores
  (job history, users, database, power devices, webcams, etc.)
- **KlippyUdsServer** — Thin UDS transport (socket lifecycle, frame
  parsing, UDS-specific subscriptions). Delegates to KlippyServer.
- **KlippyHttpServer** — Thin HTTP/WebSocket transport (Drogon routes,
  JSON-RPC, WebSocket sessions). Delegates to the same KlippyServer.

Both transports share a single KlippyServer instance, so there is zero
business-logic duplication. Adding a new transport (e.g., a raw TCP
JSON-RPC protocol) only requires implementing the transport layer and
delegating to KlippyServer.

### Using KlippyServer in Your Application

```cpp
#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/http/KlippyHttpServer.hpp"

using namespace tether::klipper::klippy;
using namespace tether::klipper::http;

// 1. Create the shared server
UdsServerConfig cfg;
cfg.socketPath = "/tmp/klippy_uds";
KlippyServer server(cfg);
server.setState(PrinterState::Ready, "Ready");

// 2. Create UDS transport (shares the server)
KlippyUdsServer uds(server, cfg);
uds.start();

// 3. Create HTTP transport (shares the same server)
HttpServerConfig httpCfg;
httpCfg.port = 7125;
httpCfg.requireAuth = false;
auto http = std::make_shared<KlippyHttpServer>(server, httpCfg);
http->start();

// ... run your application ...

http->stop();
uds.stop();
```
