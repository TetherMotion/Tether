# Tether IO Protocol — Transports

Transports move protocol *messages*, not the protocol itself — the
same `Server`/`TetherIOClient`/`Session` stack runs over TCP, serial,
WebSocket, or an in-process pipe unchanged. Wire/framing details:
[IOProtocolWireFormat.md](IOProtocolWireFormat.md#link-framing).

## The contract

`ITransport` (one endpoint):

| Method | Semantics |
|---|---|
| `send(data, len)` | blocking write of one complete message; `false` on failure |
| `receive(buf, maxLen, timeoutMs)` | byte-stream receive; `0` on timeout. Byte-oriented transports only |
| `receiveMessage(out, timeoutMs)` | **message-oriented** receive; default impl reads a byte stream. Override for message transports |
| `close()` | release the endpoint |
| `isConnected()` | live check |

`ITransportServer` (acceptor):

| Method | Semantics |
|---|---|
| `start()` / `stop()` | listener lifecycle |
| `accept()` | next pending `ITransport` — blocks or returns null |
| `isListening()` | state check |

**Byte-stream vs message-oriented is the one distinction that
matters**: TCP, serial, and pipes deliver bytes, so they need a
framing layer; a message transport delivers whole protocol messages
and uses `Framing::None` (`ServerConfig::framing`, client-side match).

`Session`/`TetherIOClient` own exactly one transport each — a serial
link is point-to-point: the server side is a bare `Session`, not an
`ITransportServer` (see `examples/tether_io_serial_transport.cpp`).

## Built-in transports

| Transport | Orient. | Framing | Notes |
|---|---|---|---|
| `TcpTransportServer` / `tcpConnect(host, port, timeout)` | byte | **must wrap in `SlipFramedTransport`** | `ServerConfig::listenPort`, `maxClients` (default 4) |
| `SerialTransport` + `PosixSerialDriver` | byte | SLIP | termios 8N1, `driver.fd()` for advanced use |
| `WsClientTransport` | **message** | `Framing::None` | RFC 6455: masking, fragmentation reassembly, ping/pong/close; `handshake()` with `Sec-WebSocket-Accept` check, `maxMessageSize` bound; wraps a connected byte transport; no TLS (`ws://` only) |
| test `PipeTransport` / custom pipe | message | `Framing::None` | the in-process pattern — see `examples/tether_io_pipe_transport.cpp` |

## SLIP framing — when it's needed

Default `ServerConfig::framing = Framing::Slip`. Over TCP:

```cpp
auto raw = tcpConnect("127.0.0.1", 4000, 2000);
TetherIOClient client(std::make_unique<SlipFramedTransport>(std::move(raw)),
                      /*timeoutMs*/ 2000);
```

Byte-stream `receive()` has no message boundaries — SLIP supplies
them: `0xC0` terminates a frame; `0xDB 0xDC`/`0xDB 0xDD` escape
`0xC0`/`0xDB`. `SlipFramedTransport` handles encode/decode and
enforces `MAX_MESSAGE_SIZE`.

A serial link is SLIP-framed the same way — `SerialTransport` is a
byte stream; wrap it in `SlipFramedTransport` on **both** ends.

## WebSocket — message framing changes the setup

`WsClientTransport` carries **one protocol message per binary WS
frame** — no SLIP. Consequences:

- Client wraps `tcpConnect` → `WsClientTransport` → `handshake()`
  (`{host, target="/tether-io", headers, handshakeTimeoutMs}`).
- The matching server runs `Framing::None` — a mixed fleet needs a
  second `Server` port or a WS-aware acceptor.
- Control frames (ping/pong/close) are handled inside
  `receiveMessage`.

## Writing a custom transport

The working recipe (`examples/tether_io_pipe_transport.cpp`):

1. **Endpoint**: `send` copies into a shared `std::deque` + CV;
   `receiveMessage` blocks on the CV, returns false on close;
   `isConnected` checks a shared flag. Both ends share two queues
   (one per direction).
2. **Acceptor**: `accept()` pops a `std::unique_ptr<ITransport>`
   queue guarded by mutex+CV; `connect()` (example-specific) builds
   the endpoint pair and enqueues the server half.
3. **Config**: `Framing::None` for message transports —
   `ServerConfig::framing` and the client's matching expectation.

Invariants to honor:

- `send` delivers **complete messages** or fails — no partial-write
  semantics to emulate TCP.
- `receiveMessage`/`receive` return within their timeout budget —
  `Session` multiplexes peer-call responses and stream data through
  it; a transport that blocks forever starves the pump.
- `close()` wakes pending `receive` calls — session teardown relies
  on it.
- A message transport never splits one protocol message across two
  `receiveMessage` returns and never joins two into one.

## Picking a transport

| Situation | Transport |
|---|---|
| Production daemon, LAN | TCP + SLIP (`TcpTransportServer`) |
| Firmware/UART link | `SerialTransport` + `SlipFramedTransport`, `Session` server-side |
| Browser/REST-adjacent frontend | `WsClientTransport`, `Framing::None` |
| Unit test / simulation loopback | pipe transport, `Framing::None` |
| Multiple concurrent clients | `TcpTransportServer`, `maxClients` bound |

## Common pitfalls

- **Unwrapped TCP** — `tcpConnect` is a byte stream; skipping
  `SlipFramedTransport` leaves the session unable to parse frames
  (connects, then deadlocks/timeouts).
- **Server without SLIP on TCP** — `Server` frames its side from
  `ServerConfig::framing`; client and server framing must match.
- **Expecting `accept()` on serial** — serial is a Session, not an
  ITransportServer.
- **WS + SLIP** — WS already frames; adding SLIP corrupts the stream.
