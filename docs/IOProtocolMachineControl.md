# Tether IO Protocol — Machine Control, Auth & Authority

The machine layer gates mutating commands behind authenticated
identities and exclusive authority leases. The protocol's generic
surface (params/signals/functions) is unchanged — enforcement lives in
`machine::ControlAuthority`, `machine::MachineService`, and the
credential path. The CiA-402 profile and simulated fleet:
[IOProtocolMachineProfile.md](IOProtocolMachineProfile.md);
end-to-end wiring: `examples/tether_io_machine_auth.cpp`.

## Roles and actions

```cpp
enum class Role : uint8_t { Observer, Operator, Technician,
                            ControlsEngineer, Administrator };

enum class Action : uint8_t {
    Enable, Disable, QuickStop, FaultReset, Recover, SetMode,
    JogStart, JogRenew, JogStop, StepMove, MoveToPosition, GroupMove,
    HomePrepare, HomeStart, HomeCancel, ConfigurationStage,
    ConfigurationValidate, ConfigurationCommit, ConfigurationRollback,
    CancelOperation, AcknowledgeAlarm
};
```

`SessionIdentity{sessionId, actor, source, role, authenticated}` —
the resolved identity carried into every gated call. Unauthenticated
sessions default to `Role::Observer`/`authenticated=false`.

## Credentials → identity

`IMachineAuthProvider::authenticate(credential) →
optional<SessionIdentity>` resolves a bearer token. The built-in
`StaticTokenAuthProvider` compares tokens in constant time and never
logs them:

```cpp
auto provider = machine::StaticTokenAuthProvider::fromFile("tokens.txt");
// tokens.txt: <token>\t<actor>\t<role>
//             roles: observer | operator | technician | engineer | admin
auto identity = provider.authenticate(*machine::bearerCredential(
                    "Bearer <client-token>"));
// nullopt → reject; SessionIdentity{actor, source="bearer-token",
//                                  role, authenticated=true} otherwise
```

`bearerCredential(authorization)` extracts the token from an HTTP
`Authorization: Bearer …` value — the format a WebSocket transport
(`WsClientTransport::Options::headers`) carries; rejects
absent/malformed/non-Bearer values and tokens containing whitespace.
`fromFile` fails hard on a malformed line or an empty usable set —
deployment misconfiguration is a startup error, not a silent
Observer downgrade.

## ControlAuthority — exclusive leases, fail-closed audit

`ControlAuthority` (constructed server-side, app-executor thread —
**not** the cyclic thread) issues time-boxed exclusive leases per
scope:

```cpp
machine::ControlAuthority authority(
    /*maximumLease*/ 30s,
    /*audit*/  [](const machine::AuthorityEvent& e){ /* persist */ },
    /*eligibleRoles*/ {Role::Operator, Role::Technician,
                       Role::ControlsEngineer});

auto r = authority.acquire("gantry", identity, /*requested*/ 10s);
if (!r.lease) reject(r.blocker);           // blocker explains why
```

`AuthorityResult{lease, blocker}`; `AuthorityLease{token, scope,
ownerSession, ownerActor, ownerRole, expiresAt}`.

| Acquire/renew blocker | Cause |
|---|---|
| scope empty | machine/motion-group scope required |
| unauthenticated / no session | auth required |
| role not in `eligibleRoles` | observer/admin by default can't acquire |
| `requested` out of `(0, maximumLease]` | outside server policy |
| scope held by another session | exclusive ownership |
| audit record fails | **lease revoked before grant** — fail closed |

Semantics:

- **Exclusive per scope** — a second session is blocked while the
  lease lives; same-session re-acquire renews.
- **Token-checked** — `renew`/`release` require the token +
  owning session; expiry sweeps stale leases before each call.
- **Fail-closed audit** — a grant the audit sink can't record is
  rolled back (`"Authority audit service is unavailable"`). No audit
  callback → every acquire fails: the server must persist, or nothing
  controls the machine.
- Every event (grant/deny/renew/release/expire) is recorded through
  `Audit` — the audit trail is the machine-safety record.

## MachineService — identity-gated dispatch

```cpp
machine::MachineService service(stateSource, dispatcher, authority,
                                gate /*, …*/);
service.setAuxiliaryServices(&capture, /*…*/);
```

Every mutating operation takes a `SessionIdentity` — enable/disable,
quick-stop, jog, moves, homing, config stage/validate/commit/rollback,
alarm acknowledge — and consults `authority_`/`gate_` before the
`IMachineDispatcher` acts. Read paths (descriptor, state signals,
event journal) are open to observers; writes are not.

- `stateSource` supplies the machine/drive snapshots the catalog
  publishes.
- `dispatcher` executes accepted commands (`CommandState`:
  InProgress/Accepted/Rejected/Failed).
- `gate` applies policy beyond role gating (interlocks, mode
  restrictions).

## Wiring it over the wire

Auth attaches at the transport/session boundary — Tether IO itself
doesn't authenticate; the front door does:

1. Client presents the credential in the WS handshake
   (`Authorization: Bearer <token>` header) or a
   connection-time message.
2. Server resolves it through `IMachineAuthProvider` →
   `SessionIdentity`.
3. The identity rides `FunctionInvokeContext::identity` into every
   function callback — server functions and machine services see the
   *same* identity the credential produced.
4. Mutating `machine.*` functions run the authority/gate check with
   that identity; a lease token carried in the call authorizes
   scope-exclusive operations.

See `examples/tether_io_machine_auth.cpp` for the token→identity→
acquire→deny chain end to end.
