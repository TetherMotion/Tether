# Application Profile Trust and Loading Rules

Defines how declarative application profiles (`machine.app.profile`) are
signed, verified, validated, and loaded (WEBINTERFACE-PLAN items 86 and 92).

## Trust model

- A profile document is trusted only when a **keyed BLAKE3 MAC** verifies
  against a key provisioned into the server. Plain hashes are not used;
  `signProfileDocument`/`verifyProfileDocument` in
  `include/tether/io/ApplicationProfile.hpp` implement a real MAC with
  constant-time comparison.
- The **signing key never ships with the profile**. Signing happens offline
  (CI or a signing tool); deployments install only the verifying key into the
  server's profile source. The demo key in `SimulatedMachineStack` is a fixed
  test value and is explicitly not a secret.
- The browser never verifies and never trusts the document's contents on its
  own; the server is the single integrity authority. The client only renders
  the narrow declarative contract (panels/widgets) after the server has
  accepted it.

## Loading rules

1. `StaticAppProfileSource::install(name, version, document, mac)` is the
   only ingestion path. It is called at startup / attach time, not per
   session.
2. `install` first verifies the MAC; a mismatched or missing MAC rejects the
   profile — no parse of unverified bytes.
3. `install` then calls `validateAppProfileDocument`, which enforces the
   `tether.app.profile.v1` contract: format marker, ≤256 KiB document,
   ≤64 panels with unique ids and titles, ≤64 widgets per panel, and the
   per-kind required fields (`entry` for display kinds, `axis` for jog/
   command, `action` integer for command, `fn` for button).
4. A profile that fails either step is refused and the `machine.app.profile`
   signal is simply absent — clients show no custom panels (fail-closed).
5. When the toolchain cannot compile the JSON validator (Glaze disabled),
   `install` refuses every document rather than serving unvalidated content.
6. Control widgets (`jog`, `command`, `button`) route only through the
   existing authority-gated `machine.command` / function surface — the
   profile can *name* an action but cannot bypass role, lease, interlock, or
   generation checks. Display widgets can never write.
7. Replacing the profile requires a new signed document installed through
   `install` — there is no client-initiated profile mutation path.

## Verification

`tests/io/test_io_machine_service.cpp`:
`AppProfileSourceValidatesDocumentContract` covers signed-but-malformed
documents (bad format, missing required fields, unknown widget kinds,
duplicate panel ids, oversized documents) and acceptance of valid
documents; `AppProfileSignalServesVerifiedDocument` covers the wire path.
