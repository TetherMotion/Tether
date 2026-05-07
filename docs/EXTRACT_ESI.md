# extract_esi_xml utility

A small host utility to extract and print human-readable information from EtherCAT ESI (XML) files.

Usage:

  extract_esi <esi.xml> [--device-index N] [--mailbox] [--sync-managers]

- By default prints a human-readable summary of the selected device (index 0 by default).
- `--mailbox` shows only mailbox information.
- `--sync-managers` prints only sync-manager configuration.
- `--json` emits structured JSON containing: `mailbox`, `syncManagers`, `fmmus`, `rxPdos`, `txPdos` (useful for scripts).

The parser now extracts additional ESI information:
- PDO mappings (`<RxPdo>` / `<TxPdo>`), including entries (index/subindex/bitlen/type/name).
- Simple FMMU listing (`<Fmmu>` names).

Build (default):

The `extract_esi` utility is built by default as part of a normal Tether build and the
binary is placed in the build `bin` directory.

To build manually or force the option:

  cmake -S Tether -B build -DTETHER_BUILD_EXTRACT_ESI=ON
  cmake --build build --target extract_esi

To disable building the utility:

  cmake -S Tether -B build -DTETHER_BUILD_EXTRACT_ESI=OFF

This tool uses `tinyxml2` (fetched automatically if not available) and depends on the
`ethercat_common` component.
