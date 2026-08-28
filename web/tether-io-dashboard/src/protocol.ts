/**
 * @file protocol.ts
 * @brief Binary protocol primitives for the Tether IO wire format.
 *
 * This module implements the TypeScript-side encoding/decoding of the Tether
 * IO binary protocol used over WebSocket (Framing::None — each WebSocket
 * binary message is exactly one protocol message, no SLIP framing).
 *
 * The protocol is little-endian and uses these primitive types:
 *   - u8/u16/u32/u64  — fixed-width little-endian integers
 *   - varint          — LEB128-style variable-length unsigned integer
 *   - string16        — u16 length prefix followed by UTF-8 bytes
 *
 * See the C++ side (include/tether/io/Protocol.hpp) for the authoritative
 * message-type definitions.
 */

// ---------------------------------------------------------------------------
// Message type identifiers
// ---------------------------------------------------------------------------

/**
 * Numeric identifiers for every Tether IO protocol message.
 *
 * The first byte of every protocol message is one of these values.  They
 * match the `MessageType` enum on the C++ server side
 * (include/tether/io/Protocol.hpp).
 */
export const MessageType = {
  listParamsReq: 0x01,
  listParamsResp: 0x02,
  configureStream: 0x03,
  configureAck: 0x04,
  startStream: 0x05,
  stopStream: 0x06,
  streamData: 0x07,
  error: 0x08,
  setParameterReq: 0x0b,
  setParameterResp: 0x0c,
  listSignalsReq: 0x20,
  listSignalsResp: 0x21,
  getParamReq: 0x22,
  getParamResp: 0x23,
  getSignalReq: 0x24,
  getSignalResp: 0x25,
  listFunctionsReq: 0x35,
  listFunctionsResp: 0x36,
  callFunctionReq: 0x37,
  callFunctionResp: 0x38,
} as const;

// ---------------------------------------------------------------------------
// Value types
// ---------------------------------------------------------------------------

/**
 * Enumeration of value types carried by parameters, signals, and function
 * arguments.  Mirrors `ValueType` on the C++ side.
 */
export enum ValueType {
  U8 = 1,
  U16,
  U32,
  U64,
  I8,
  I16,
  I32,
  I64,
  F32,
  F64,
  Bool,
  String,
  Binary,
  IPv4,
  IPv6,
  MAC,
  Enum,
  UVarint,
  IVarint,
  Struct,
  Array,
  Stream,
}

// ---------------------------------------------------------------------------
// Data interfaces
// ---------------------------------------------------------------------------

/**
 * One entry in the parameter or signal catalog.
 *
 * `kind` is set by the caller (the client) to distinguish parameters from
 * signals — the wire format does not carry this information because the
 * request type (ListParamsReq vs ListSignalsReq) already implies it.
 */
export interface CatalogEntry {
  /** Unique 64-bit identifier assigned by the server. */
  id: bigint;
  /** Value type (F64, U32, String, …). */
  type: ValueType;
  /** Fixed payload size in bytes, or 0 for variable-length entries. */
  valueSize: number;
  /** Bitmask: bit 0 = readable, bit 1 = writable, bit 2 = variable-length. */
  flags: number;
  /** Human-readable name (e.g. "amplitude"). */
  name: string;
  /** Longer description (e.g. "Sine/cosine wave amplitude"). */
  description: string;
  /** Logical grouping (e.g. "wave", "pid.axis0"). */
  group: string;
  /** Whether this entry is a parameter (writable) or a signal (read-only). */
  kind: 'param' | 'signal';
}

/**
 * One parameter of a remotely callable function.
 */
export interface FunctionParameter {
  name: string;
  description: string;
  type: ValueType;
  flags: number;
  maxValueSize: number;
  /** Present when the parameter has a default value (flag bit 1). */
  defaultValue?: Uint8Array;
  /** Optional structured value descriptor (not yet decoded by the UI). */
  descriptor?: unknown;
}

/**
 * One entry in the function catalog.
 */
export interface FunctionEntry {
  id: bigint;
  name: string;
  description: string;
  group: string;
  parameters: FunctionParameter[];
  /** Whether the function returns a value. */
  returnPresent: boolean;
  /** Type of the return value, if `returnPresent` is true. */
  returnType?: ValueType;
}

/**
 * One slot in a configured stream's layout (returned by ConfigureStreamAck).
 */
export interface StreamLayoutEntry {
  id: bigint;
  type: ValueType;
  valueSize: number;
}

/**
 * A single row of streamed data.
 *
 * `values[i]` corresponds to `layout[i]` from the ConfigureStreamAck.
 */
export interface StreamRow {
  specId: number;
  timestampUs: bigint;
  values: Uint8Array[];
}

// ---------------------------------------------------------------------------
// BinaryReader — sequential little-endian reader
// ---------------------------------------------------------------------------

/**
 * Sequential little-endian binary reader.
 *
 * Every method advances an internal cursor and returns the value at the
 * current position.  Throws `Error` on truncation or trailing data.
 */
export class BinaryReader {
  private offset = 0;
  private readonly view: DataView;

  /** @param input The buffer to read from (a copy is not made). */
  constructor(private readonly input: Uint8Array) {
    this.view = new DataView(input.buffer, input.byteOffset, input.byteLength);
  }

  /** Number of bytes still unread. */
  get remaining(): number {
    return this.view.byteLength - this.offset;
  }

  /** Advance the cursor by `size` bytes, returning the previous offset. */
  private take(size: number): number {
    if (size < 0 || this.remaining < size) throw new Error('truncated packet');
    const start = this.offset;
    this.offset += size;
    return start;
  }

  /** Read one unsigned 8-bit integer. */
  u8(): number {
    return this.view.getUint8(this.take(1));
  }

  /** Read one unsigned 16-bit little-endian integer. */
  u16(): number {
    return this.view.getUint16(this.take(2), true);
  }

  /** Read one unsigned 32-bit little-endian integer. */
  u32(): number {
    return this.view.getUint32(this.take(4), true);
  }

  /** Read one unsigned 64-bit little-endian integer as a `bigint`. */
  u64(): bigint {
    return this.view.getBigUint64(this.take(8), true);
  }

  /** Read `size` raw bytes as a sub-array copy. */
  bytesOf(size: number): Uint8Array {
    const start = this.take(size);
    return this.input.slice(start, start + size);
  }

  /**
   * Read a LEB128-style variable-length unsigned integer (up to 35 bits).
   * Each byte contributes 7 payload bits; the high bit signals continuation.
   */
  varint(): number {
    let value = 0;
    for (let shift = 0; shift < 35; shift += 7) {
      const byte = this.u8();
      value += (byte & 0x7f) * 2 ** shift;
      if (!(byte & 0x80)) return value;
    }
    throw new Error('invalid varint');
  }

  /** Read a UTF-8 string prefixed by a u16 length. */
  string16(): string {
    return new TextDecoder().decode(this.bytesOf(this.u16()));
  }

  /** Assert that the cursor is exactly at the end of the buffer. */
  assertEnd(): void {
    if (this.remaining !== 0) throw new Error('trailing packet data');
  }
}

// ---------------------------------------------------------------------------
// BinaryWriter — sequential little-endian writer
// ---------------------------------------------------------------------------

/**
 * Sequential little-endian binary writer with a fixed-size backing buffer.
 *
 * Methods are chainable (each returns `this`) so that request messages can
 * be composed fluently, e.g. `new BinaryWriter(9).u8(type).u32(0).u32(n).finish()`.
 */
export class BinaryWriter {
  private readonly output: Uint8Array;
  private readonly view: DataView;
  private offset = 0;

  /** @param size Capacity in bytes. Throws if exceeded. */
  constructor(size = 256) {
    this.output = new Uint8Array(size);
    this.view = new DataView(this.output.buffer);
  }

  /** Reserve `size` bytes and return the offset of the first byte. */
  private room(size: number): number {
    if (this.offset + size > this.output.length) throw new Error('packet too large');
    const start = this.offset;
    this.offset += size;
    return start;
  }

  /** Write one unsigned 8-bit integer. */
  u8(value: number): this {
    this.view.setUint8(this.room(1), value);
    return this;
  }

  /** Write one unsigned 16-bit little-endian integer. */
  u16(value: number): this {
    this.view.setUint16(this.room(2), value, true);
    return this;
  }

  /** Write one unsigned 32-bit little-endian integer. */
  u32(value: number): this {
    this.view.setUint32(this.room(4), value, true);
    return this;
  }

  /** Write one unsigned 64-bit little-endian integer. */
  u64(value: bigint): this {
    this.view.setBigUint64(this.room(8), value, true);
    return this;
  }

  /** Write a run of raw bytes. */
  bytes(value: Uint8Array): this {
    this.output.set(value, this.room(value.length));
    return this;
  }

  /** Write a LEB128-style variable-length unsigned integer. */
  varint(value: number): this {
    do {
      const byte = value % 128;
      value = Math.floor(value / 128);
      this.u8(byte | (value ? 0x80 : 0));
    } while (value);
    return this;
  }

  /** Return a compact copy of the written bytes. */
  finish(): Uint8Array {
    return this.output.slice(0, this.offset);
  }
}

// ---------------------------------------------------------------------------
// SLIP framing (legacy, unused for WebSocket transport)
// ---------------------------------------------------------------------------

/**
 * SLIP-encode a payload (RFC 1055 framing with 0xC0 end marker).
 *
 * @deprecated The WebSocket transport uses Framing::None — each WebSocket
 *             binary message is already one protocol message, so SLIP is not
 *             needed.  Kept for compatibility with serial/TCP transports.
 */
export function slipEncode(payload: Uint8Array): Uint8Array {
  const result: number[] = [];
  for (const byte of payload) {
    if (byte === 0xc0) result.push(0xdb, 0xdc);
    else if (byte === 0xdb) result.push(0xdb, 0xdd);
    else result.push(byte);
  }
  result.push(0xc0);
  return Uint8Array.from(result);
}

/**
 * Streaming SLIP decoder.  Feed it arbitrary chunks of bytes and it yields
 * complete frames as they become available.
 *
 * @deprecated Not used by the WebSocket transport (see {@link slipEncode}).
 */
export class SlipDecoder {
  private frame: number[] = [];
  private escaped = false;
  private discarding = false;

  /** @param maxFrameSize Maximum bytes per frame before forcing a resync. */
  constructor(private readonly maxFrameSize = 1024 * 1024) {}

  /**
   * Push a chunk of bytes and return any complete frames decoded so far.
   */
  push(chunk: Uint8Array): Uint8Array[] {
    const frames: Uint8Array[] = [];
    for (const byte of chunk) {
      if (this.discarding) {
        if (byte === 0xc0) {
          this.discarding = false;
          this.frame = [];
        }
        continue;
      }
      if (byte === 0xc0) {
        if (this.frame.length) frames.push(Uint8Array.from(this.frame));
        this.frame = [];
        this.escaped = false;
        continue;
      }
      if (this.escaped) {
        if (byte === 0xdc) this.frame.push(0xc0);
        else if (byte === 0xdd) this.frame.push(0xdb);
        else {
          this.frame = [];
          this.discarding = true;
        }
        this.escaped = false;
      } else if (byte === 0xdb) {
        this.escaped = true;
      } else {
        this.frame.push(byte);
      }
      if (this.frame.length > this.maxFrameSize) {
        this.frame = [];
        this.escaped = false;
        this.discarding = true;
      }
    }
    return frames;
  }
}

// ---------------------------------------------------------------------------
// Request builders
// ---------------------------------------------------------------------------

/** Encode a standalone varint (convenience wrapper). */
export function encodeVarint(value: number): Uint8Array {
  return new BinaryWriter(5).varint(value).finish();
}

/**
 * Build a ListParamsReq / ListSignalsReq / ListFunctionsReq message.
 *
 * All three share the same body layout: `offset(u32) + maxCount(u32)`.
 *
 * @param type   One of `MessageType.listParamsReq`, `listSignalsReq`, `listFunctionsReq`.
 * @param offset Zero-based index of the first entry to return.
 * @param count  Maximum number of entries to return.
 */
export function makeListRequest(type: number, offset = 0, count = 1000): Uint8Array {
  return new BinaryWriter(9).u8(type).u32(offset).u32(count).finish();
}

/**
 * Build a GetParamReq / GetSignalReq message.
 *
 * Body layout: `id(u64)`.
 *
 * @param type One of `MessageType.getParamReq`, `getSignalReq`.
 * @param id   Identifier of the entry to read.
 */
export function makeGetRequest(type: number, id: bigint): Uint8Array {
  return new BinaryWriter(9).u8(type).u64(id).finish();
}

// ---------------------------------------------------------------------------
// Value decoding
// ---------------------------------------------------------------------------

/**
 * Decode a raw byte payload into a typed JavaScript value.
 *
 * @returns A `number`, `bigint`, `boolean`, `string`, or `Uint8Array`
 *          depending on `type`.
 */
export function decodeValueBytes(
  bytes: Uint8Array,
  type: ValueType,
): number | bigint | boolean | string | Uint8Array {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  switch (type) {
    case ValueType.U8:
      return view.getUint8(0);
    case ValueType.U16:
      return view.getUint16(0, true);
    case ValueType.U32:
      return view.getUint32(0, true);
    case ValueType.U64:
      return view.getBigUint64(0, true);
    case ValueType.I8:
      return view.getInt8(0);
    case ValueType.I16:
      return view.getInt16(0, true);
    case ValueType.I32:
      return view.getInt32(0, true);
    case ValueType.I64:
      return view.getBigInt64(0, true);
    case ValueType.F32:
      return view.getFloat32(0, true);
    case ValueType.F64:
      return view.getFloat64(0, true);
    case ValueType.Bool:
      return view.getUint8(0) !== 0;
    case ValueType.String:
      return new TextDecoder().decode(bytes);
    default:
      return bytes;
  }
}

/**
 * Format a decoded value for human-readable display.
 *
 * - Byte arrays are rendered as hex strings.
 * - BigInts are rendered as decimal strings.
 * - Everything else uses `String(...)`.
 */
export function formatValue(value: ReturnType<typeof decodeValueBytes>): string {
  if (value instanceof Uint8Array)
    return `0x${[...value].map((byte) => byte.toString(16).padStart(2, '0')).join('')}`;
  return typeof value === 'bigint' ? value.toString() : String(value);
}

// ---------------------------------------------------------------------------
// Response parsers
// ---------------------------------------------------------------------------

/**
 * Parse a ListParamsResp / ListSignalsResp payload into catalog entries.
 *
 * The `kind` field defaults to `'signal'` and is overwritten by the caller
 * (`TetherIOClient.list`) when the request was for parameters.
 *
 * Wire layout:
 *   type(u8) + total(u32) + returnedOffset(u32) + count(u32)
 *   + count × [id(u64) + type(u8) + valueSize(u8) + flags(u8)
 *               + name(string16) + description(string16) + group(string16)]
 */
export function readEntryCatalog(payload: Uint8Array): CatalogEntry[] {
  const reader = new BinaryReader(payload);
  reader.u8(); // type
  reader.u32(); // total
  reader.u32(); // returnedOffset
  const count = reader.u32();
  const entries: CatalogEntry[] = [];
  for (let i = 0; i < count; i += 1)
    entries.push({
      id: reader.u64(),
      type: reader.u8() as ValueType,
      valueSize: reader.u8(),
      flags: reader.u8(),
      name: reader.string16(),
      description: reader.string16(),
      group: reader.string16(),
      kind: 'signal',
    });
  reader.assertEnd();
  return entries;
}

/**
 * Parse a ListFunctionsResp payload into function catalog entries.
 *
 * Wire layout (per function):
 *   id(u64) + name(string16) + description(string16) + group(string16)
 *   + paramCount(u32) + paramCount × [name + description + type(u8) + flags(u8)
 *       + enumRef(u64) + structRef(u64) + maxValueSize(u32)
 *       + hasDescriptor(u8) + [descriptor(u32-length-prefixed)]
 *       + [if flags & 2] defaultValue(varint-length-prefixed)
 *       + metadataCount(u32) + metadataCount × [key(string16) + value(string16)]]
 *   + hasReturn(u8) + [if hasReturn] [name + description + type(u8) + flags(u8)
 *       + enumRef(u64) + structRef(u64) + maxValueSize(u32)
 *       + hasDescriptor(u8) + [descriptor] + metadataCount × [key + value]]
 *   + metadataCount(u32) + metadataCount × [key(string16) + value(string16)]
 */
export function readFunctionCatalog(payload: Uint8Array): FunctionEntry[] {
  const reader = new BinaryReader(payload);
  reader.u8(); // type
  reader.u32(); // total
  reader.u32(); // returnedOffset
  const count = reader.u32();
  const functions: FunctionEntry[] = [];
  for (let i = 0; i < count; i += 1) {
    const id = reader.u64();
    const name = reader.string16();
    const description = reader.string16();
    const group = reader.string16();
    const parameters: FunctionParameter[] = [];
    for (let p = 0, total = reader.u32(); p < total; p += 1) {
      const parameter = {
        name: reader.string16(),
        description: reader.string16(),
        type: reader.u8() as ValueType,
        flags: reader.u8(),
        maxValueSize: 0,
      } as FunctionParameter;
      reader.u64(); // enumReference (unused by UI)
      reader.u64(); // structReference (unused by UI)
      parameter.maxValueSize = reader.u32();
      if (reader.u8()) reader.bytesOf(reader.u32()); // skip value descriptor
      if (parameter.flags & 2) parameter.defaultValue = reader.bytesOf(reader.varint());
      // Skip per-parameter metadata
      for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) {
        reader.string16();
        reader.string16();
      }
      parameters.push(parameter);
    }
    const returnPresent = reader.u8() !== 0;
    let returnType: ValueType | undefined;
    if (returnPresent) {
      reader.string16(); // return name
      reader.string16(); // return description
      returnType = reader.u8() as ValueType;
      reader.u8(); // return flags
      reader.u64(); // enumReference
      reader.u64(); // structReference
      reader.u32(); // maxValueSize
      if (reader.u8()) reader.bytesOf(reader.u32()); // skip descriptor
      // Skip return-value metadata
      for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) {
        reader.string16();
        reader.string16();
      }
    }
    // Skip per-function metadata
    for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) {
      reader.string16();
      reader.string16();
    }
    functions.push({ id, name, description, group, parameters, returnPresent, returnType });
  }
  reader.assertEnd();
  return functions;
}

/**
 * Parse a ConfigureStreamAck payload.
 *
 * Wire layout:
 *   type(u8) + specId(u32) + count(u32) + rowSize(u32)
 *   + count × [id(u64) + type(u8) + valueSize(u8)]
 *
 * @returns The stream spec ID, the row size in bytes, and the layout
 *          (one entry per streamed value).
 */
export function readConfigureAck(payload: Uint8Array): {
  specId: number;
  rowSize: number;
  layout: StreamLayoutEntry[];
} {
  const reader = new BinaryReader(payload);
  reader.u8(); // type
  const specId = reader.u32();
  const count = reader.u32();
  const rowSize = reader.u32();
  const layout: StreamLayoutEntry[] = [];
  for (let i = 0; i < count; i += 1)
    layout.push({ id: reader.u64(), type: reader.u8() as ValueType, valueSize: reader.u8() });
  reader.assertEnd();
  return { specId, rowSize, layout };
}

/**
 * Parse a StreamData payload into one or more stream rows.
 *
 * Wire layout:
 *   type(u8) + specId(u32) + count(u32)
 *   + count × [timestampUs(u64) + layout.length × value(valueSize or varint)]
 *
 * For variable-length entries (`valueSize === 0`), each value is prefixed
 * by a varint length.
 *
 * @param layout The layout returned by {@link readConfigureAck}.
 */
export function decodeStreamData(payload: Uint8Array, layout: StreamLayoutEntry[]): StreamRow[] {
  const reader = new BinaryReader(payload);
  reader.u8(); // type
  const specId = reader.u32();
  const count = reader.u32();
  const rows: StreamRow[] = [];
  for (let row = 0; row < count; row += 1) {
    const timestampUs = reader.u64();
    const values = layout.map((entry) => reader.bytesOf(entry.valueSize || reader.varint()));
    rows.push({ specId, timestampUs, values });
  }
  reader.assertEnd();
  return rows;
}
