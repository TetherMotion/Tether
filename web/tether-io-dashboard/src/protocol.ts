export const MessageType = {
  listParamsReq: 0x01, listParamsResp: 0x02, configureStream: 0x03, configureAck: 0x04,
  startStream: 0x05, stopStream: 0x06, streamData: 0x07, error: 0x08,
  setParameterReq: 0x0b, setParameterResp: 0x0c, listSignalsReq: 0x20,
  listSignalsResp: 0x21, getParamReq: 0x22, getParamResp: 0x23,
  getSignalReq: 0x24, getSignalResp: 0x25, listFunctionsReq: 0x35,
  listFunctionsResp: 0x36, callFunctionReq: 0x37, callFunctionResp: 0x38,
} as const;

export enum ValueType { U8 = 1, U16, U32, U64, I8, I16, I32, I64, F32, F64, Bool, String, Binary, IPv4, IPv6, MAC, Enum, UVarint, IVarint, Struct, Array, Stream }
export interface CatalogEntry { id: bigint; type: ValueType; valueSize: number; flags: number; name: string; description: string; group: string; }
export interface FunctionParameter { name: string; description: string; type: ValueType; flags: number; maxValueSize: number; defaultValue?: Uint8Array; descriptor?: unknown; }
export interface FunctionEntry { id: bigint; name: string; description: string; group: string; parameters: FunctionParameter[]; returnPresent: boolean; returnType?: ValueType; }
export interface StreamLayoutEntry { id: bigint; type: ValueType; valueSize: number; }
export interface StreamRow { specId: number; timestampUs: bigint; values: Uint8Array[]; }

export class BinaryReader {
  private offset = 0;
  private readonly view: DataView;
  constructor(private readonly input: Uint8Array) { this.view = new DataView(input.buffer, input.byteOffset, input.byteLength); }
  get remaining(): number { return this.view.byteLength - this.offset; }
  private take(size: number): number { if (size < 0 || this.remaining < size) throw new Error('truncated packet'); const start = this.offset; this.offset += size; return start; }
  u8(): number { return this.view.getUint8(this.take(1)); }
  u16(): number { return this.view.getUint16(this.take(2), true); }
  u32(): number { return this.view.getUint32(this.take(4), true); }
  u64(): bigint { return this.view.getBigUint64(this.take(8), true); }
  bytesOf(size: number): Uint8Array { const start = this.take(size); return this.input.slice(start, start + size); }
  varint(): number { let value = 0; for (let shift = 0; shift < 35; shift += 7) { const byte = this.u8(); value += (byte & 0x7f) * 2 ** shift; if (!(byte & 0x80)) return value; } throw new Error('invalid varint'); }
  string16(): string { return new TextDecoder().decode(this.bytesOf(this.u16())); }
  assertEnd(): void { if (this.remaining !== 0) throw new Error('trailing packet data'); }
}

export class BinaryWriter {
  private readonly output: Uint8Array; private readonly view: DataView; private offset = 0;
  constructor(size = 256) { this.output = new Uint8Array(size); this.view = new DataView(this.output.buffer); }
  private room(size: number): number { if (this.offset + size > this.output.length) throw new Error('packet too large'); const start = this.offset; this.offset += size; return start; }
  u8(value: number): this { this.view.setUint8(this.room(1), value); return this; }
  u16(value: number): this { this.view.setUint16(this.room(2), value, true); return this; }
  u32(value: number): this { this.view.setUint32(this.room(4), value, true); return this; }
  u64(value: bigint): this { this.view.setBigUint64(this.room(8), value, true); return this; }
  bytes(value: Uint8Array): this { this.output.set(value, this.room(value.length)); return this; }
  varint(value: number): this { do { const byte = value % 128; value = Math.floor(value / 128); this.u8(byte | (value ? 0x80 : 0)); } while (value); return this; }
  finish(): Uint8Array { return this.output.slice(0, this.offset); }
}

export function slipEncode(payload: Uint8Array): Uint8Array { const result: number[] = []; for (const byte of payload) { if (byte === 0xc0) result.push(0xdb, 0xdc); else if (byte === 0xdb) result.push(0xdb, 0xdd); else result.push(byte); } result.push(0xc0); return Uint8Array.from(result); }
export class SlipDecoder {
  private frame: number[] = []; private escaped = false; private discarding = false;
  constructor(private readonly maxFrameSize = 1024 * 1024) {}
  push(chunk: Uint8Array): Uint8Array[] { const frames: Uint8Array[] = []; for (const byte of chunk) { if (this.discarding) { if (byte === 0xc0) { this.discarding = false; this.frame = []; } continue; } if (byte === 0xc0) { if (this.frame.length) frames.push(Uint8Array.from(this.frame)); this.frame = []; this.escaped = false; continue; } if (this.escaped) { if (byte === 0xdc) this.frame.push(0xc0); else if (byte === 0xdd) this.frame.push(0xdb); else { this.frame = []; this.discarding = true; } this.escaped = false; } else if (byte === 0xdb) this.escaped = true; else this.frame.push(byte); if (this.frame.length > this.maxFrameSize) { this.frame = []; this.escaped = false; this.discarding = true; } } return frames; }
}
export function encodeVarint(value: number): Uint8Array { return new BinaryWriter(5).varint(value).finish(); }
export function makeListRequest(type: number, offset = 0, count = 1000): Uint8Array { return new BinaryWriter(9).u8(type).u32(offset).u32(count).finish(); }
export function makeGetRequest(type: number, id: bigint): Uint8Array { return new BinaryWriter(9).u8(type).u64(id).finish(); }
export function decodeValueBytes(bytes: Uint8Array, type: ValueType): number | bigint | boolean | string | Uint8Array { const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength); switch (type) { case ValueType.U8: return view.getUint8(0); case ValueType.U16: return view.getUint16(0, true); case ValueType.U32: return view.getUint32(0, true); case ValueType.U64: return view.getBigUint64(0, true); case ValueType.I8: return view.getInt8(0); case ValueType.I16: return view.getInt16(0, true); case ValueType.I32: return view.getInt32(0, true); case ValueType.I64: return view.getBigInt64(0, true); case ValueType.F32: return view.getFloat32(0, true); case ValueType.F64: return view.getFloat64(0, true); case ValueType.Bool: return view.getUint8(0) !== 0; case ValueType.String: return new TextDecoder().decode(bytes); default: return bytes; } }
export function formatValue(value: ReturnType<typeof decodeValueBytes>): string { if (value instanceof Uint8Array) return `0x${[...value].map(byte => byte.toString(16).padStart(2, '0')).join('')}`; return typeof value === 'bigint' ? value.toString() : String(value); }
export function readEntryCatalog(payload: Uint8Array): CatalogEntry[] { const reader = new BinaryReader(payload); reader.u8(); reader.u32(); reader.u32(); const count = reader.u32(); const entries: CatalogEntry[] = []; for (let i = 0; i < count; i += 1) entries.push({ id: reader.u64(), type: reader.u8() as ValueType, valueSize: reader.u8(), flags: reader.u8(), name: reader.string16(), description: reader.string16(), group: reader.string16() }); reader.assertEnd(); return entries; }
export function readFunctionCatalog(payload: Uint8Array): FunctionEntry[] { const reader = new BinaryReader(payload); reader.u8(); reader.u32(); reader.u32(); const count = reader.u32(); const functions: FunctionEntry[] = []; for (let i = 0; i < count; i += 1) { const id = reader.u64(); const name = reader.string16(); const description = reader.string16(); const group = reader.string16(); const parameters: FunctionParameter[] = []; for (let p = 0, total = reader.u32(); p < total; p += 1) { const parameter = { name: reader.string16(), description: reader.string16(), type: reader.u8() as ValueType, flags: reader.u8(), maxValueSize: 0 } as FunctionParameter; reader.u64(); reader.u64(); parameter.maxValueSize = reader.u32(); if (reader.u8()) reader.bytesOf(reader.u32()); if (parameter.flags & 2) parameter.defaultValue = reader.bytesOf(reader.varint()); for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) { reader.string16(); reader.string16(); } parameters.push(parameter); } const returnPresent = reader.u8() !== 0; let returnType: ValueType | undefined; if (returnPresent) { reader.string16(); reader.string16(); returnType = reader.u8() as ValueType; reader.u8(); reader.u64(); reader.u64(); reader.u32(); if (reader.u8()) reader.bytesOf(reader.u32()); for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) { reader.string16(); reader.string16(); } } for (let m = 0, metadata = reader.u32(); m < metadata; m += 1) { reader.string16(); reader.string16(); } functions.push({ id, name, description, group, parameters, returnPresent, returnType }); } reader.assertEnd(); return functions; }
export function readConfigureAck(payload: Uint8Array): { specId: number; rowSize: number; layout: StreamLayoutEntry[] } { const reader = new BinaryReader(payload); reader.u8(); const specId = reader.u32(); const count = reader.u32(); const rowSize = reader.u32(); const layout: StreamLayoutEntry[] = []; for (let i = 0; i < count; i += 1) layout.push({ id: reader.u64(), type: reader.u8() as ValueType, valueSize: reader.u8() }); reader.assertEnd(); return { specId, rowSize, layout }; }
export function decodeStreamData(payload: Uint8Array, layout: StreamLayoutEntry[]): StreamRow[] { const reader = new BinaryReader(payload); reader.u8(); const specId = reader.u32(); const count = reader.u32(); const rows: StreamRow[] = []; for (let row = 0; row < count; row += 1) { const timestampUs = reader.u64(); const values = layout.map(entry => reader.bytesOf(entry.valueSize || reader.varint())); rows.push({ specId, timestampUs, values }); } reader.assertEnd(); return rows; }
