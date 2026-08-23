import { BinaryReader, CatalogEntry, FunctionEntry, MessageType, SlipDecoder, StreamLayoutEntry, StreamRow, ValueType, decodeStreamData, makeGetRequest, makeListRequest, readConfigureAck, readEntryCatalog, readFunctionCatalog, slipEncode } from './protocol';

type Pending = { type: number; resolve: (payload: Uint8Array) => void; reject: (error: Error) => void; timer: ReturnType<typeof setTimeout> };
export type ConnectionState = 'disconnected' | 'connecting' | 'connected';
export interface TetherError { code: number; message: string; }

export class TetherIoClient extends EventTarget {
  private socket?: WebSocket;
  private decoder = new SlipDecoder();
  private pending: Pending[] = [];
  private streamLayout: StreamLayoutEntry[] = [];
  state: ConnectionState = 'disconnected';

  connect(url: string): Promise<void> {
    this.disconnect(); this.state = 'connecting';
    return new Promise((resolve, reject) => {
      const socket = new WebSocket(url); this.socket = socket; socket.binaryType = 'arraybuffer';
      socket.onopen = () => { this.state = 'connected'; this.dispatchEvent(new Event('connected')); resolve(); };
      socket.onerror = () => { this.state = 'disconnected'; this.rejectPending(new Error('WebSocket connection failed')); reject(new Error('WebSocket connection failed')); };
      socket.onclose = () => { this.state = 'disconnected'; this.rejectPending(new Error('WebSocket closed')); this.dispatchEvent(new Event('disconnected')); };
      socket.onmessage = async event => { const bytes = event.data instanceof ArrayBuffer ? new Uint8Array(event.data) : new Uint8Array(await (event.data as Blob).arrayBuffer()); for (const frame of this.decoder.push(bytes)) this.handleFrame(frame); };
    });
  }

  disconnect(): void { this.socket?.close(); this.socket = undefined; this.state = 'disconnected'; }

  async list(kind: 'params' | 'signals', count = 1000): Promise<CatalogEntry[]> {
    const type = kind === 'params' ? MessageType.listParamsReq : MessageType.listSignalsReq;
    const payload = await this.request(makeListRequest(type, 0, count), kind === 'params' ? MessageType.listParamsResp : MessageType.listSignalsResp);
    return readEntryCatalog(payload);
  }

  async listFunctions(count = 1000): Promise<FunctionEntry[]> { return readFunctionCatalog(await this.request(makeListRequest(MessageType.listFunctionsReq, 0, count), MessageType.listFunctionsResp)); }

  async setParameter(id: bigint, value: Uint8Array, variable = false): Promise<void> { const bytes = new Uint8Array(1 + 8 + (variable ? 5 : 0) + value.length); const view = new DataView(bytes.buffer); view.setUint8(0, MessageType.setParameterReq); view.setBigUint64(1, id, true); let offset = 9; if (variable) { let length = value.length; while (length >= 0x80) { bytes[offset++] = (length & 0x7f) | 0x80; length >>>= 7; } bytes[offset++] = length; } bytes.set(value, offset); await this.request(bytes, MessageType.setParameterResp); }

  get(kind: 'param' | 'signal', id: bigint): Promise<Uint8Array> {
    const requestType = kind === 'param' ? MessageType.getParamReq : MessageType.getSignalReq;
    const responseType = kind === 'param' ? MessageType.getParamResp : MessageType.getSignalResp;
    return this.request(makeGetRequest(requestType, id), responseType).then(payload => this.extractValue(payload));
  }

  configureStream(ids: bigint[], intervalMs = 100, chunkSize = 1): Promise<number> {
    const out = new Uint8Array(1 + 1 + 4 + 4 + 4 + 8 + 4 + ids.length * 8 + 4); const view = new DataView(out.buffer); let offset = 0;
    view.setUint8(offset++, MessageType.configureStream); view.setUint8(offset++, 0); view.setUint32(offset, intervalMs, true); offset += 4; view.setUint32(offset, chunkSize, true); offset += 4; view.setUint32(offset, 0, true); offset += 4; view.setBigUint64(offset, 0n, true); offset += 8; view.setUint32(offset, ids.length, true); offset += 4;
    for (const id of ids) { view.setBigUint64(offset, id, true); offset += 8; } view.setUint32(offset, 0, true);
    return this.request(out, MessageType.configureAck).then(payload => { const ack = readConfigureAck(payload); this.streamLayout = ack.layout; return ack.specId; });
  }

  startStream(): Promise<void> { return this.request(Uint8Array.from([MessageType.startStream]), -1).then(() => undefined); }
  stopStream(): Promise<void> { return this.request(Uint8Array.from([MessageType.stopStream]), -1).then(() => undefined); }

  private request(payload: Uint8Array, responseType: number): Promise<Uint8Array> {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) return Promise.reject(new Error('Not connected'));
    const response = new Promise<Uint8Array>((resolve, reject) => { if (responseType >= 0) { const timer = setTimeout(() => { const index = this.pending.findIndex(item => item.timer === timer); if (index >= 0) this.pending.splice(index, 1); reject(new Error('Request timed out')); }, 10000); this.pending.push({ type: responseType, resolve, reject, timer }); } else resolve(new Uint8Array()); });
    this.socket.send(slipEncode(payload)); return response;
  }

  private handleFrame(frame: Uint8Array): void {
    if (frame.length === 0) return; const type = frame[0];
    if (type === MessageType.streamData) { try { for (const row of decodeStreamData(frame, this.streamLayout)) this.dispatchEvent(new CustomEvent<StreamRow>('stream', { detail: row })); } catch (error) { this.dispatchEvent(new CustomEvent('error-message', { detail: error })); } return; }
    const index = this.pending.findIndex(item => item.type === type);
    if (index >= 0) { const item = this.pending.splice(index, 1)[0]; if (item) { clearTimeout(item.timer); item.resolve(frame); } }
    else if (type === MessageType.error) { const error = this.decodeError(frame); const pending = this.pending.shift(); if (pending) { clearTimeout(pending.timer); pending.reject(new Error(error.message)); } this.dispatchEvent(new CustomEvent<TetherError>('error-message', { detail: error })); }
  }
  private extractValue(payload: Uint8Array): Uint8Array { const reader = new BinaryReader(payload); reader.u8(); reader.u64(); const size = reader.u8(); if (size === 0) reader.varint(); return reader.bytesOf(reader.remaining); }
  private rejectPending(error: Error): void { this.pending.splice(0).forEach(item => { clearTimeout(item.timer); item.reject(error); }); }
  private decodeError(payload: Uint8Array): TetherError { const reader = new BinaryReader(payload); reader.u8(); const code = reader.u32(); const message = reader.string16(); return { code, message }; }
}
