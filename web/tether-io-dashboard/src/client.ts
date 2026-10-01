/**
 * @file client.ts
 * @brief WebSocket client for the Tether IO binary protocol.
 *
 * `TetherIOClient` wraps a WebSocket and provides typed async methods for
 * every protocol request (list, get, set, configureStream, …).  It also
 * dispatches `EventTarget` events for connection state changes, streamed
 * data, and protocol errors.
 *
 * The transport uses Framing::None — each WebSocket binary message is
 * exactly one protocol message (no SLIP encoding/decoding).
 */

import {
  BinaryReader,
  CatalogEntry,
  FunctionCallArg,
  FunctionCallResponse,
  FunctionEntry,
  InvokeExResponse,
  MessageType,
  StreamLayoutEntry,
  StreamRow,
  decodeStreamData,
  makeGetRequest,
  makeInvokeExRequest,
  makeListRequest,
  readConfigureAck,
  readEntryCatalog,
  readFunctionCatalog,
  readInvokeExResponse,
} from './protocol';
import {
  SchemaCatalogV6,
  SchemaMessage,
  decodeSchemaDefinitionV6,
  decodeServerHelloV6,
  makeClientHelloV6,
  makeSchemaCommitV6,
  makeSchemaRequestV6,
  decodeSchemaValue,
  verifyAndBuildCatalog,
} from './schema-v6';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** A pending request awaiting its matching response frame. */
type Pending = {
  /** Expected response message type (e.g. `MessageType.listParamsResp`). */
  type: number;
  resolve: (payload: Uint8Array) => void;
  reject: (error: Error) => void;
  timer: ReturnType<typeof setTimeout>;
  correlationId?: bigint;
};

type BootstrapWaiter = {
  expected: Set<number>;
  count: number;
  frames: Uint8Array[];
  resolve: (frames: Uint8Array[]) => void;
  reject: (error: Error) => void;
  timer: ReturnType<typeof setTimeout>;
};

/** Coarse connection state for UI display. */
export type ConnectionState = 'disconnected' | 'connecting' | 'connected';

/** A decoded protocol error frame. */
export interface TetherError {
  code: number;
  message: string;
}

export interface SnapshotFrame {
  timestampUs: bigint;
  schemaEpoch: bigint;
  values: Map<bigint, { schemaSlot: number; bytes: Uint8Array }>;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Map a numeric MessageType to a human-readable name for console logging.
 * Falls back to a hex representation for unknown types.
 */
function msgTypeName(type: number): string {
  const names: Record<number, string> = {
    0x01: 'ListParamsReq',
    0x02: 'ListParamsResp',
    0x03: 'ConfigureStream',
    0x04: 'ConfigureAck',
    0x05: 'StartStream',
    0x06: 'StopStream',
    0x07: 'StreamData',
    0x08: 'Error',
    0x0b: 'SetParameterReq',
    0x0c: 'SetParameterResp',
    0x0d: 'PingReq',
    0x0e: 'PongResp',
    0x20: 'ListSignalsReq',
    0x21: 'ListSignalsResp',
    0x22: 'GetParamReq',
    0x23: 'GetParamResp',
    0x24: 'GetSignalReq',
    0x25: 'GetSignalResp',
    0x35: 'ListFunctionsReq',
    0x36: 'ListFunctionsResp',
    0x37: 'CallFunctionReq',
    0x38: 'CallFunctionResp',
  };
  return names[type] ?? `0x${type.toString(16).padStart(2, '0')}`;
}

// ---------------------------------------------------------------------------
// TetherIOClient
// ---------------------------------------------------------------------------

/**
 * WebSocket client for the Tether IO binary protocol.
 *
 * @extends EventTarget
 *
 * @fires connected    — WebSocket is open.
 * @fires disconnected — WebSocket closed or errored.
 * @fires stream       — A StreamData row was received (detail: {@link StreamRow}).
 * @fires error-message — A protocol error was received (detail: {@link TetherError}).
 */
export class TetherIOClient extends EventTarget {
  /** The underlying WebSocket, or undefined when not connected. */
  private socket?: WebSocket;
  /** Queue of requests awaiting their matching response frame. */
  private pending: Pending[] = [];
  private bootstrapWaiters: BootstrapWaiter[] = [];
  private verifiedCatalog?: SchemaCatalogV6;
  /** Layout of the currently configured stream (set by configureStream). */
  private streamLayout: StreamLayoutEntry[] = [];

  /** Current stream layout (one entry per channel, set by configureStream). */
  get currentStreamLayout(): StreamLayoutEntry[] {
    return this.streamLayout;
  }
  /** V6 schemas verified for the current server epoch, if available. */
  get schemaCatalog(): SchemaCatalogV6 | undefined {
    return this.verifiedCatalog;
  }
  /** Whether a stream is currently active (set by startStream/stopStream). */
  private streamActive = false;
  /** Monotonic counter for log correlation. */
  private msgCounter = 0;
  private invokeRequestId = 1n;

  /** Current connection state (readable by the UI). */
  state: ConnectionState = 'disconnected';

  // ---- Connection management --------------------------------------------

  /**
   * Open a WebSocket connection to the Tether IO server.
   *
   * @param url WebSocket URL, e.g. `ws://127.0.0.1:8080/tether-io`.
   * @resolves When the WebSocket is open.
   * @rejects  On connection error.
   */
  connect(url: string): Promise<void> {
    this.disconnect();
    this.state = 'connecting';
    console.log(`[TetherIO] connect → ${url}`);
    return new Promise((resolve, reject) => {
      const socket = new WebSocket(url);
      this.socket = socket;
      socket.binaryType = 'arraybuffer';

      socket.onopen = async () => {
        try {
          console.log('[TetherIO] WebSocket opened; negotiating V6 schemas');
          await this.negotiateV6();
          this.state = 'connected';
          this.dispatchEvent(new Event('connected'));
          resolve();
        } catch (error) {
          const failure = error instanceof Error ? error : new Error('V6 schema negotiation failed');
          this.state = 'disconnected';
          this.rejectPending(failure);
          socket.close(1002, 'V6 schema negotiation failed');
          reject(failure);
        }
      };

      socket.onerror = (e) => {
        this.state = 'disconnected';
        const err = new Error('WebSocket connection failed');
        console.error('[TetherIO] WebSocket error:', e);
        this.rejectPending(err);
        reject(err);
      };

      socket.onclose = (e) => {
        this.state = 'disconnected';
        console.log(`[TetherIO] WebSocket closed (code=${e.code}, reason=${e.reason || '(none)'})`);
        this.rejectPending(new Error('WebSocket closed'));
        this.dispatchEvent(new Event('disconnected'));
      };

      socket.onmessage = async (event) => {
        const bytes =
          event.data instanceof ArrayBuffer
            ? new Uint8Array(event.data)
            : new Uint8Array(await (event.data as Blob).arrayBuffer());
        // Framing::None — each WebSocket binary message is one protocol message.
        this.handleFrame(bytes);
      };
    });
  }

  /** Close the WebSocket and cancel all pending requests. */
  disconnect(): void {
    if (this.socket) {
      console.log('[TetherIO] disconnect');
      this.socket.close();
    }
    this.socket = undefined;
    this.state = 'disconnected';
    this.streamActive = false;
    this.verifiedCatalog = undefined;
  }

  // ---- Catalog: parameters & signals -----------------------------------

  /**
   * List parameters or signals from the server.
   *
   * @param kind  `'params'` or `'signals'`.
   * @param count Maximum number of entries to request.
   * @returns     Array of catalog entries with `kind` set accordingly.
   */
  async list(kind: 'params' | 'signals', count = 1000): Promise<CatalogEntry[]> {
    const type = kind === 'params' ? MessageType.listParamsReq : MessageType.listSignalsReq;
    const respType = kind === 'params' ? MessageType.listParamsResp : MessageType.listSignalsResp;
    console.log(`[TetherIO] list(${kind}, count=${count})`);
    const payload = await this.request(makeListRequest(type, 0, count), respType);
    const entries = readEntryCatalog(payload);
    const entryKind = kind === 'params' ? 'param' : 'signal';
    for (const entry of entries) entry.kind = entryKind;
    console.log(`[TetherIO] list(${kind}) → ${entries.length} entries`);
    return entries;
  }

  /**
   * List remotely callable functions from the server.
   *
   * @param count Maximum number of functions to request.
   */
  async listFunctions(count = 1000): Promise<FunctionEntry[]> {
    console.log(`[TetherIO] listFunctions(count=${count})`);
    const entries = readFunctionCatalog(
      await this.request(
        makeListRequest(MessageType.listFunctionsReq, 0, count),
        MessageType.listFunctionsResp,
      ),
    );
    console.log(`[TetherIO] listFunctions → ${entries.length} entries`);
    return entries;
  }

  /** Load annotations for a catalog entry through the V6 metadata service. */
  async getMetadata(id: bigint): Promise<Record<string, string>> {
    const request = new Uint8Array(9);
    const view = new DataView(request.buffer);
    view.setUint8(0, MessageType.getMetadataReq);
    view.setBigUint64(1, id, true);
    const payload = await this.request(request, MessageType.getMetadataResp);
    const reader = new BinaryReader(payload);
    reader.u8();
    reader.u64();
    const count = reader.u32();
    if (count > 4096) throw new Error('metadata count exceeds limit');
    const metadata: Record<string, string> = {};
    for (let i = 0; i < count; i += 1) metadata[reader.string16()] = reader.string16();
    reader.assertEnd();
    return metadata;
  }

  /**
   * Invoke a remote function.
   *
   * @param id   Function identifier from the catalog.
   * @param args Positional arguments (see {@link FunctionCallArg} /
   *             `encodeScalarArgument`).
   * @returns    The decoded call result; `success === false` carries the
   *             server-side error code/message.  Transport-level failures
   *             reject the promise.
   */
  async callFunction(id: bigint, args: FunctionCallArg[] = []): Promise<FunctionCallResponse> {
    const requestId = this.invokeRequestId++;
    const payload = await this.request(
      makeInvokeExRequest(requestId, id, args),
      MessageType.invokeExResp,
      requestId,
    );
    const response: InvokeExResponse = readInvokeExResponse(payload);
    if (response.requestId !== requestId) throw new Error('mismatched InvokeEx response request ID');
    return { ...response, functionId: id };
  }

  /**
   * Invoke a remote function, throwing on server-side failure.
   *
   * Convenience wrapper around {@link callFunction} for UI code that only
   * cares about the return value.
   */
  async callFunctionOrThrow(id: bigint, args: FunctionCallArg[] = []): Promise<Uint8Array> {
    const result = await this.callFunction(id, args);
    if (!result.success)
      throw new Error(result.errorMessage || `function call failed (${result.error})`);
    return result.returnValue;
  }

  // ---- Read / write values ---------------------------------------------

  /**
   * Read the current value of a parameter or signal.
   *
   * @param kind `'param'` or `'signal'`.
   * @param id   Identifier of the entry to read.
   * @returns    Raw value bytes (decode with `decodeValueBytes`).
   */
  get(kind: 'param' | 'signal', id: bigint): Promise<Uint8Array> {
    const requestType = kind === 'param' ? MessageType.getParamReq : MessageType.getSignalReq;
    const responseType = kind === 'param' ? MessageType.getParamResp : MessageType.getSignalResp;
    console.log(`[TetherIO] get(${kind}, id=${id})`);
    return this.request(makeGetRequest(requestType, id), responseType).then((payload) =>
      this.extractValue(payload),
    );
  }

  /** Read and decode a catalog value only when its negotiated epoch and slot are current. */
  async getTyped(entry: CatalogEntry): Promise<unknown> {
    const catalog = this.verifiedCatalog;
    if (!catalog) throw new Error('schema catalog is not negotiated');
    if (entry.schemaEpoch !== BigInt(catalog.epoch)) throw new Error('catalog entry belongs to a stale schema epoch');
    const bytes = await this.get(entry.kind, entry.id);
    if (this.verifiedCatalog !== catalog) throw new Error('schema catalog changed during value read');
    return decodeSchemaValue(catalog, entry.schemaSlot, bytes);
  }

  /** Read a coherent server-side snapshot for selected catalog entries. */
  async snapshot(kind: 'params' | 'signals', ids: bigint[] = []): Promise<SnapshotFrame> {
    const messageType = kind === 'params' ? MessageType.snapshotParamsReq : MessageType.snapshotSignalsReq;
    const responseType = kind === 'params' ? MessageType.snapshotParamsResp : MessageType.snapshotSignalsResp;
    const writer = new Uint8Array(5 + ids.length * 8);
    const view = new DataView(writer.buffer);
    view.setUint8(0, messageType);
    view.setUint32(1, ids.length, true);
    ids.forEach((id, index) => view.setBigUint64(5 + index * 8, id, true));
    const payload = await this.request(writer, responseType);
    const reader = new BinaryReader(payload);
    reader.u8();
    const timestampUs = reader.u64();
    const count = reader.u32();
    const schemaEpoch = reader.u64();
    if (count > 65536) throw new Error('snapshot contains too many values');
    const values = new Map<bigint, { schemaSlot: number; bytes: Uint8Array }>();
    for (let index = 0; index < count; index += 1) {
      const id = reader.u64();
      const schemaSlot = reader.u32();
      const valueSize = reader.u8();
      const length = valueSize === 0 ? reader.varint() : valueSize;
      if (length > reader.remaining) throw new Error('truncated snapshot value');
      values.set(id, { schemaSlot, bytes: reader.bytesOf(length) });
    }
    reader.assertEnd();
    const catalog = this.verifiedCatalog;
    if (!catalog || schemaEpoch !== BigInt(catalog.epoch)) throw new Error('snapshot belongs to a stale schema epoch');
    return { timestampUs, schemaEpoch, values };
  }

  /**
   * Write a new value to a parameter.
   *
   * @param id       Parameter identifier.
   * @param value    Raw value bytes.
   * @param variable If true, prefix the value with a varint length
   *                 (for variable-length parameters such as String/Binary).
   */
  async setParameter(id: bigint, value: Uint8Array, variable = false): Promise<void> {
    console.log(`[TetherIO] setParameter(id=${id}, ${value.length} bytes)`);
    void variable; // The V6 framing prefixes all schema values, fixed or variable.
    let length = value.length;
    let lengthSize = 1;
    while (length >= 0x80) {
      length = Math.floor(length / 128);
      lengthSize += 1;
    }
    const bytes = new Uint8Array(1 + 8 + lengthSize + value.length);
    const view = new DataView(bytes.buffer);
    view.setUint8(0, MessageType.setParameterReq);
    view.setBigUint64(1, id, true);
    let offset = 9;
    length = value.length;
    while (length >= 0x80) {
      bytes[offset++] = (length % 128) | 0x80;
      length = Math.floor(length / 128);
    }
    bytes[offset++] = length;
    bytes.set(value, offset);
    await this.request(bytes, MessageType.setParameterResp);
  }

  // ---- Streaming --------------------------------------------------------

  /**
   * Configure a stream over the given entry IDs.
   *
   * @param ids        Entry IDs to include in the stream (params and/or signals).
   * @param intervalMs Sampling interval in milliseconds.
   * @param chunkSize  Number of rows per StreamData message.
   * @returns          The stream spec ID assigned by the server.
   */
  configureStream(ids: bigint[], intervalMs = 1, chunkSize = 20): Promise<number> {
    console.log(
      `[TetherIO] configureStream(ids=[${ids.map((id) => id.toString()).join(', ')}], intervalMs=${intervalMs}, chunkSize=${chunkSize})`,
    );
    // Wire layout:
    //   type(u8) + triggerMode(u8) + intervalMs(u32) + chunkSize(u32)
    //   + skipCount(u32) + triggerEntryId(u64) + entryCount(u32)
    //   + entryCount × entryId(u64) + filterCount(u32)
    const out = new Uint8Array(1 + 1 + 4 + 4 + 4 + 8 + 4 + ids.length * 8 + 4);
    const view = new DataView(out.buffer);
    let offset = 0;
    view.setUint8(offset++, MessageType.configureStream);
    view.setUint8(offset++, 0); // triggerMode = Time
    view.setUint32(offset, intervalMs, true);
    offset += 4;
    view.setUint32(offset, chunkSize, true);
    offset += 4;
    view.setUint32(offset, 0, true); // skipCount
    offset += 4;
    view.setBigUint64(offset, 0n, true); // triggerEntryId
    offset += 8;
    view.setUint32(offset, ids.length, true);
    offset += 4;
    for (const id of ids) {
      view.setBigUint64(offset, id, true);
      offset += 8;
    }
    view.setUint32(offset, 0, true); // filterCount
    return this.request(out, MessageType.configureAck).then((payload) => {
      const ack = readConfigureAck(payload);
      this.streamLayout = ack.layout;
      console.log(
        `[TetherIO] configureStream → specId=${ack.specId}, ${ack.layout.length} entries, rowSize=${ack.rowSize}`,
      );
      return ack.specId;
    });
  }

  /**
   * Start streaming.  No response is expected from the server.
   *
   * StreamData frames will be dispatched as `'stream'` events.
   */
  startStream(): Promise<void> {
    console.log('[TetherIO] startStream');
    this.streamActive = true;
    // responseType = -1 means "no response expected".
    return this.request(Uint8Array.from([MessageType.startStream]), -1).then(() => undefined);
  }

  /** Stop the currently active stream.  No response is expected. */
  stopStream(): Promise<void> {
    console.log('[TetherIO] stopStream');
    this.streamActive = false;
    return this.request(Uint8Array.from([MessageType.stopStream]), -1).then(() => undefined);
  }

  // ---- Internal: request/response plumbing ------------------------------

  /**
   * Send a protocol message and (optionally) wait for a response.
   *
   * @param payload      Raw message bytes to send.
   * @param responseType Expected response message type, or `-1` for
   *                     fire-and-forget messages (StartStream, StopStream).
   * @returns            The response payload, or an empty array for
   *                     fire-and-forget messages.
   */
  private request(payload: Uint8Array, responseType: number, correlationId?: bigint): Promise<Uint8Array> {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN)
      return Promise.reject(new Error('Not connected'));

    const seq = ++this.msgCounter;
    const timeoutMs = 10000;
    console.log(
      `[TetherIO] → #${seq} ${msgTypeName(payload[0]!)} (${payload.length} bytes), ` +
        `expecting ${responseType >= 0 ? msgTypeName(responseType) : 'no response'} ` +
        `(timeout ${timeoutMs}ms)`,
    );

    const response = new Promise<Uint8Array>((resolve, reject) => {
      if (responseType >= 0) {
        const timer = setTimeout(() => {
          const index = this.pending.findIndex((item) => item.timer === timer);
          if (index >= 0) this.pending.splice(index, 1);
          const err = new Error(
            `Request timed out (${msgTypeName(payload[0]!)} → ${msgTypeName(responseType)}, ${timeoutMs}ms)`,
          );
          console.error(`[TetherIO] ✗ #${seq} timeout`);
          reject(err);
        }, timeoutMs);
        this.pending.push({ type: responseType, resolve, reject, timer, correlationId });
      } else {
        // Fire-and-forget: resolve immediately after the send completes.
        resolve(new Uint8Array());
      }
    });

    // Framing::None — send raw bytes directly (no SLIP encoding).
    this.socket.send(payload);
    return response;
  }

  /**
   * Dispatch a received frame to the matching pending request, or to the
   * stream/error event listeners.
   *
   * - StreamData frames are decoded and dispatched as `'stream'` events.
   * - Error frames reject the oldest pending request and dispatch an
   *   `'error-message'` event.
   * - All other frames are matched against `pending[].type`.
   */
  private handleFrame(frame: Uint8Array): void {
    if (frame.length === 0) return;
    const type = frame[0]!;

    if (this.resolveBootstrapFrame(type, frame)) return;

    if (type === MessageType.catalogChanged || type === SchemaMessage.update) {
      this.verifiedCatalog = undefined;
      this.dispatchEvent(new CustomEvent('catalog-changed', { detail: { type, frame } }));
      return;
    }

    // Stream data is dispatched out-of-band (not matched to a pending request).
    if (type === MessageType.streamData) {
      try {
        const rows = decodeStreamData(frame, this.streamLayout);
        for (const row of rows)
          this.dispatchEvent(new CustomEvent<StreamRow>('stream', { detail: row }));
        if (rows.length > 0)
          console.log(`[TetherIO] ← StreamData: ${rows.length} row(s), specId=${rows[0]!.specId}`);
      } catch (error) {
        console.error('[TetherIO] ← StreamData decode error:', error);
        this.dispatchEvent(new CustomEvent('error-message', { detail: error }));
      }
      return;
    }

    console.log(`[TetherIO] ← ${msgTypeName(type)} (${frame.length} bytes)`);

    // Match against the first pending request expecting this response type.
    const responseId = type === MessageType.invokeExResp && frame.length >= 9
      ? new DataView(frame.buffer, frame.byteOffset, frame.byteLength).getBigUint64(1, true)
      : undefined;
    const index = this.pending.findIndex((item) => item.type === type &&
      (item.correlationId === undefined || item.correlationId === responseId));
    if (index >= 0) {
      const item = this.pending.splice(index, 1)[0];
      if (item) {
        clearTimeout(item.timer);
        item.resolve(frame);
      }
    } else if (type === MessageType.error) {
      // Unsolicited error: reject the oldest pending request (if any) and
      // dispatch an error event for the UI to display.
      const error = this.decodeError(frame);
      console.error(`[TetherIO] ← Error: code=${error.code} msg=${JSON.stringify(error.message)}`);
      const pending = this.pending.shift();
      if (pending) {
        clearTimeout(pending.timer);
        pending.reject(new Error(error.message));
      }
      this.dispatchEvent(new CustomEvent<TetherError>('error-message', { detail: error }));
    } else {
      console.warn(
        `[TetherIO] ← unsolicited ${msgTypeName(type)} (${frame.length} bytes), ` +
          'no pending request matches',
      );
    }
  }

  private async negotiateV6(): Promise<void> {
    this.verifiedCatalog = undefined;
    const helloPromise = this.waitForBootstrapFrames(
      new Set([SchemaMessage.serverHello, SchemaMessage.reject]),
      1,
    );
    this.sendRaw(makeClientHelloV6());
    const [helloFrame] = await helloPromise;
    if (!helloFrame) throw new Error('missing V6 ServerHello');
    if (helloFrame[0] === SchemaMessage.reject) {
      const reader = new BinaryReader(helloFrame);
      reader.u8();
      const code = reader.u8();
      const message = reader.string32();
      throw new Error(`V6 schema negotiation rejected (${code}): ${message}`);
    }
    const hello = decodeServerHelloV6(helloFrame);
    const definitionsPromise = hello.manifest.length
      ? this.waitForBootstrapFrames(
          new Set([SchemaMessage.definition, SchemaMessage.reject]),
          hello.manifest.length,
        )
      : Promise.resolve([] as Uint8Array[]);
    if (hello.manifest.length) this.sendRaw(makeSchemaRequestV6(hello.epoch, hello.manifest));
    const definitionFrames = await definitionsPromise;
    const definitions = definitionFrames.map((frame) => {
      if (frame[0] === SchemaMessage.reject) {
        const reader = new BinaryReader(frame);
        reader.u8();
        const code = reader.u8();
        throw new Error(`Schema definition rejected (${code}): ${reader.string32()}`);
      }
      return decodeSchemaDefinitionV6(frame);
    });
    const catalog = await verifyAndBuildCatalog(hello, definitions);
    this.sendRaw(makeSchemaCommitV6(hello.epoch));
    this.verifiedCatalog = catalog;
    console.log(`[TetherIO] V6 schema epoch ${hello.epoch} committed (${catalog.manifest.length} schemas)`);
  }

  private sendRaw(payload: Uint8Array): void {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) throw new Error('Not connected');
    this.socket.send(payload);
  }

  private waitForBootstrapFrames(expected: Set<number>, count: number): Promise<Uint8Array[]> {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.bootstrapWaiters = this.bootstrapWaiters.filter((waiter) => waiter.timer !== timer);
        reject(new Error('V6 schema negotiation timed out'));
      }, 10000);
      this.bootstrapWaiters.push({ expected, count, frames: [], resolve, reject, timer });
    });
  }

  private resolveBootstrapFrame(type: number, frame: Uint8Array): boolean {
    const index = this.bootstrapWaiters.findIndex((waiter) => waiter.expected.has(type));
    if (index < 0) return false;
    const waiter = this.bootstrapWaiters[index]!;
    waiter.frames.push(frame);
    if (type === SchemaMessage.reject || waiter.frames.length >= waiter.count) {
      this.bootstrapWaiters.splice(index, 1);
      clearTimeout(waiter.timer);
      waiter.resolve(waiter.frames);
    }
    return true;
  }

  /**
   * Extract the value bytes from a GetParamResp / GetSignalResp payload.
   *
  * Wire layout: `type(u8) + id(u64) + varint length + value`.
   */
  private extractValue(payload: Uint8Array): Uint8Array {
    const reader = new BinaryReader(payload);
    reader.u8(); // type
    reader.u64(); // id
    const length = reader.varint();
    if (length > reader.remaining) throw new Error('truncated value response');
    const value = reader.bytesOf(length);
    reader.assertEnd();
    return value;
  }

  /** Reject all pending requests with the given error (used on disconnect). */
  private rejectPending(error: Error): void {
    const count = this.pending.length;
    this.pending.splice(0).forEach((item) => {
      clearTimeout(item.timer);
      item.reject(error);
    });
    if (count > 0) console.log(`[TetherIO] rejected ${count} pending request(s)`);
    this.bootstrapWaiters.splice(0).forEach((waiter) => {
      clearTimeout(waiter.timer);
      waiter.reject(error);
    });
  }

  /**
   * Decode an Error frame.
   *
   * Wire layout: `type(u8) + code(u32) + message(string16)`.
   */
  private decodeError(payload: Uint8Array): TetherError {
    const reader = new BinaryReader(payload);
    reader.u8(); // type
    const code = reader.u32();
    const message = reader.string16();
    return { code, message };
  }
}
