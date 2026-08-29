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
  FunctionEntry,
  MessageType,
  StreamLayoutEntry,
  StreamRow,
  ValueType,
  decodeStreamData,
  makeGetRequest,
  makeListRequest,
  readConfigureAck,
  readEntryCatalog,
  readFunctionCatalog,
} from './protocol';

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
};

/** Coarse connection state for UI display. */
export type ConnectionState = 'disconnected' | 'connecting' | 'connected';

/** A decoded protocol error frame. */
export interface TetherError {
  code: number;
  message: string;
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
  /** Layout of the currently configured stream (set by configureStream). */
  private streamLayout: StreamLayoutEntry[] = [];

  /** Current stream layout (one entry per channel, set by configureStream). */
  get currentStreamLayout(): StreamLayoutEntry[] {
    return this.streamLayout;
  }
  /** Whether a stream is currently active (set by startStream/stopStream). */
  private streamActive = false;
  /** Monotonic counter for log correlation. */
  private msgCounter = 0;

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

      socket.onopen = () => {
        this.state = 'connected';
        console.log('[TetherIO] WebSocket opened');
        this.dispatchEvent(new Event('connected'));
        resolve();
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
    const bytes = new Uint8Array(1 + 8 + (variable ? 5 : 0) + value.length);
    const view = new DataView(bytes.buffer);
    view.setUint8(0, MessageType.setParameterReq);
    view.setBigUint64(1, id, true);
    let offset = 9;
    if (variable) {
      let length = value.length;
      while (length >= 0x80) {
        bytes[offset++] = (length & 0x7f) | 0x80;
        length >>>= 7;
      }
      bytes[offset++] = length;
    }
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
  private request(payload: Uint8Array, responseType: number): Promise<Uint8Array> {
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
        this.pending.push({ type: responseType, resolve, reject, timer });
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
    const index = this.pending.findIndex((item) => item.type === type);
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

  /**
   * Extract the value bytes from a GetParamResp / GetSignalResp payload.
   *
   * Wire layout: `type(u8) + id(u64) + valueSize(u8) + [varint length] + value`.
   * For fixed-size entries, `valueSize` is the byte count.
   * For variable-length entries, `valueSize` is 0 and a varint length follows.
   */
  private extractValue(payload: Uint8Array): Uint8Array {
    const reader = new BinaryReader(payload);
    reader.u8(); // type
    reader.u64(); // id
    const size = reader.u8();
    if (size === 0) reader.varint(); // variable-length prefix
    return reader.bytesOf(reader.remaining);
  }

  /** Reject all pending requests with the given error (used on disconnect). */
  private rejectPending(error: Error): void {
    const count = this.pending.length;
    this.pending.splice(0).forEach((item) => {
      clearTimeout(item.timer);
      item.reject(error);
    });
    if (count > 0) console.log(`[TetherIO] rejected ${count} pending request(s)`);
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
